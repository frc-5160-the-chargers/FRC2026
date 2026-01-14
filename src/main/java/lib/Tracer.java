// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.function.Supplier;

/**
 * A Utility class for tracing code execution time. Data can be found under the "Tracer" table.
 *
 * <p>Example inside {@code Robot.java}
 *
 * <pre><code>
 * public void robotPeriodic() {
 * 	 // Calls CommandScheduler.getInstance.run() while measuring it's total loop time.
 *   Tracer.trace("CommandScheduler", CommandScheduler.getInstance()::run);
 *   Tracer.trace("SignalUpdate", SignalBatchRefresher::refreshAll);
 *   Tracer.endCycle(); // This MUST be called after all Tracer.trace() calls.
 * }
 * </code></pre>
 *
 * <p>Note that {@link Tracer#trace}, {@link Tracer#startTrace} and {@link Tracer#endTrace}
 * will not work in replay. However, you can use {@link Tracer#fetchTimeMs} to fetch already
 * logged tracer times to set breakpoints, log latency deltas, and more in replay mode.
 */
public class Tracer {
	/**
	 * Logs data recorded by the tracer and cleans up stale entries.
	 * <h3>This must be the very last statement in robotPeriodic().
	 */
	public static void endCycle() {
		if (logger == null) return;
		if (!traceStack.isEmpty()) {
			DriverStation.reportWarning("Tracer.endCycle() must be run after ALL Tracer.trace() calls.", false);
			return;
		}
		Logger.processInputs("Tracer", loggerFetcher);
		// update times for all already existing entries
		for (var entry : entryArray) {
			// if the entry isn't found, time will null-cast to 0.0
			Double time = traceTimes.remove(entry);
			if (time == null) time = 0.0;
			logger.put(entry, time);
		}
		// log all new entries
		for (var traceTime : traceTimes.entrySet()) {
			logger.put(traceTime.getKey(), traceTime.getValue());
			entryArray.add(traceTime.getKey());
		}

		// clean up state
		traceTimes.clear();
		traceStackHistory.clear();
	}

	/**
	 * Runs and traces a function.
	 *
	 * @param name the name of the trace, should be unique to the function.
	 * @param functionToTrace the function to trace.
	 * @apiNote If you want to return a value then use {@link Tracer#trace(String, Supplier)}.
	 */
	public static void trace(String name, Runnable functionToTrace) {
		try {
			startTrace(name);
			functionToTrace.run();
		} finally {
			endTrace();
		}
	}

	/**
	 * Runs and traces a function.
	 *
	 * @param name the name of the trace, should be unique to the function.
	 * @param functionToTrace the function to trace.
	 */
	public static <T> T trace(String name, Supplier<T> functionToTrace) {
		try {
			return functionToTrace.get();
		} finally {
			endTrace();
		}
	}

	/**
	 * Returns a command that has it's execute() method traced.
	 *
	 * @param name the name of the trace, should be unique to the function.
	 * @param command the command to trace.
	 */
	public static Command trace(String name, Command command) {
		return new WrapperCommand(command) {
			@Override
			public void execute() {
				startTrace(name);
				super.execute();
				endTrace();
			}

			@Override
			public String getName() {
				return name;
			}
		};
	}

	/**
	 * Starts a trace, should be called at the beginning of a function that's not being called by user
	 * code.
	 *
	 * @param name the name of the trace, should be unique to the function.
	 */
	public static void startTrace(String name) {
		if (RobotMode.get() == RobotMode.REPLAY) return;
		String stack = appendTraceStack(name);
		TraceStartData data = traceStartTimes.get(stack);
		if (data == null) {
			data = new TraceStartData();
			traceStartTimes.put(stack, data);
		}
		data.set(RobotController.getFPGATime() / 1000.0, totalGCTime());
	}

	/**
	 * Ends a trace, should only be called at the end of a function that's not being called by user
	 * code. If a {@link Tracer#startTrace(String)} is not paired with an endTrace() call
	 * there could be dropped or incorrect data.
	 */
	public static void endTrace() {
		if (RobotMode.get() == RobotMode.REPLAY) return;
		try {
			String stack = popTraceStack();
			var startData = traceStartTimes.get(stack);
			double gcTimeSinceStart = totalGCTime() - startData.startGCTotalTime;
			traceTimes.put(
				stack, RobotController.getFPGATime() / 1000.0 - startData.startTime - gcTimeSinceStart);
		} catch (Exception e) {
			DriverStation.reportError(
				"[Tracer] An end trace was called with no opening trace " + e, true);
		}
	}

	/**
	 * Fetches a trace time that has been logged. This works in replay mode.
	 * <pre><code>
	 *     Tracer.trace("AAA", () -> {
	 *         Tracer.trace("BBB", () -> {
	 *             // run something here
	 *         });
	 *     });
	 *     double bbbTime = Tracer.fetchTimeMs("AAA/BBB"); // Fetches the time elapsed of BBB
	 *     double aaaTime = Tracer.fetchTimeMs("AAA"); // Fetches the time elapsed of AAA
	 *     Tracer.endCycle();
	 * </code></pre>
	 */
	public static double fetchTimeMs(String key) {
		return logger.get(key, 0.0);
	}

	// Internal Implementation
	private static final class TraceStartData {
		private double startTime = 0.0;
		private double startGCTotalTime = 0.0;

		private void set(double startTime, double startGCTotalTime) {
			this.startTime = startTime;
			this.startGCTotalTime = startGCTotalTime;
		}
	}

	// the stack of traces, every startTrace will add to this stack
	// and every endTrace will remove from this stack
	private static final ArrayList<String> traceStack = new ArrayList<>();
	// ideally we only need `traceStack` but in the interest of memory optimization
	// and string concatenation speed we store the history of the stack to reuse the stack names
	private static final ArrayList<String> traceStackHistory = new ArrayList<>();
	// the time of each trace, the key is the trace name, modified every endTrace
	private static final HashMap<String, Double> traceTimes = new HashMap<>();
	// the start time of each trace and the gc time at the start of the trace,
	// the key is the trace name, modified every startTrace and endTrace.
	private static final HashMap<String, TraceStartData> traceStartTimes = new HashMap<>();
	private static final HashSet<String> entryArray = new HashSet<>();

	// the garbage collector beans
	private static final ArrayList<GarbageCollectorMXBean> gcs =
		new ArrayList<>(ManagementFactory.getGarbageCollectorMXBeans());

	// The logger that the tracer records data to.
	private static LogTable logger = null;
	// Here, we abuse LoggableInputs to supply the LogTable for logging/replay purposes.
	private static final LoggableInputs loggerFetcher = new LoggableInputs() {
		@Override public void toLog(LogTable logTable) { logger = logTable; }
		@Override public void fromLog(LogTable logTable) { logger = logTable; }
	};

	static {
		Logger.processInputs("Tracer", loggerFetcher);
	}

	private static String appendTraceStack(String trace) {
		traceStack.add(trace);
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < traceStack.size(); i++) {
			sb.append(traceStack.get(i));
			if (i < traceStack.size() - 1) {
				sb.append("/");
			}
		}
		String str = sb.toString();
		traceStackHistory.add(str);
		return str;
	}

	private static String popTraceStack() {
		traceStack.remove(traceStack.size() - 1);
		return traceStackHistory.remove(traceStackHistory.size() - 1);
	}

	private static double totalGCTime() {
		double gcTime = 0;
		for (GarbageCollectorMXBean gc : gcs) {
			gcTime += gc.getCollectionTime();
		}
		return gcTime;
	}
}