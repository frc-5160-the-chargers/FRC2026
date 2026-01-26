package lib.commands;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.RequiredArgsConstructor;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

public class TriggerUtil {
	/** "Binds" an alert to a trigger/boolean supplier; pushing it to the dashboard when the condition returns true. */
	public static void bind(Alert alert, BooleanSupplier condition) {
		CommandScheduler.getInstance().getActiveButtonLoop()
			.bind(() -> alert.set(condition.getAsBoolean()));
	}

	/**
	 * Creates a new trigger that stays true for only a single loop cycle,
	 * before going back down to false.
	 */
	public static Trigger risingEdge(BooleanSupplier receiver) {
		var previous = new AtomicBoolean(receiver.getAsBoolean());
		return new Trigger(() -> {
			boolean current = receiver.getAsBoolean();
			boolean risingEdge = current && !previous.get();
			previous.set(current);
			return risingEdge;
		});
	}
}
