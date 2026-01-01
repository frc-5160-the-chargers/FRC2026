// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

import static robot.vision.VisionConsts.FIELD_LAYOUT;

/**
 * Assists the robot in pathfinding to a pose on the field, while avoiding obstacles.
 * This is accomplished by simulating a "force" around every obstacle, effectively
 * "repulsing" the robot away from them.
 */
public class RepulsorFieldPlanner {
	private static final double SOURCE_X = 1.75;
	private static final double SOURCE_Y = 1.25;

	private static final int ARROWS_X = 40;
	private static final int ARROWS_Y = 20;

	private static final List<Obstacle> FIELD_OBSTACLES =
		List.of(
			// Reef
			new TeardropObstacle(
				new Translation2d(4.495, 4),
				0.9, 2.5,
				.83, 3.2, 2
			),
			new TeardropObstacle(
				new Translation2d(13.08, 4),
				0.9, 2.5,
				.83, 3.2, 2
			),
			// Walls
			new HorizontalObstacle(0.0, 0.5, .5, true),
			new HorizontalObstacle(FIELD_LAYOUT.getFieldWidth(), 0.5, .5, false),
			new VerticalObstacle(0.0, 0.5, .5, true),
			new VerticalObstacle(FIELD_LAYOUT.getFieldLength(), 0.5, .5, false),
			// Sources
			new LineObstacle(
				new Translation2d(0, SOURCE_Y),
				new Translation2d(SOURCE_X, 0),
				.5, .5
			),
			new LineObstacle(
				new Translation2d(0, FIELD_LAYOUT.getFieldWidth() - SOURCE_Y),
				new Translation2d(SOURCE_X, FIELD_LAYOUT.getFieldWidth()),
				.5, .5
			),
			new LineObstacle(
				new Translation2d(FIELD_LAYOUT.getFieldLength(), SOURCE_Y),
				new Translation2d(FIELD_LAYOUT.getFieldLength() - SOURCE_X, 0),
				.5, .5
			),
			new LineObstacle(
				new Translation2d(FIELD_LAYOUT.getFieldLength(), FIELD_LAYOUT.getFieldWidth() - SOURCE_Y),
				new Translation2d(FIELD_LAYOUT.getFieldLength() - SOURCE_X, FIELD_LAYOUT.getFieldWidth()),
				.5, .5
			)
		);

	private static final double[] EMPTY_DOUBLE_ARRAY = new double[4];

	public abstract static class Obstacle {
		double strength;
		boolean positive;
		
		public Obstacle(double strength, boolean positive) {
			this.strength = strength;
			this.positive = positive;
		}
		
		public abstract Translation2d getForceAtPosition(Translation2d position, Translation2d target);
		
		protected double distToForceMag(double dist, double maxRange) {
			if (Math.abs(dist) > maxRange) {
				return 0;
			}
			if (MathUtil.isNear(0, dist, 1e-2)) {
				dist = 1e-2;
			}
			var forceMag = strength / (dist * dist);
			forceMag -= strength / (maxRange * maxRange);
			forceMag *= positive ? 1 : -1;
			return forceMag;
		}
	}

	public static class TeardropObstacle extends Obstacle {
		final Translation2d loc;
		final double primaryMaxRange;
		final double primaryRadius;
		final double tailStrength;
		final double tailLength;
		
		public TeardropObstacle(
			Translation2d loc,
			double primaryStrength,
			double primaryMaxRange,
			double primaryRadius,
			double tailStrength,
			double tailLength) {
			super(primaryStrength, true);
			this.loc = loc;
			this.primaryMaxRange = primaryMaxRange;
			this.primaryRadius = primaryRadius;
			this.tailStrength = tailStrength;
			this.tailLength = tailLength + primaryMaxRange;
		}
		
		public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
			var targetToLoc = loc.minus(target);
			var targetToLocAngle = targetToLoc.getAngle();
			var sidewaysPoint = new Translation2d(tailLength, targetToLoc.getAngle()).plus(loc);
			
			var positionToLocation = position.minus(loc);
			var positionToLocationDistance = positionToLocation.getNorm();
			Translation2d outwardsForce;
			if (positionToLocationDistance <= primaryMaxRange) {
				outwardsForce =
					new Translation2d(
						distToForceMag(
							Math.max(positionToLocationDistance - primaryRadius, 0),
							primaryMaxRange - primaryRadius),
						positionToLocation.getAngle());
			} else {
				outwardsForce = Translation2d.kZero;
			}
			
			var positionToLine = position.minus(loc).rotateBy(targetToLocAngle.unaryMinus());
			var distanceAlongLine = positionToLine.getX();
			
			Translation2d sidewaysForce;
			var distanceScalar = distanceAlongLine / tailLength;
			if (distanceScalar >= 0 && distanceScalar <= 1) {
				var secondaryMaxRange =
					MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
				var distanceToLine = Math.abs(positionToLine.getY());
				if (distanceToLine <= secondaryMaxRange) {
					double strength;
					if (distanceAlongLine < primaryMaxRange) {
						strength = tailStrength * (distanceAlongLine / primaryMaxRange);
					} else {
						strength =
							-tailStrength * distanceAlongLine / (tailLength - primaryMaxRange)
								+ tailLength * tailStrength / (tailLength - primaryMaxRange);
					}
					strength *= 1 - distanceToLine / secondaryMaxRange;
					
					var sidewaysMag = tailStrength * strength * (secondaryMaxRange - distanceToLine);
					// flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
					var sidewaysTheta =
						target.minus(position).getAngle().minus(position.minus(sidewaysPoint).getAngle());
					sidewaysForce =
						new Translation2d(
							sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians())),
							targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg));
				} else {
					sidewaysForce = Translation2d.kZero;
				}
			} else {
				sidewaysForce = Translation2d.kZero;
			}
			
			return outwardsForce.plus(sidewaysForce);
		}
	}

	public static class HorizontalObstacle extends Obstacle {
		final double y;
		final double maxRange;
		
		public HorizontalObstacle(double y, double strength, double maxRange, boolean positive) {
			super(strength, positive);
			this.y = y;
			this.maxRange = maxRange;
		}
		
		public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
			var dist = Math.abs(position.getY() - y);
			if (dist > maxRange) {
				return Translation2d.kZero;
			}
			return new Translation2d(0, distToForceMag(y - position.getY(), maxRange));
		}
	}

	public static class VerticalObstacle extends Obstacle {
		final double x;
		final double maxRange;
		
		public VerticalObstacle(double x, double strength, double maxRange, boolean positive) {
			super(strength, positive);
			this.x = x;
			this.maxRange = maxRange;
		}
		
		public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
			var dist = Math.abs(position.getX() - x);
			if (dist > maxRange) {
				return Translation2d.kZero;
			}
			return new Translation2d(distToForceMag(x - position.getX(), maxRange), 0);
		}
	}

	public static class LineObstacle extends Obstacle {
		final Translation2d startPoint;
		final Translation2d endPoint;
		final double length;
		final Rotation2d angle;
		final Rotation2d inverseAngle;
		final double maxRange;
		
		public LineObstacle(Translation2d start, Translation2d end, double strength, double maxRange) {
			super(strength, true);
			startPoint = start;
			endPoint = end;
			var delta = end.minus(start);
			length = delta.getNorm();
			angle = delta.getAngle();
			inverseAngle = angle.unaryMinus();
			this.maxRange = maxRange;
		}
		
		public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
			var positionToLine = position.minus(startPoint).rotateBy(inverseAngle);
			if (positionToLine.getX() > 0 && positionToLine.getX() < length) {
				return new Translation2d(
					Math.copySign(distToForceMag(positionToLine.getY(), maxRange), positionToLine.getY()),
					angle.rotateBy(Rotation2d.kCCW_90deg));
			}
			Translation2d closerPoint;
			if (positionToLine.getX() <= 0) {
				closerPoint = startPoint;
			} else {
				closerPoint = endPoint;
			}
			return new Translation2d(
				distToForceMag(position.getDistance(closerPoint), maxRange),
				position.minus(closerPoint).getAngle());
		}
	}

	private final List<Obstacle> obstacles = new ArrayList<>(FIELD_OBSTACLES);

	/**
	 * Takes in the current position of the target, and returns the next pose
	 * and velocities needed to reach the goal pose.
	 */
	public SwerveSample sampleField(
		Translation2d currentTranslation,
		Pose2d goalPose,
		double maxSpeedMps,
		double slowdownDistMeters
	) {
		var goalTrans = goalPose.getTranslation();
		var err = currentTranslation.minus(goalTrans);
		var netForce = getForce(currentTranslation, goalTrans);

		double stepSize_m;
		if (err.getNorm() < slowdownDistMeters) {
			stepSize_m =
				MathUtil.interpolate(
					0, maxSpeedMps * 0.02, err.getNorm() / slowdownDistMeters);
		} else {
			stepSize_m = maxSpeedMps * 0.02;
		}
		var step = new Translation2d(stepSize_m, netForce.getAngle());
		return new SwerveSample(
			0.0,
			currentTranslation.getX() + step.getX(),
			currentTranslation.getY() + step.getY(),
			goalPose.getRotation().getRadians(),
			step.getX() / 0.02,
			step.getY() / 0.02,
			0.0, 0.0, 0.0, 0.0,
			EMPTY_DOUBLE_ARRAY, EMPTY_DOUBLE_ARRAY
		);
	}

	public Pose2d[] getArrows(Pose2d goal) {
		var list = new ArrayList<Pose2d>();
		for (int x = 0; x <= ARROWS_X; x++) {
			for (int y = 0; y <= ARROWS_Y; y++) {
				var translation =
					new Translation2d(x * FIELD_LAYOUT.getFieldLength() / ARROWS_X, y * FIELD_LAYOUT.getFieldWidth() / ARROWS_Y);
				var force = getObstacleForce(translation, goal.getTranslation());
				if (force.getNorm() > 1e-6) {
					var rotation = force.getAngle();
					list.add(new Pose2d(translation, rotation));
				}
			}
		}
		return list.toArray(new Pose2d[0]);
	}

	public void addObstacle(Obstacle obstacle) {
		obstacles.add(obstacle);
	}

	public void removeObstacle(Obstacle obstacle) {
		obstacles.remove(obstacle);
	}
	
	private Translation2d getGoalForce(Translation2d curLocation, Translation2d goal) {
		var displacement = goal.minus(curLocation);
		if (displacement.getNorm() == 0) {
			return Translation2d.kZero;
		}
		var direction = displacement.getAngle();
		var mag = (1 + 1.0 / (1e-6 + displacement.getNorm()));
		return new Translation2d(mag, direction);
	}

	private Translation2d getObstacleForce(Translation2d curLocation, Translation2d target) {
		var force = Translation2d.kZero;
		for (Obstacle obs : obstacles) {
			force = force.plus(obs.getForceAtPosition(curLocation, target));
		}
		return force;
	}

	private Translation2d getForce(Translation2d curLocation, Translation2d target) {
		return getGoalForce(curLocation, target).plus(getObstacleForce(curLocation, target));
	}
}