package us.ihmc.robotics.trajectories;

public class TwoWaypointTrajectoryGeneratorParameters
{
   private static final double MINIMUM_GROUND_CLEARANCE = 0.05;
   private static final double DEFAULT_GROUND_CLEARANCE = 0.10; //0.08; //0.12;
   private static final double MINIMUM_DESIRED_PROPORTION_OF_ARC_LENGTH_TAKEN_AT_CONSTANT_SPEED = 0.35;
   private static final double[] DEFAULT_PROPORTIONS_THROUGH_TRAJECTORY_FOR_GROUND_CLEARANCE = new double[] {0.15, 1.0 - 0.15};
   private static final double[] STEP_OR_OFF_PROPORTIONS_THROUGH_TRAJECTORY_FOR_GROUND_CLEARANCE = new double[] {0.15, 0.85};
   private static final double maxHorizontalOffsetForWaypoints = 0.04;

   // Specific to push recovery
   private static final double[] PUSH_RECOVERY_GROUND_CLEARANCES = new double[] {0.05, 0.06};
   private static final double[] PUSH_RECOVERY_PROPORTIONS_THROUGH_TRAJECTORY_FOR_GROUND_CLEARANCE = new double[] {0.05, 1.0 - 0.15};

   public TwoWaypointTrajectoryGeneratorParameters()
   {
   }

   public static double getMinimumGroundClearance()
   {
      return MINIMUM_GROUND_CLEARANCE;
   }

   public static double[] getPushRecoveryGroundClearances()
   {
      return PUSH_RECOVERY_GROUND_CLEARANCES;
   }

   public static double getDefaultGroundClearance()
   {
      return DEFAULT_GROUND_CLEARANCE;
   }

   public static double getMinimumDesiredProportionOfArcLengthTakenAtConstantSpeed()
   {
      return MINIMUM_DESIRED_PROPORTION_OF_ARC_LENGTH_TAKEN_AT_CONSTANT_SPEED;
   }

   public static double[] getDefaultProportionsThroughTrajectoryForGroundClearance()
   {
      return DEFAULT_PROPORTIONS_THROUGH_TRAJECTORY_FOR_GROUND_CLEARANCE;
   }

   public static double[] getStepOnOrOffProportionsThroughTrajectoryForGroundClearance()
   {
      return STEP_OR_OFF_PROPORTIONS_THROUGH_TRAJECTORY_FOR_GROUND_CLEARANCE;
   }

   public static double[] getPushRecoveryProportionsThroughTrajectoryForGroundClearance()
   {
      return PUSH_RECOVERY_PROPORTIONS_THROUGH_TRAJECTORY_FOR_GROUND_CLEARANCE;
   }

   public static double getMaxHorizontalOffsetForWaypoints()
   {
      return maxHorizontalOffsetForWaypoints;
   }
}
