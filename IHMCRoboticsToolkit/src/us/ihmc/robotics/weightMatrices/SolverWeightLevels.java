package us.ihmc.robotics.weightMatrices;

public abstract class SolverWeightLevels
{
   public final static double HARD_CONSTRAINT = Double.POSITIVE_INFINITY;
   public final static double VERY_HIGH = 50.0;
   public final static double HIGH = 10.0;
   public final static double LOW = 0.10;
   public final static double VERY_LOW = 0.05;
   public final static double MEDIUM = 1.00;

   public static final double CHEST_WEIGHT = HIGH;
   public static final double PELVIS_WEIGHT = MEDIUM;
   public static final double ARM_JOINTSPACE_WEIGHT = MEDIUM;
   public static final double HAND_TASKSPACE_WEIGHT = MEDIUM;
   public static final double HEAD_WEIGHT = 200.0;
   public static final double FOOT_SWING_WEIGHT = HIGH;
   public static final double FOOT_SUPPORT_WEIGHT = VERY_HIGH;

   public static final double MOMENTUM_WEIGHT = VERY_LOW;
}
