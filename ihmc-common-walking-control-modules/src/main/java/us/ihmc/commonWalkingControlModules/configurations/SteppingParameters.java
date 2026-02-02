package us.ihmc.commonWalkingControlModules.configurations;

public class SteppingParameters
{
   private final double DEFAULT_ANKLE_HEIGHT = 0.06;

   private final double DEFAULT_FOOT_WIDTH_REDUCTION = 0.015;
   private final double DEFAULT_FOOT_LENGTH_REDUCTION = 0.01;

   private final double DEFAULT_ACTUAL_FOOT_LENGTH = 0.197;
   private final double DEFAULT_ACTUAL_FOOT_WIDTH = 0.09;
   private final double DEFAULT_FOOT_BACK = 0.05;

   private final double FOOT_FORWARD = DEFAULT_ACTUAL_FOOT_LENGTH - DEFAULT_FOOT_BACK;
   private final double FOOT_FORWARD_FOR_CONTROL = FOOT_FORWARD - DEFAULT_FOOT_LENGTH_REDUCTION / 2.0;
   private final double FOOT_BACK_FOR_CONTROL = DEFAULT_FOOT_BACK - DEFAULT_FOOT_LENGTH_REDUCTION / 2.0;
   private final double FOOT_WIDTH_FOR_CONTROL = DEFAULT_ACTUAL_FOOT_WIDTH - DEFAULT_FOOT_WIDTH_REDUCTION;
   private final double FOOT_LENGTH_FOR_CONTROL = DEFAULT_ACTUAL_FOOT_LENGTH - DEFAULT_FOOT_LENGTH_REDUCTION;

   public double getFootForwardOffset()
   {
      return FOOT_FORWARD_FOR_CONTROL;
   }

   public double getFootBackwardOffset()
   {
      return FOOT_BACK_FOR_CONTROL;
   }

   public double getFootWidth()
   {
      return FOOT_WIDTH_FOR_CONTROL;
   }

   public double getToeWidth()
   {
      return FOOT_WIDTH_FOR_CONTROL;
   }

   public double getFootLength()
   {
      return FOOT_LENGTH_FOR_CONTROL;
   }

   public double getActualFootWidth()
   {
      return DEFAULT_ACTUAL_FOOT_WIDTH;
   }

   public double getActualFootLength()
   {
      return DEFAULT_ACTUAL_FOOT_LENGTH;
   }

   public double getMaxStepLength()
   {
      return 0.7;
   }

   public double getMaxBackwardStepLength()
   {
      return 0.5;
   }

   public double getDefaultStepLength()
   {
      return 0.4;
   }

   public double getMaxStepWidth()
   {
      return 0.8;
   }

   public double getMinStepWidth()
   {
      return 0.12;
   }

   public double getInPlaceWidth()
   {
      return 0.22;
   }

   public double getMaxStepUp()
   {
      return 0.25;
   }

   public double getMaxStepDown()
   {
      return 0.2;
   }

   public double getTurningStepWidth()
   {
      return 0.2;
   }

   /**
    * Returns the maximum angle the foot can turn outwards in a step.
    */
   public double getMaxAngleTurnOutwards()
   {
      return 0.65;
   }

   /**
    * Returns the maximum angle the foot can turn inwards in a step.
    * <ul>
    * <li>zero indicates that the maximum inward rotation of the foot is with the foot pointing straight forward.
    * <li>a positive value indicates a limited range of motion such that the foot cannot rotate inward.
    * <li>a negative value indicates an extended range of motion such that the foot can point to the inside.
    * </ul>
    */
   public double getMaxAngleTurnInwards()
   {
      return 0.0;
   }
}
