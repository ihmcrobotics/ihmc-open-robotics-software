package us.ihmc.darpaRoboticsChallenge.controllers;

// FIXME Extract these constant to be robot agnostic.
public enum StraightLinePathType
{
   STRAIGHT, REVERSE, LEFT_SHUFFLE, RIGHT_SHUFFLE;

   private static final String STRAIGHT_PATH_NAME = "Forward Path";
   private static final double STRAIGHT_STEP_LENGTH = 0.4; // For Steppr: 0.30;
   private static final double STRAIGHT_STEP_WIDTH = 0.25; // For Steppr: 0.35;
   private static final String REVERSE_PATH_NAME = "Reverse Path";
   private static final double REVERSE_ANGLE = Math.PI;
   private static final double REVERSE_STEP_LENGTH = 0.15;
   private static final double REVERSE_STEP_WIDTH = 0.25; // For Steppr: 0.35;
   private static final String RIGHT_SHUFFLE_PATH_NAME = "Right Shuffle Path";
   private static final String LEFT_SHUFFLE_PATH_NAME = "Left Shuffle Path";
   private static final double SHUFFLE_STEP_LENGTH = 0.25;  // For Steppr: 0.3;
   private static final double SHUFFLE_STEP_WIDTH = 0.21;
   private static final double LEFT_SHUFFLE_ANGLE = -Math.PI / 2;

   public String getTypeName()
   {
      String name = null;
      switch (this)
      {
         case LEFT_SHUFFLE :
            name = LEFT_SHUFFLE_PATH_NAME;

            break;

         case REVERSE :
            name = REVERSE_PATH_NAME;

            break;

         case RIGHT_SHUFFLE :
            name = RIGHT_SHUFFLE_PATH_NAME;

            break;

         case STRAIGHT :
            name = STRAIGHT_PATH_NAME;

            break;

         default :
            break;

      }

      return name;
   }

   public double getAngle()
   {
      double angle = Double.NaN;
      switch (this)
      {
         case LEFT_SHUFFLE :
            angle = LEFT_SHUFFLE_ANGLE;

            break;

         case REVERSE :
            angle = REVERSE_ANGLE;

            break;

         case RIGHT_SHUFFLE :
            angle = -LEFT_SHUFFLE_ANGLE;

            break;

         case STRAIGHT :
            angle = 0.0;

            break;

         default :
            break;

      }

      return angle;
   }

   public double getStepWidth(double stepLengthMultiplier)
   {
      double width = Double.NaN;
      switch (this)
      {
         case LEFT_SHUFFLE :
            width = SHUFFLE_STEP_WIDTH;

            break;

         case REVERSE :
            width = REVERSE_STEP_WIDTH;

            break;

         case RIGHT_SHUFFLE :
            width = SHUFFLE_STEP_WIDTH;

            break;

         case STRAIGHT :
            width = STRAIGHT_STEP_WIDTH;

            break;

         default :
            break;

      }

      return width;
   }

   public double getStepLength(double stepLengthMultiplier)
   {
      double length = Double.NaN;
      switch (this)
      {
         case LEFT_SHUFFLE :
            length = stepLengthMultiplier * SHUFFLE_STEP_LENGTH;

            break;

         case REVERSE :
            length = stepLengthMultiplier * REVERSE_STEP_LENGTH;

            break;

         case RIGHT_SHUFFLE :
            length = stepLengthMultiplier * SHUFFLE_STEP_LENGTH;

            break;

         case STRAIGHT :
            length = stepLengthMultiplier * STRAIGHT_STEP_LENGTH;

            break;

         default :
            break;

      }

      return length;
   }
}
