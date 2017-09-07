package us.ihmc.robotics.robotSide;

import java.util.Random;

public enum RobotEnd
{
   HIND, FRONT;

   public static final RobotEnd[] values = values();

   public static RobotEnd generateRandomRobotEnd(Random random)
   {
      if (random.nextBoolean())
      {
         return FRONT;
      }
      else
      {
         return HIND;
      }
   }

   public RobotEnd getOppositeEnd()
   {
      if (this == HIND)
         return FRONT;
      else
         return HIND;
   }

   public String getShortLowerCaseName()
   {
      return getCamelCaseNameForMiddleOfExpression().substring(0, 1).toLowerCase();
   }

   public String getCamelCaseNameForStartOfExpression()
   {
      if (this == HIND)
         return "hind";
      else
         return "front";
   }

   public String getCamelCaseNameForMiddleOfExpression()
   {
      if (this == HIND)
         return "Hind";
      else
         return "Front";
   }

   public String getLowerCaseName()
   {
      if (this == HIND)
         return "hind";
      else
         return "front";
   }

   public String getSideNameInAllCaps()
   {
      if (this == HIND)
         return "HIND";
      else
         return "FRONT";
   }

   public String getEndNameFirstLetter()
   {
      if (this == HIND)
         return "H";
      else
         return "F";
   }

   public double negateIfFrontEnd(double value)
   {
      if (this == FRONT)
         return -value;

      return value;
   }

   public double negateIfHindEnd(double value)
   {
      if (this == HIND)
         return -value;

      return value;
   }

   public float negateIfFrontEnd(float value)
   {
      if (this == FRONT)
         return -value;

      return value;
   }

   public float negateIfHindEnd(float value)
   {
      if (this == HIND)
         return -value;

      return value;
   }

   public void checkRobotEndMatch(RobotEnd other)
   {
      if (!(this == other))
      {
         throw new RobotEndMismatchException();
      }
   }

   private static class RobotEndMismatchException extends RuntimeException
   {
      private static final long serialVersionUID = -159864473420885631L;

      public RobotEndMismatchException()
      {
         super();
      }
   }

   public static RobotEnd getEndFromString(String robotEndName)
   {
      RobotEnd[] ends = RobotEnd.values;

      for (RobotEnd end : ends)
      {
         if (robotEndName.equals(end.getEndNameFirstLetter()))
            return end;
      }

      return null;
   }

   public String getDocumentation(RobotEnd var)
   {
      switch (var)
      {
      case HIND:
         return "refers to the HIND end of a robot";
      case FRONT:
         return "refers to the FRONT end of a robot";

      default:
         return "no documentation available";
      }
   }
}
