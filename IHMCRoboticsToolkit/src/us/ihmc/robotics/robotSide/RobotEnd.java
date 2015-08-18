package us.ihmc.robotics.robotSide;

import java.util.Random;

import us.ihmc.tools.DocumentedEnum;
import us.ihmc.tools.FormattingTools;

public enum RobotEnd implements DocumentedEnum<RobotEnd>
{
   BACK, FRONT;

   public static final RobotEnd[] values = values();

   public static RobotEnd generateRandomRobotEnd(Random random)
   {
      if (random.nextBoolean())
      {
         return FRONT;
      }
      else
      {
         return BACK;
      }
   }

   public RobotEnd getOppositeEnd()
   {
      if (this == BACK)
         return FRONT;
      else
         return BACK;
   }

   public String getShortLowerCaseName()
   {
      return getCamelCaseNameForMiddleOfExpression().substring(0, 1).toLowerCase();
   }

   public String getCamelCaseNameForStartOfExpression()
   {
      return FormattingTools.lowerCaseFirstLetter(getCamelCaseNameForMiddleOfExpression());
   }

   public String getCamelCaseNameForMiddleOfExpression()
   {
      if (this == BACK)
         return "Back";
      else
         return "Front";
   }

   public String getLowerCaseName()
   {
      if (this == BACK)
         return "back";
      else
         return "front";
   }

   public String getSideNameInAllCaps()
   {
      if (this == BACK)
         return "BACK";
      else
         return "FRONT";
   }

   public String getEndNameFirstLetter()
   {
      if (this == BACK)
         return "B";
      else
         return "F";
   }

   public double negateIfFrontEnd(double value)
   {
      if (this == FRONT)
         return -value;

      return value;
   }

   public double negateIfBackEnd(double value)
   {
      if (this == BACK)
         return -value;

      return value;
   }

   public float negateIfFrontEnd(float value)
   {
      if (this == FRONT)
         return -value;

      return value;
   }

   public float negateIfBackEnd(float value)
   {
      if (this == BACK)
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

   @Override
   public String getDocumentation(RobotEnd var)
   {
      switch (var)
      {
      case BACK:
         return "refers to the BACK end of a robot";
      case FRONT:
         return "refers to the FRONT end of a robot";

      default:
         return "no documentation available";
      }
   }

   @Override
   public RobotEnd[] getDocumentedValues()
   {
      return values;
   }
}
