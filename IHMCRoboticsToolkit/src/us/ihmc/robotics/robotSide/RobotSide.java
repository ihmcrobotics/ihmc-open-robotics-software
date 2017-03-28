package us.ihmc.robotics.robotSide;

import java.util.EnumSet;
import java.util.Random;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public enum RobotSide
{
   LEFT, RIGHT;

   public static final EnumSet<RobotSide> set = EnumSet.allOf(RobotSide.class);
   public static final RobotSide[] values = values();

   public static RobotSide generateRandomRobotSide(Random random)
   {
      if (random.nextBoolean())
      {
         return LEFT;
      }
      else
      {
         return RIGHT;
      }
   }

   public RobotSide getOppositeSide()
   {
      if (this == RIGHT)
         return LEFT;
      else
         return RIGHT;
   }

   public String getShortLowerCaseName()
   {
      return getCamelCaseNameForMiddleOfExpression().substring(0, 1).toLowerCase();
   }

   public String getCamelCaseNameForStartOfExpression()
   {
      if (this == RIGHT)
         return "right";
      else
         return "left";
   }

   public String getCamelCaseNameForMiddleOfExpression()
   {
      if (this == RIGHT)
         return "Right";
      else
         return "Left";
   }
   
   public String getCamelCaseName()
   {
      return getCamelCaseNameForStartOfExpression();
   }
   
   public String getPascalCaseName()
   {
      return getCamelCaseNameForMiddleOfExpression();
   }
   
   public String getLowerCaseName()
   {
      if (this == RIGHT)
         return "right";
      else
         return "left";
   }

   public String getSideNameInAllCaps()
   {
      if (this == RIGHT)
         return "RIGHT";
      else
         return "LEFT";
   }

   public String getSideNameFirstLetter()
   {
      if (this == RIGHT)
         return "R";
      else
         return "L";
   }

   public String getSideStringInRobonetFormat()
   {
      if (this == RIGHT)
         return "/right_leg";
      else
         return "/left_leg";
   }

   public String getSideStringInRobonetFormatWithoutSlash()
   {
      if (this == RIGHT)
         return "right_leg";
      else
         return "left_leg";
   }

   public double negateIfLeftSide(double value)
   {
      if (this == LEFT)
         return -value;

      return value;
   }

   public double negateIfRightSide(double value)
   {
      if (this == RIGHT)
         return -value;

      return value;
   }

   public float negateIfLeftSide(float value)
   {
      if (this == LEFT)
         return -value;

      return value;
   }

   public float negateIfRightSide(float value)
   {
      if (this == RIGHT)
         return -value;

      return value;
   }

   public void checkRobotSideMatch(RobotSide other)
   {
      if (!(this == other))
      {
         throw new RobotSideMismatchException();
      }
   }

   private static class RobotSideMismatchException extends RuntimeException
   {
      private static final long serialVersionUID = -159864473420885631L;

      public RobotSideMismatchException()
      {
         super();
      }
   }
   
   public static RobotSide getSideFromString(String robotSideName)
   {
      RobotSide[] sides = RobotSide.values;
      
      
      for(RobotSide side : sides)
      {
         if (robotSideName.equals(side.getSideNameFirstLetter()))
            return side;
      }
      
      return null;
   }
   
   public static void main(String[] args)
   {
      String testString = RobotSide.LEFT.toString();
      
      RobotSide side = RobotSide.getSideFromString(testString);
      
      System.out.println(side.toString());
      
      testString = RobotSide.RIGHT.toString();
      side = RobotSide.getSideFromString(testString);
      
      System.out.println(side.toString());
      
      testString = "junk";
      side = RobotSide.getSideFromString(testString);
      
      System.out.println(side.toString());
   }

   public static String getDocumentation(RobotSide var)
   {
      switch (var)
      {
      case RIGHT:
         return "refers to the RIGHT side of a robot";
      case LEFT:
         return "refers to the LEFT side of a robot";

      default:
         return "no documentation available";
      }
   }
}
