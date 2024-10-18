package us.ihmc.commons.robotics.robotSide;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class RobotSideTest
{

	@Test
   public void testRobotSide()
   {
      RobotSide leftRobotSide = RobotSide.LEFT;
      RobotSide rightRobotSide = RobotSide.RIGHT;
      assertEquals(rightRobotSide, leftRobotSide.getOppositeSide());
      assertEquals(leftRobotSide, rightRobotSide.getOppositeSide());
      
      assertEquals("r", rightRobotSide.getShortLowerCaseName());
      assertEquals("l", leftRobotSide.getShortLowerCaseName());
      
      assertEquals("right", rightRobotSide.getCamelCaseNameForStartOfExpression());
      assertEquals("left", leftRobotSide.getCamelCaseNameForStartOfExpression());
      
      assertEquals("Right", rightRobotSide.getCamelCaseNameForMiddleOfExpression());
      assertEquals("Left", leftRobotSide.getCamelCaseNameForMiddleOfExpression());

      assertEquals("right", rightRobotSide.getLowerCaseName());
      assertEquals("left", leftRobotSide.getLowerCaseName());

      assertEquals("RIGHT", rightRobotSide.getSideNameInAllCaps());
      assertEquals("LEFT", leftRobotSide.getSideNameInAllCaps());

      assertEquals("R", rightRobotSide.getSideNameFirstLetter());
      assertEquals("L", leftRobotSide.getSideNameFirstLetter());

      assertEquals("/right_leg", rightRobotSide.getSideStringInRobonetFormat());
      assertEquals("/left_leg", leftRobotSide.getSideStringInRobonetFormat());

      assertEquals("right_leg", rightRobotSide.getSideStringInRobonetFormatWithoutSlash());
      assertEquals("left_leg", leftRobotSide.getSideStringInRobonetFormatWithoutSlash());

      assertEquals(rightRobotSide, RobotSide.getSideFromString("R"));
      assertEquals(rightRobotSide, RobotSide.getSideFromString("Right"));
      assertEquals(rightRobotSide, RobotSide.getSideFromString("right"));
      assertEquals(rightRobotSide, RobotSide.getSideFromString("r"));
      assertEquals(leftRobotSide, RobotSide.getSideFromString("L"));
      assertEquals(leftRobotSide, RobotSide.getSideFromString("Left"));
      assertEquals(leftRobotSide, RobotSide.getSideFromString("left"));
      assertEquals(leftRobotSide, RobotSide.getSideFromString("l"));

      double numberToTest = 34.33;
      assertEquals(numberToTest, rightRobotSide.negateIfLeftSide(numberToTest), 1e-7);
      assertEquals(-numberToTest, leftRobotSide.negateIfLeftSide(numberToTest), 1e-7);
      
      numberToTest = 74.99;
      assertEquals(-numberToTest, rightRobotSide.negateIfRightSide(numberToTest), 1e-7);
      assertEquals(numberToTest, leftRobotSide.negateIfRightSide(numberToTest), 1e-7);
   }

	@Test
   public void testCheckRobotSideMatch()
   {
      RobotSide leftRobotSide = RobotSide.LEFT;
      RobotSide rightRobotSide = RobotSide.RIGHT;
      RobotSide nullRobotSide = null;
      
      leftRobotSide.checkRobotSideMatch(RobotSide.LEFT);
      rightRobotSide.checkRobotSideMatch(RobotSide.RIGHT);
      
      try
      {
         leftRobotSide.checkRobotSideMatch(RobotSide.RIGHT);
         fail();
      }
      catch(RuntimeException e)
      {
      }
      
      try
      {
         rightRobotSide.checkRobotSideMatch(RobotSide.LEFT);
         fail();
      }
      catch(RuntimeException e)
      {
      }
      
      try
      {
         leftRobotSide.checkRobotSideMatch(nullRobotSide);
         fail();
      }
      catch(RuntimeException e)
      {
      }
      
      try
      {
         rightRobotSide.checkRobotSideMatch(nullRobotSide);
         fail();
      }
      catch(RuntimeException e)
      {
      }
      
   }

}
