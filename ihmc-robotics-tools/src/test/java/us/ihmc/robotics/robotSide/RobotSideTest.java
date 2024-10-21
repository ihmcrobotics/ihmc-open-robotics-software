package us.ihmc.robotics.robotSide;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class RobotSideTest
{

	@Test
   public void testRobotSide()
   {
      RobotSide leftRobotSide = RobotSide.LEFT;
      RobotSide rightRobotSide = RobotSide.RIGHT;
      Assertions.assertEquals(rightRobotSide, leftRobotSide.getOppositeSide());
      Assertions.assertEquals(leftRobotSide, rightRobotSide.getOppositeSide());
      
      Assertions.assertEquals("r", rightRobotSide.getShortLowerCaseName());
      Assertions.assertEquals("l", leftRobotSide.getShortLowerCaseName());
      
      Assertions.assertEquals("right", rightRobotSide.getCamelCaseNameForStartOfExpression());
      Assertions.assertEquals("left", leftRobotSide.getCamelCaseNameForStartOfExpression());
      
      Assertions.assertEquals("Right", rightRobotSide.getCamelCaseNameForMiddleOfExpression());
      Assertions.assertEquals("Left", leftRobotSide.getCamelCaseNameForMiddleOfExpression());

      Assertions.assertEquals("right", rightRobotSide.getLowerCaseName());
      Assertions.assertEquals("left", leftRobotSide.getLowerCaseName());

      Assertions.assertEquals("RIGHT", rightRobotSide.getSideNameInAllCaps());
      Assertions.assertEquals("LEFT", leftRobotSide.getSideNameInAllCaps());

      Assertions.assertEquals("R", rightRobotSide.getSideNameFirstLetter());
      Assertions.assertEquals("L", leftRobotSide.getSideNameFirstLetter());

      Assertions.assertEquals("/right_leg", rightRobotSide.getSideStringInRobonetFormat());
      Assertions.assertEquals("/left_leg", leftRobotSide.getSideStringInRobonetFormat());

      Assertions.assertEquals("right_leg", rightRobotSide.getSideStringInRobonetFormatWithoutSlash());
      Assertions.assertEquals("left_leg", leftRobotSide.getSideStringInRobonetFormatWithoutSlash());

      Assertions.assertEquals(rightRobotSide, RobotSide.getSideFromString("R"));
      Assertions.assertEquals(rightRobotSide, RobotSide.getSideFromString("Right"));
      Assertions.assertEquals(rightRobotSide, RobotSide.getSideFromString("right"));
      Assertions.assertEquals(rightRobotSide, RobotSide.getSideFromString("r"));
      Assertions.assertEquals(leftRobotSide, RobotSide.getSideFromString("L"));
      Assertions.assertEquals(leftRobotSide, RobotSide.getSideFromString("Left"));
      Assertions.assertEquals(leftRobotSide, RobotSide.getSideFromString("left"));
      Assertions.assertEquals(leftRobotSide, RobotSide.getSideFromString("l"));

      double numberToTest = 34.33;
      Assertions.assertEquals(numberToTest, rightRobotSide.negateIfLeftSide(numberToTest), 1e-7);
      Assertions.assertEquals(-numberToTest, leftRobotSide.negateIfLeftSide(numberToTest), 1e-7);
      
      numberToTest = 74.99;
      Assertions.assertEquals(-numberToTest, rightRobotSide.negateIfRightSide(numberToTest), 1e-7);
      Assertions.assertEquals(numberToTest, leftRobotSide.negateIfRightSide(numberToTest), 1e-7);
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
