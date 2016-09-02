package us.ihmc.robotics.robotDescription;

import static org.junit.Assert.*;

import org.junit.Test;

public class RobotDescriptionTest
{

   @Test
   public void testOne()
   {
      RobotDescription description = new RobotDescription("Test");

      assertEquals("Test", description.getName());


   }


}
