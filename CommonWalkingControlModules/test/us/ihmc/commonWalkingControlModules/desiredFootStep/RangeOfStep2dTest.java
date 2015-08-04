package us.ihmc.commonWalkingControlModules.desiredFootStep;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

public class RangeOfStep2dTest
{
   private static final double EPSILON = 1e-7;
//   private final double verticalLength = 0.0;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void testExampleUsage()
   {
      double forwardLength = 1.0;
      double sideLength = 1.0;
      double offset = 0.0;
      
      RigidBody rigidBody = new RigidBody("rigidBody", ReferenceFrame.getWorldFrame());
      
      RangeOfStep2d range = new RangeOfStep2d(rigidBody, RobotSide.LEFT, forwardLength, sideLength, offset);
      
      assertTrue(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0)));
      assertEquals(range.height, forwardLength, 1e-7);
      assertEquals(range.width, sideLength, 1e-7);
   }

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void testEllipseAtCenter()
   {
      double forwardLength = 1.0;
      double sideLength = 1.0;
      double offset = 0.1;
      
      RobotSide robotSide = null;
      
      RigidBody rigidBody = new RigidBody("rigidBody", ReferenceFrame.getWorldFrame());
      
      for (int i = 0; i < 2; i++)
      {
         if (i == 0)
            robotSide = RobotSide.LEFT;
         if (i == 1)
            robotSide = RobotSide.RIGHT;
         
         RangeOfStep2d range = new RangeOfStep2d(rigidBody, robotSide, forwardLength, sideLength, offset);
         
         // center of ellipse is shifted by offset so neither hemi-ellipse should contain the origin
         assertFalse(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0)));
         assertFalse(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0)));
         assertFalse(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, -1.0)));
         
         // moving in +/- y (depending on robotSide) by offset should give the midpoint of the straight edge of the hemi-ellipse
         assertTrue(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(offset), 0.0)));
         assertFalse(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(offset-EPSILON), 0.0)));
         
         // from midpoint of flat edge, moving in +/- x by half the forwardLength should give the two corners of the hemi-ellipse
         assertTrue(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), 0.5 * forwardLength - EPSILON, robotSide.negateIfRightSide(offset), 0.0)));
         assertFalse(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), forwardLength, robotSide.negateIfRightSide(offset-EPSILON), 0.0)));
         
         assertTrue(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), -0.5 * (forwardLength - EPSILON), robotSide.negateIfRightSide(offset), 0.0)));
         assertFalse(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), -forwardLength, robotSide.negateIfRightSide(offset - EPSILON), 0.0)));
      }
   }

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void testTranslatedEllipse()
   {
      double forwardLength = 1.0;
      double sideLength = 1.0;
      double offset = 0.1;

      double x = 5.0;
      double y = 5.0;
      double z = 5.0;

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(new Vector3d(x, y, z));
      ReferenceFrame frame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("translatedFrame", ReferenceFrame.getWorldFrame(), transform);

      RigidBody rigidBody = new RigidBody("rigidBody", frame);

      for (RobotSide robotSide : RobotSide.values)
      {
         RangeOfStep2d range = new RangeOfStep2d(rigidBody, robotSide, forwardLength, sideLength, offset);

         System.out.println("center X: " + range.getCenterX() + " center Y: " + range.getCenterY());

         assertFalse(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), x, y, 0.0)));
         assertFalse(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), x, y, 1.0)));
         assertFalse(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), x, y, -1.0)));

         assertFalse(range.contains(new FramePoint(frame, 0.0, 0.0, 0.0)));

         assertTrue(range.contains(new FramePoint(frame, 0.0, robotSide.negateIfRightSide(offset + EPSILON), 0.0)));
         assertFalse(range.contains(new FramePoint(frame, 0.0, robotSide.negateIfRightSide(offset - EPSILON), 0.0)));

         assertTrue(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), x, y + robotSide.negateIfRightSide(offset + EPSILON), z)));
         assertFalse(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), x, y + robotSide.negateIfRightSide(offset - EPSILON), z)));

         assertTrue(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), x + 0.5 * forwardLength - EPSILON, y + robotSide.negateIfRightSide(offset + EPSILON), z)));
         assertFalse(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), x + 0.5 * forwardLength + EPSILON, y + robotSide.negateIfRightSide(offset + EPSILON), z)));

         assertTrue(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), x - (0.5 * forwardLength - EPSILON), y + robotSide.negateIfRightSide(offset + EPSILON), z)));
         assertFalse(range.contains(new FramePoint(ReferenceFrame.getWorldFrame(), x - (0.5 * forwardLength + EPSILON), y + robotSide.negateIfRightSide(offset + EPSILON), z)));
      }
   }
}
