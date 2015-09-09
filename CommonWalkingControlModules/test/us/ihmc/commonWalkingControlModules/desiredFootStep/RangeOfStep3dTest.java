package us.ihmc.commonWalkingControlModules.desiredFootStep;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

public class RangeOfStep3dTest
{
   private static final double epsilon = 1e-4;
   private static final double iterations = 100;
   
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

	@DeployableTestMethod(duration = 0.3)
	@Test(timeout = 30000)
   public void testExampleUsage()
   {
      RigidBody rigidBody = new RigidBody("rigidBody", ReferenceFrame.getWorldFrame());
      
      RobotSide robotSide = RobotSide.LEFT;
      
      double forwardLength = 1.0;
      double sideLength = 1.0;
      double verticalLength = 1.0;
      double offset = 0.1;
      
      RangeOfStep3d range = new RangeOfStep3d(rigidBody, robotSide, forwardLength, sideLength, verticalLength, offset);
      
      assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0)));
      assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(offset), 0.0)));
      
      assertEquals(range.getEllipsoid3d().getXRadius(), 0.5 * forwardLength, 1e-7);
      assertEquals(range.getEllipsoid3d().getYRadius(), 0.5 * sideLength, 1e-7);
      assertEquals(range.getEllipsoid3d().getZRadius(), 0.5 * verticalLength, 1e-7);
   }

	@DeployableTestMethod(duration = 0.3)
	@Test(timeout = 30000)
   public void testAtCenter()
   {
      testAtTranslation(2013L, ReferenceFrame.getWorldFrame(), new Vector3d(0.0, 0.0, 0.0));
   }

	@DeployableTestMethod(duration = 0.2)
	@Test(timeout = 30000)
   public void testAtRandomTranslation()
   {
      Random random = new Random(58008L);
      
      double xTranslation = random.nextDouble();
      double yTranslation = random.nextDouble();
      double zTranslation = random.nextDouble();
      
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(new Vector3d(xTranslation, yTranslation, zTranslation));
      
      ReferenceFrame frame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("translatedFrame", ReferenceFrame.getWorldFrame(), transform);
      
      testAtTranslation(5318008L, frame, new Vector3d(xTranslation, yTranslation, zTranslation));
   }

	@DeployableTestMethod(duration = 0.2)
	@Test(timeout = 30000)
   public void test90DegreeRotation()
   {
      RobotSide robotSide = null;
      
      for (int n = 0; n < iterations; n++)
      {
         Random random = new Random(317718L);
         
         double forwardLength = generateRandomDoubleBetween(random, 0.001, 100);
         double sideLength = generateRandomDoubleBetween(random, 0.001, 100);
         double verticalLength = generateRandomDoubleBetween(random, 0.001, 100);
         double offset = generateRandomDoubleBetween(random, 0.001, 100);
         
         for (int i = 0; i < 2; i++)
         {
            if (i == 0)
               robotSide = RobotSide.LEFT;
            if (i == 1)
               robotSide = RobotSide.RIGHT;
            
            for (int j = 0; j < 3; j++)
            {
               RigidBodyTransform transform = new RigidBodyTransform();
               
               if (j == 0)
                  transform.rotX(Math.PI / 2.0);
               if (j == 1)
                  transform.rotY(Math.PI / 2.0);
               if (j == 2)
                  transform.rotZ(Math.PI / 2.0);
               
               ReferenceFrame frame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("rotatedFrame", ReferenceFrame.getWorldFrame(), transform);
               
               RigidBody rigidBody = new RigidBody("rotatedBody", frame);
               
               RangeOfStep3d range = new RangeOfStep3d(rigidBody, robotSide, forwardLength, sideLength, verticalLength, offset);
               
               assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0)));
               
               if (j == 0)
               {
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, robotSide.negateIfRightSide(offset))));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, robotSide.negateIfRightSide(offset - epsilon))));
                  
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, robotSide.negateIfRightSide(0.5 * sideLength + offset - epsilon))));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, robotSide.negateIfRightSide(0.5 * sideLength + offset + epsilon))));
                  
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.5 * forwardLength - epsilon, 0.0, robotSide.negateIfRightSide(offset))));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.5 * forwardLength + epsilon, 0.0, robotSide.negateIfRightSide(offset))));

                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), -0.5 * forwardLength, 0.0, robotSide.negateIfRightSide(offset))));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), -(0.5 * forwardLength + epsilon), 0.0, robotSide.negateIfRightSide(offset))));
               }
               
               if (j == 1)
               {
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(offset), 0.0)));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(offset - epsilon), 0.0)));
                  
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(0.5 * sideLength + offset - epsilon), 0.0)));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(0.5 * sideLength + offset + epsilon), 0.0)));
                  
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.5 * verticalLength - epsilon, robotSide.negateIfRightSide(offset), 0.0)));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.5 * verticalLength + epsilon, robotSide.negateIfRightSide(offset), 0.0)));
                  
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), -0.5 * verticalLength, robotSide.negateIfRightSide(offset), 0.0)));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), -(0.5 * verticalLength + epsilon), robotSide.negateIfRightSide(offset), 0.0)));
                  
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(offset), 0.5 * forwardLength)));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(offset), 0.5 * forwardLength + epsilon)));
                  
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(offset), -0.5 * forwardLength)));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(offset), -(0.5 * forwardLength + epsilon))));
               }
               
               if (j == 2)
               {
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), robotSide.negateIfLeftSide(offset), 0.0, 0.0)));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), robotSide.negateIfLeftSide(offset - epsilon), 0.0, 0.0)));
                  
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), robotSide.negateIfLeftSide(0.5 * sideLength + offset - epsilon), 0.0, 0.0)));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), robotSide.negateIfLeftSide(0.5 * sideLength + offset + epsilon), 0.0, 0.0)));
                  
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), robotSide.negateIfLeftSide(offset), 0.5 * forwardLength - epsilon, 0.0)));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), robotSide.negateIfLeftSide(offset), 0.5 * forwardLength + epsilon, 0.0)));
                  
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), robotSide.negateIfLeftSide(offset), -(0.5 * forwardLength - epsilon), 0.0)));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), robotSide.negateIfLeftSide(offset), -(0.5 * forwardLength + epsilon), 0.0)));
                  
                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), robotSide.negateIfLeftSide(offset), 0.0, 0.5 * verticalLength - epsilon)));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), robotSide.negateIfLeftSide(offset), 0.0, 0.5 * verticalLength + epsilon)));

                  assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), robotSide.negateIfLeftSide(offset), 0.0, -(0.5 * verticalLength - epsilon))));
                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), robotSide.negateIfLeftSide(offset), 0.0, -(0.5 * verticalLength + epsilon))));
               }
            }
         }
      }
   }

	@DeployableTestMethod(duration = 0.3)
	@Test(timeout = 30000)
   public void testUnderRotation()
   {
      RobotSide robotSide = null;
      
      for (int n = 0; n < iterations; n++)
      {
         Random random = new Random(5318008L);
         
         double forwardLength = generateRandomDoubleBetween(random, 0.001, 100);
         double sideLength = generateRandomDoubleBetween(random, 0.001, 100);
         double verticalLength = generateRandomDoubleBetween(random, 0.001, 100);
         double offset = generateRandomDoubleBetween(random, 0.001, 100);
         double[] angles = new double[] { Math.PI / 3.0, Math.PI / 4.0, Math.PI / 6.0};
         for (double angle : angles)
         {
            for (int j = 0; j < 2; j++)
            {
               if (j == 0)
                  robotSide = RobotSide.LEFT;
               if (j == 1)
                  robotSide = RobotSide.RIGHT;

               for (int i = 0; i < 3; i++)
               {
                  RigidBodyTransform transform = new RigidBodyTransform();

                  if (i == 0)
                     transform.rotX(angle);
                  if (i == 1)
                     transform.rotY(angle);
                  if (i == 2)
                     transform.rotZ(angle);

                  ReferenceFrame frame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("rotatedFrame", ReferenceFrame.getWorldFrame(), transform);

                  RigidBody rigidBody = new RigidBody("rotatedBody", frame);

                  RangeOfStep3d range = new RangeOfStep3d(rigidBody, robotSide, forwardLength, sideLength, verticalLength, offset);

                  assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0)));

                  if (i == 0)
                  {
                     double yCoord = robotSide.negateIfRightSide(offset) * Math.cos(angle);
                     double zCoord = robotSide.negateIfRightSide(offset) * Math.sin(angle);

                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, yCoord, zCoord)));
                     assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, yCoord, zCoord - robotSide.negateIfRightSide(epsilon))));

                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.5 * forwardLength, yCoord, zCoord)));
                     assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.5 * forwardLength + epsilon, yCoord, zCoord)));
                     
                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), -0.5 * forwardLength, yCoord, zCoord)));
                     assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), -(0.5 * forwardLength + epsilon), yCoord, zCoord)));

                     yCoord = robotSide.negateIfRightSide(offset + 0.5 * sideLength) * Math.cos(angle);
                     zCoord = robotSide.negateIfRightSide(offset + 0.5 * sideLength) * Math.sin(angle);

                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, yCoord - robotSide.negateIfRightSide(epsilon), zCoord)));
                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, yCoord, zCoord - robotSide.negateIfRightSide(epsilon))));
                     assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, yCoord + robotSide.negateIfRightSide(epsilon), zCoord)));
                     assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, yCoord, zCoord + robotSide.negateIfRightSide(epsilon))));
                  }

                  if (i == 1)
                  {
                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(offset + epsilon), 0.0)));
                     assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(offset - epsilon), 0.0)));
                     
                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(0.5 * sideLength + offset - epsilon), 0.0)));
                     assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, robotSide.negateIfRightSide(0.5 * sideLength + offset + epsilon), 0.0)));

                     double xCoord = 0.5 * (forwardLength - epsilon) * Math.cos(angle);
                     double zCoord = -0.5 * (forwardLength - epsilon) * Math.sin(angle);

                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), xCoord, robotSide.negateIfRightSide(offset), zCoord)));
                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), -xCoord, robotSide.negateIfRightSide(offset), -zCoord)));

                     xCoord = 0.5 * verticalLength * Math.sin(angle);
                     zCoord = 0.5 * verticalLength * Math.cos(angle);
                     
                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), xCoord, robotSide.negateIfRightSide(offset), zCoord - epsilon)));
                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), -xCoord, robotSide.negateIfRightSide(offset), -zCoord + epsilon)));
                  }

                  if (i == 2)
                  {
                     double xCoord = robotSide.negateIfRightSide(offset) * Math.sin(angle);
                     double yCoord = robotSide.negateIfRightSide(offset) * Math.cos(angle);

                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), -xCoord, yCoord, 0.0)));
                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), -xCoord, yCoord, 0.5 * verticalLength)));

                     xCoord = robotSide.negateIfRightSide(offset + 0.5 * sideLength - epsilon) * Math.sin(angle);
                     yCoord = robotSide.negateIfRightSide(offset + 0.5 * sideLength - epsilon) * Math.cos(angle);

                     assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), -xCoord, yCoord, 0.0)));
                  }
               }
            }
         }
      }
   }
   
   public void testAtTranslation(long seed, ReferenceFrame frame, Vector3d translation)
   {
      Random random = new Random(seed);
      
      RigidBody rigidBody = new RigidBody("rigidBody", frame);
      
      RobotSide robotSide = null;
      
      for (int n = 0; n < iterations; n++)
      {
         double forwardLength = generateRandomDoubleBetween(random, 0.001, 100);
         double sideLength = generateRandomDoubleBetween(random, 0.001, 100);
         double verticalLength = generateRandomDoubleBetween(random, 0.001, 100);
         double offset = generateRandomDoubleBetween(random, 0.001, 100);
         
         for (int i = 0; i < 2; i++)
         {
            if (i == 0)
               robotSide = RobotSide.LEFT;
            if (i == 1)
               robotSide = RobotSide.RIGHT;

            RangeOfStep3d range = new RangeOfStep3d(rigidBody, robotSide, forwardLength, sideLength, verticalLength, offset);
            
            assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), translation.getX(), translation.getY(), translation.getZ())));
            assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), translation.getX(), translation.getY() + robotSide.negateIfRightSide(offset), translation.getZ())));
            
            assertEquals(range.getEllipsoid3d().getXRadius(), 0.5 * forwardLength, 1e-7);
            assertEquals(range.getEllipsoid3d().getYRadius(), 0.5 * sideLength, 1e-7);
            assertEquals(range.getEllipsoid3d().getZRadius(), 0.5 * verticalLength, 1e-7);
            
            assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), translation.getX() + 0.5 * forwardLength - epsilon, translation.getY() + robotSide.negateIfRightSide(offset), translation.getZ())));
            assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), translation.getX() + 0.5 * forwardLength + epsilon, translation.getY() + robotSide.negateIfRightSide(offset), translation.getZ())));
            assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), translation.getX() - (0.5 * forwardLength - epsilon), translation.getY() + robotSide.negateIfRightSide(offset), translation.getZ())));
            assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), translation.getX() - (0.5 * forwardLength + epsilon), translation.getY() + robotSide.negateIfRightSide(offset), translation.getZ())));
            
            assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), translation.getX(), translation.getY() + robotSide.negateIfRightSide(offset + 0.5 * sideLength - epsilon), translation.getZ())));
            assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), translation.getX(), translation.getY() + robotSide.negateIfRightSide(offset + 0.5 * sideLength + epsilon), translation.getZ())));
            
            assertTrue(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), translation.getX(), translation.getY() + robotSide.negateIfRightSide(offset), translation.getZ() + 0.5 * verticalLength - epsilon)));
            assertFalse(range.containsPoint(new FramePoint(ReferenceFrame.getWorldFrame(), translation.getX(), translation.getY() + robotSide.negateIfRightSide(offset), translation.getZ() + 0.5 * verticalLength + epsilon)));
         }
      }
   }
   
   private double generateRandomDoubleBetween(Random random, double minValue, double maxValue)
   {
      return minValue + random.nextDouble() * (maxValue - minValue);
   }
}
