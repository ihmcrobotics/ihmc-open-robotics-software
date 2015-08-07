package us.ihmc.commonWalkingControlModules.calibration.virtualChain;


import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;

public class VirtualChainBuilderTest
{
   private static Random random = new Random(105L);

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

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testOne()
   {
      VirtualChainExampleRobot exampleRobot = VirtualChainExampleRobot.constructExampleOne();
      double comNoiseMaximum = 0.01;
      int numberOfDataPoints = 100;
      testVirtualChainBuilderForARobot(numberOfDataPoints, comNoiseMaximum, exampleRobot, exampleRobot);
   }

	@EstimatedDuration(duration = 1.9)
	@Test(timeout = 30000)
   public void testTwo()
   {
      VirtualChainExampleRobot exampleRobot = VirtualChainExampleRobot.constructExampleTwo();
      double comNoiseMaximum = 0.01;
      int numberOfDataPoints = 450;
      testVirtualChainBuilderForARobot(numberOfDataPoints, comNoiseMaximum, exampleRobot, exampleRobot);
   }

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void testThree()
   {
      VirtualChainExampleRobot exampleRobot = VirtualChainExampleRobot.constructExampleThree();
      double comNoiseMaximum = 0.01;
      int numberOfDataPoints = 100;
      testVirtualChainBuilderForARobot(numberOfDataPoints, comNoiseMaximum, exampleRobot, exampleRobot);
   }

	@EstimatedDuration(duration = 1.7)
	@Test(timeout = 30000)
   public void testFour()
   {
      VirtualChainExampleRobot exampleRobot = VirtualChainExampleRobot.constructExampleFour();
      double comNoiseMaximum = 0.01;
      int numberOfDataPoints = 450;
      VirtualChainBuilderTest.testVirtualChainBuilderForARobot(numberOfDataPoints, comNoiseMaximum, exampleRobot, exampleRobot);
   }

   private static void testVirtualChainBuilderForARobot(int numberOfDataPoints, double comNoiseMaximum, Robot exampleRobot,
           RobotRandomPositionMover randomPositionMover)
   {
      // First get exact answer and setup reference frames and such:
      VirtualChainConstructorFromARobot constructor = new VirtualChainConstructorFromARobot();
      VirtualLinkFromJoint virtualLinkFromJoint = constructor.createVirtualChainTestObject(exampleRobot);

      ArrayList<FrameVector> virtualMassParameterVectors = VirtualChainConstructorFromARobot.getVirtualChainFrameVectors(virtualLinkFromJoint);

//    ReferenceFrame baseFrame = ReferenceFrame.getWorldFrame(); 
      ReferenceFrame firstJointFrame = virtualMassParameterVectors.get(0).getReferenceFrame();
      ReferenceFrame baseFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), firstJointFrame, "baseFrame");

      VirtualChainCenterOfMassEstimator estimator = new VirtualChainCenterOfMassEstimator(baseFrame, virtualMassParameterVectors);

      // Now test that the exact answer is correct:
      randomPositionMover.moveToRandomPosition();
      exampleRobot.update();
      virtualLinkFromJoint.updateReferenceFrameFromJointAngleRecursively();
      baseFrame.update();

      FramePoint shouldBeExactCenterOfMass = estimator.getCenterOfMassInFrame(ReferenceFrame.getWorldFrame());

      Point3d centerOfMass = new Point3d();
      exampleRobot.computeCenterOfMass(centerOfMass);

//      System.out.println("centerOfMass = " + centerOfMass);
//      System.out.println("shouldBeExactCenterOfMass = " + shouldBeExactCenterOfMass);
//      System.out.println("\nExact VirtualChainCenterOfMassEstimator:\n" + estimator);

      VirtualChainBuilderTest.assertFramePointEquals(centerOfMass, shouldBeExactCenterOfMass, 1e-7);
      ArrayList<ReferenceFrame> referenceFrames = VirtualChainConstructorFromARobot.getReferenceFrames(virtualLinkFromJoint);

      double comErrorTolerance = comNoiseMaximum * 1.25;

      testVirtualChainBuilderForARobot(numberOfDataPoints, comNoiseMaximum, comErrorTolerance, virtualLinkFromJoint, baseFrame, referenceFrames, exampleRobot,
                                       randomPositionMover);
   }

   public static void testVirtualChainBuilderForARobot(int numberOfDataPoints, double comNoiseMaximum, double comErrorTolerance,
           VirtualLinkFromJoint virtualLinkFromJoint, ReferenceFrame baseFrame, ArrayList<ReferenceFrame> referenceFrames, Robot exampleRobot,
           RobotRandomPositionMover randomPositionMover)
   {
      // Now construct the builder

      VirtualChainBuilder builder = new VirtualChainBuilder(baseFrame, referenceFrames);

      // Now populate with data:
      Point3d centerOfMass = new Point3d();

      for (int i = 0; i < numberOfDataPoints; i++)
      {
         randomPositionMover.moveToRandomPosition();
         exampleRobot.update();

         updateReferenceFrames(virtualLinkFromJoint, baseFrame, referenceFrames);

         exampleRobot.computeCenterOfMass(centerOfMass);
         FramePoint centerOfMassFramePoint = new FramePoint(ReferenceFrame.getWorldFrame(), centerOfMass);

         centerOfMassFramePoint.changeFrame(baseFrame);
         FramePoint2d centerOfMassProjection = centerOfMassFramePoint.toFramePoint2d();

         double xCoMNoise = -comNoiseMaximum + 2.0 * comNoiseMaximum * random.nextDouble();
         double yCoMNoise = -comNoiseMaximum + 2.0 * comNoiseMaximum * random.nextDouble();

         FramePoint2d noiseToCenterOfMass = new FramePoint2d(centerOfMassProjection.getReferenceFrame(), xCoMNoise, yCoMNoise);

         centerOfMassProjection.add(noiseToCenterOfMass);
         builder.recordDataPoint(centerOfMassProjection);

//       builder.recordDataPoint(centerOfMassFramePoint);
      }

//    System.out.println("Builder: " + builder);


      // Now create the CoM estimator:
      if (builder.estimateVirtualChainParameterVectors())
      {
         ArrayList<FrameVector> estimatedVirtualChainParameterVectors = builder.getVirtualChainParameterVectors();
         VirtualChainCenterOfMassEstimator virtualChainCenterOfMassEstimator = new VirtualChainCenterOfMassEstimator(baseFrame,
                                                                                  estimatedVirtualChainParameterVectors);

//         System.out.println("VirtualChainCenterOfMassEstimator:\n" + virtualChainCenterOfMassEstimator);

         // Now compare and make sure it is close

         int numberOfTests = 100;

         for (int i = 0; i < numberOfTests; i++)
         {
            randomPositionMover.moveToRandomPosition();
            exampleRobot.update();
            updateReferenceFrames(virtualLinkFromJoint, baseFrame, referenceFrames);

            FramePoint estimatedCenterOfMass = virtualChainCenterOfMassEstimator.getCenterOfMassInFrame(ReferenceFrame.getWorldFrame());

            exampleRobot.computeCenterOfMass(centerOfMass);

//          System.out.println("centerOfMass = " + centerOfMass);
//          System.out.println("estimatedCenterOfMass = " + estimatedCenterOfMass);

            assertFramePointEquals(centerOfMass, estimatedCenterOfMass, comErrorTolerance);
         }
      }
      else
         throw new RuntimeException("Not enough data point to estimate the virtual chain");
   }


   private static void updateReferenceFrames(VirtualLinkFromJoint virtualLinkFromJoint, ReferenceFrame baseFrame, ArrayList<ReferenceFrame> referenceFrames)
   {
      virtualLinkFromJoint.updateReferenceFrameFromJointAngleRecursively();
      baseFrame.update();


//    for (ReferenceFrame referenceFrame : referenceFrames)
//    {
//       referenceFrame.update();
//    }
//
//    baseFrame.update();
   }

   private static void assertFramePointEquals(Point3d point3d, FramePoint framePoint, double epsilon)
   {
      assertEquals(point3d.getX(), framePoint.getX(), epsilon);
      assertEquals(point3d.getY(), framePoint.getY(), epsilon);
      assertEquals(point3d.getZ(), framePoint.getZ(), epsilon);
   }


}
