package us.ihmc.commonWalkingControlModules.calibration.virtualChain;


import static org.junit.Assert.assertEquals;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;

public class VirtualChainConstructorFromARobotTest
{
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

	@DeployableTestMethod(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testOneAtZero()
   {
      VirtualChainConstructorFromARobot constructor = new VirtualChainConstructorFromARobot();

      VirtualChainExampleRobot testRobot = VirtualChainExampleRobot.constructExampleOne();
      VirtualLinkFromJoint virtualLink1 = constructor.createVirtualChainTestObject(testRobot);

      testRobot.update();
      Point3d centerOfMass = new Point3d();
      double totalMass = testRobot.computeCenterOfMass(centerOfMass);

      assertEquals(VirtualChainExampleRobot.mass1 + VirtualChainExampleRobot.mass2 + VirtualChainExampleRobot.mass3, totalMass, 1e-7);
      assertEquals(totalMass, virtualLink1.getMassHereOnOut(), 1e-7);

      // Check for link 1:
      FrameVector virtualLinkFrameVector1 = virtualLink1.getVirtualLinkFrameVector();

      Vector3d expectedVirtualLinkVector1 = new Vector3d(VirtualChainExampleRobot.comOffset1);
      expectedVirtualLinkVector1.scale(VirtualChainExampleRobot.mass1);

      Vector3d temp = new Vector3d(VirtualChainExampleRobot.offset2);
      temp.scale(VirtualChainExampleRobot.mass2 + VirtualChainExampleRobot.mass3);

      expectedVirtualLinkVector1.add(temp);
      expectedVirtualLinkVector1.scale(1.0 / totalMass);

      assertFrameVectorEquals(expectedVirtualLinkVector1, virtualLinkFrameVector1);

      // Check for link 2:
      VirtualLinkFromJoint virtualLink2 = virtualLink1.getChildren().get(0);
      FrameVector virtualLinkFrameVector2 = virtualLink2.getVirtualLinkFrameVector();

      Vector3d expectedVirtualLinkVector2 = new Vector3d(VirtualChainExampleRobot.comOffset2);
      expectedVirtualLinkVector2.scale(VirtualChainExampleRobot.mass2);

      temp = new Vector3d(VirtualChainExampleRobot.offset3);
      temp.scale(VirtualChainExampleRobot.mass3);

      expectedVirtualLinkVector2.add(temp);
      expectedVirtualLinkVector2.scale(1.0 / totalMass);

      assertFrameVectorEquals(expectedVirtualLinkVector2, virtualLinkFrameVector2);

      // Check for link 3:
      VirtualLinkFromJoint virtualLink3 = virtualLink2.getChildren().get(0);
      FrameVector virtualLinkFrameVector3 = virtualLink3.getVirtualLinkFrameVector();

      Vector3d expectedVirtualLinkVector3 = new Vector3d(VirtualChainExampleRobot.comOffset3);
      expectedVirtualLinkVector3.scale(VirtualChainExampleRobot.mass3);

      expectedVirtualLinkVector3.scale(1.0 / totalMass);

      assertFrameVectorEquals(expectedVirtualLinkVector3, virtualLinkFrameVector3);

      // Print the virtual link frame vectors:
//    System.out.println("virtualLinkFrameVector1 = " + virtualLinkFrameVector1);
//    System.out.println("virtualLinkFrameVector2 = " + virtualLinkFrameVector2);
//    System.out.println("virtualLinkFrameVector3 = " + virtualLinkFrameVector3);


      // When all joints are 0.0, then the CoM should just be the sum of the virtual link frame vectors:
      Point3d comEstimateAtZero = new Point3d();

      comEstimateAtZero.add(virtualLinkFrameVector1.getVectorCopy());
      comEstimateAtZero.add(virtualLinkFrameVector2.getVectorCopy());
      comEstimateAtZero.add(virtualLinkFrameVector3.getVectorCopy());

//    System.out.println("Robot centerOfMass = " + centerOfMass);
//    System.out.println("comEstimateAtZero = " + comEstimateAtZero);

      assertTupleEquals(centerOfMass, comEstimateAtZero);
   }

	@DeployableTestMethod(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testOnePartTwo()
   {
      VirtualChainExampleRobot testRobotOne = VirtualChainExampleRobot.constructExampleOne();
      testRobotInRandomPositions(testRobotOne);
   }

	@DeployableTestMethod(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testTwo()
   {
      VirtualChainExampleRobot testRobotTwo = VirtualChainExampleRobot.constructExampleTwo();
      testRobotInRandomPositions(testRobotTwo);
   }

	@DeployableTestMethod(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testThree()
   {
      VirtualChainExampleRobot testRobotThree = VirtualChainExampleRobot.constructExampleThree();
      testRobotInRandomPositions(testRobotThree);
   }

	@DeployableTestMethod(estimatedDuration = 0.4)
	@Test(timeout = 30000)
   public void testFour()
   {
      VirtualChainExampleRobot testRobotFour = VirtualChainExampleRobot.constructExampleFour();
      testRobotInRandomPositions(testRobotFour);
   }


   private void testRobotInRandomPositions(VirtualChainExampleRobot testRobot)
   {
      VirtualChainConstructorFromARobot constructor = new VirtualChainConstructorFromARobot();
      VirtualLinkFromJoint virtualLink1 = constructor.createVirtualChainTestObject(testRobot);

      testRobot.update();
      Point3d centerOfMass = new Point3d();
      double totalMass = testRobot.computeCenterOfMass(centerOfMass);

      assertEquals(totalMass, virtualLink1.getMassHereOnOut(), 1e-7);

      // Now test CoM stuff:

      ArrayList<FrameVector> virtualMassParameterVectors = VirtualChainConstructorFromARobot.getVirtualChainFrameVectors(virtualLink1);
      ReferenceFrame firstFrame = virtualMassParameterVectors.get(0).getReferenceFrame();
      ReferenceFrame baseFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), firstFrame, "baseFrame");
      baseFrame.update();

      VirtualChainCenterOfMassEstimator estimator = new VirtualChainCenterOfMassEstimator(baseFrame, virtualMassParameterVectors);

      FramePoint estimatedCenterOfMass = estimator.getCenterOfMassInFrame(ReferenceFrame.getWorldFrame());
      totalMass = testRobot.computeCenterOfMass(centerOfMass);

      assertFramePointEquals(centerOfMass, estimatedCenterOfMass);

//    Move the joints some and do it again

      int numberOfTests = 100;
      for (int i = 0; i < numberOfTests; i++)
      {
         testRobot.moveToRandomPosition();
         testRobot.update();

         virtualLink1.updateReferenceFrameFromJointAngleRecursively();
         baseFrame.update();

         estimatedCenterOfMass = estimator.getCenterOfMassInFrame(ReferenceFrame.getWorldFrame());

         totalMass = testRobot.computeCenterOfMass(centerOfMass);

//       System.out.println("estimatedCenterOfMass = " + estimatedCenterOfMass);
//       System.out.println("centerOfMass = " + centerOfMass);

         assertFramePointEquals(centerOfMass, estimatedCenterOfMass);
      }

   }

   private void assertTupleEquals(Tuple3d vector1, Tuple3d vector2)
   {
      assertEquals(vector1.getX(), vector2.getX(), 1e-7);
      assertEquals(vector1.getY(), vector2.getY(), 1e-7);
      assertEquals(vector1.getZ(), vector2.getZ(), 1e-7);
   }


   private void assertFrameVectorEquals(Vector3d vector3d, FrameVector frameVector)
   {
      assertEquals(vector3d.getX(), frameVector.getX(), 1e-7);
      assertEquals(vector3d.getY(), frameVector.getY(), 1e-7);
      assertEquals(vector3d.getZ(), frameVector.getZ(), 1e-7);
   }

   private void assertFramePointEquals(Point3d point3d, FramePoint framePoint)
   {
      assertEquals(point3d.getX(), framePoint.getX(), 1e-7);
      assertEquals(point3d.getY(), framePoint.getY(), 1e-7);
      assertEquals(point3d.getZ(), framePoint.getZ(), 1e-7);
   }


}
