package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class CentroidalMomentumHandlerTest
{
   private static final int ITERATIONS = 1000;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCenterOfMassVelocityChainRobot() throws Exception
   {
      Random random = new Random(32342L);
      int numberOfJoints = 20;
      List<RevoluteJoint> joints = ScrewTestTools.createRandomChainRobot(numberOfJoints, random);

      RigidBodyBasics rootBody = ScrewTools.getRootBody(joints.get(0).getPredecessor());
      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, rootBody);
      CentroidalMomentumHandler centroidalMomentumHandler = new CentroidalMomentumHandler(rootBody, centerOfMassFrame);
      CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody);

      FrameVector3D actualCenterOfMassVelocity = new FrameVector3D();
      FrameVector3D expectedCenterOfMassVelocity = new FrameVector3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         ScrewTestTools.setRandomPositions(joints, random);
         ScrewTestTools.setRandomVelocities(joints, random);
         centerOfMassFrame.update();

         centroidalMomentumHandler.compute();
         centerOfMassJacobian.compute();

         centroidalMomentumHandler.getCenterOfMassVelocity(actualCenterOfMassVelocity);
         centerOfMassJacobian.getCenterOfMassVelocity(expectedCenterOfMassVelocity);

         actualCenterOfMassVelocity.changeFrame(worldFrame);
         expectedCenterOfMassVelocity.changeFrame(worldFrame);
         EuclidCoreTestTools.assertTuple3DEquals(expectedCenterOfMassVelocity, actualCenterOfMassVelocity, 1.0e-12);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCenterOfMassVelocityFloatingBaseRobot() throws Exception
   {
      Random random = new Random(32342L);
      int numberOfJoints = 20;
      RandomFloatingChain randomFloatingChain = new RandomFloatingChain(random, numberOfJoints);

      RigidBodyBasics rootBody = randomFloatingChain.getElevator();
      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, rootBody);
      CentroidalMomentumHandler centroidalMomentumHandler = new CentroidalMomentumHandler(rootBody, centerOfMassFrame);
      CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(rootBody);

      FrameVector3D actualCenterOfMassVelocity = new FrameVector3D();
      FrameVector3D expectedCenterOfMassVelocity = new FrameVector3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         randomFloatingChain.setRandomPositionsAndVelocities(random);
         centerOfMassFrame.update();

         centroidalMomentumHandler.compute();
         centerOfMassJacobian.compute();

         centroidalMomentumHandler.getCenterOfMassVelocity(actualCenterOfMassVelocity);
         centerOfMassJacobian.getCenterOfMassVelocity(expectedCenterOfMassVelocity);

         actualCenterOfMassVelocity.changeFrame(worldFrame);
         expectedCenterOfMassVelocity.changeFrame(worldFrame);
         EuclidCoreTestTools.assertTuple3DEquals(expectedCenterOfMassVelocity, actualCenterOfMassVelocity, 1.0e-12);
      }
   }

}
