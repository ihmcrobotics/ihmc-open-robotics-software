package us.ihmc.commonWalkingControlModules.stateEstimation;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.controlFlow.NullControlFlowElement;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTestTools.RandomFloatingChain;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.test.JUnitTools;

public class CenterOfMassBasedFullRobotModelUpdaterTest
{
   private static final boolean TEST_EFFICIENCY = false;

   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

   @Test
   public void testModelUpdatorWithRandomFloatingChain()
   {
      Random random = new Random(1235L);
      Vector3d[] jointAxes = new Vector3d[] {X, Y, Z};
      RandomFloatingChain randomFloatingChain = new RandomFloatingChain(random, jointAxes);
      RigidBody elevator = randomFloatingChain.getElevator();
      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();

      RigidBody estimationLink = randomFloatingChain.getRevoluteJoints().get(0).getSuccessor();    // some link in the middle of the chain
      ReferenceFrame estimationFrame = estimationLink.getParentJoint().getFrameAfterJoint();

      ControlFlowElement controlFlowElement = new NullControlFlowElement();

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator);

      ControlFlowOutputPort<FramePoint> centerOfMassPositionPort = new ControlFlowOutputPort<FramePoint>(controlFlowElement);
      ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort = new ControlFlowOutputPort<FrameVector>(controlFlowElement);
      ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort = new ControlFlowOutputPort<FrameVector>(controlFlowElement);

      ControlFlowOutputPort<FrameOrientation> orientationPort = new ControlFlowOutputPort<FrameOrientation>(controlFlowElement);
      ControlFlowOutputPort<FrameVector> angularVelocityPort = new ControlFlowOutputPort<FrameVector>(controlFlowElement);
      ControlFlowOutputPort<FrameVector> angularAccelerationPort = new ControlFlowOutputPort<FrameVector>(controlFlowElement);

      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, twistCalculator, 0.0, false);
      CenterOfMassBasedFullRobotModelUpdater fullRobotModelUpdater = new CenterOfMassBasedFullRobotModelUpdater(twistCalculator, spatialAccelerationCalculator,
                                                                        centerOfMassPositionPort, centerOfMassVelocityPort, centerOfMassAccelerationPort,
                                                                        orientationPort, angularVelocityPort, angularAccelerationPort, estimationLink,
                                                                        estimationFrame, rootJoint);

      // TODO: test linear acceleration

      int nTests = 100;
      for (int i = 0; i < nTests; i++)
      {
         randomFloatingChain.setRandomPositionsAndVelocities(random);
         randomFloatingChain.getElevator().updateFramesRecursively();
         twistCalculator.compute();
         spatialAccelerationCalculator.compute();

         centerOfMassPositionPort.setData(new FramePoint(ReferenceFrame.getWorldFrame(), RandomTools.generateRandomVector(random)));
         centerOfMassVelocityPort.setData(new FrameVector(ReferenceFrame.getWorldFrame(), RandomTools.generateRandomVector(random)));
         centerOfMassAccelerationPort.setData(new FrameVector(estimationFrame, RandomTools.generateRandomVector(random)));
         Matrix3d orientation = new Matrix3d();
         orientation.set(RandomTools.generateRandomRotation(random));
         orientationPort.setData(new FrameOrientation(ReferenceFrame.getWorldFrame(), orientation));
         angularVelocityPort.setData(new FrameVector(estimationFrame, RandomTools.generateRandomVector(random)));
         angularAccelerationPort.setData(new FrameVector(estimationFrame, RandomTools.generateRandomVector(random)));

         // update full robot model
         fullRobotModelUpdater.run();
         Twist tempTwist = new Twist();
         twistCalculator.packTwistOfBody(tempTwist, randomFloatingChain.getRevoluteJoints().get(0).getSuccessor());
         tempTwist.changeFrame(randomFloatingChain.getRevoluteJoints().get(0).getFrameAfterJoint());

         // compare with ports
         double epsilon = 1e-12;
         compareCenterOfMass(elevator, centerOfMassPositionPort, epsilon);
         compareOrientation(estimationFrame, orientationPort, epsilon);
         compareCenterOfMassVelocity(elevator, centerOfMassVelocityPort, epsilon);
         compareAngularVelocity(estimationLink, estimationFrame, twistCalculator, angularVelocityPort, epsilon);
         compareAngularAcceleration(estimationLink, estimationFrame, spatialAccelerationCalculator, angularAccelerationPort, epsilon);
      }


      // Test efficiency:
      if (TEST_EFFICIENCY)
      {
         long startTime = System.currentTimeMillis();
         nTests = 100000;
                  
         for (int i = 0; i < nTests; i++)
         {
            fullRobotModelUpdater.run();
         }
         long endTime = System.currentTimeMillis();
         double millisecondsPerCall = ((endTime - startTime))/((double) nTests);

         System.out.println("millisecondsPerCall = " + millisecondsPerCall);
         assertTrue("Not efficient! millisecondsPerCall = " + millisecondsPerCall, millisecondsPerCall < 0.03);
      }
   }

   private void compareCenterOfMass(RigidBody elevator, ControlFlowOutputPort<FramePoint> centerOfMassPositionPort, double epsilon)
   {
      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(elevator, ReferenceFrame.getWorldFrame());
      centerOfMassCalculator.compute();
      FramePoint centerOfMassBack = new FramePoint(ReferenceFrame.getWorldFrame());
      centerOfMassCalculator.packCenterOfMass(centerOfMassBack);
      JUnitTools.assertFramePointEquals(centerOfMassPositionPort.getData(), centerOfMassBack, epsilon);
   }

   private void compareOrientation(ReferenceFrame estimationFrame, ControlFlowOutputPort<FrameOrientation> orientationPort, double epsilon)
   {
      FrameOrientation estimationFrameOrientation = new FrameOrientation(estimationFrame);
      estimationFrameOrientation.changeFrame(orientationPort.getData().getReferenceFrame());
      Matrix3d rotationBack = estimationFrameOrientation.getMatrix3d();
      Matrix3d rotation = orientationPort.getData().getMatrix3d();
      assertTrue(rotationBack.epsilonEquals(rotation, epsilon));
   }

   private void compareCenterOfMassVelocity(RigidBody elevator, ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort, double epsilon)
   {
      CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(elevator);
      centerOfMassJacobian.compute();
      FrameVector centerOfMassVelocityBack = new FrameVector(ReferenceFrame.getWorldFrame());
      centerOfMassJacobian.packCenterOfMassVelocity(centerOfMassVelocityBack);
      FrameVector centerOfMassVelocity = centerOfMassVelocityPort.getData();
      centerOfMassVelocityBack.changeFrame(centerOfMassVelocity.getReferenceFrame());
      JUnitTools.assertFrameVectorEquals(centerOfMassVelocity, centerOfMassVelocityBack, epsilon);
   }

   private void compareAngularVelocity(RigidBody estimationLink, ReferenceFrame estimationFrame, TwistCalculator twistCalculator,
           ControlFlowOutputPort<FrameVector> angularVelocityPort, double epsilon)
   {
      Twist estimationLinkTwist = new Twist();
      twistCalculator.packTwistOfBody(estimationLinkTwist, estimationLink);
      estimationLinkTwist.changeFrame(estimationFrame);
      FrameVector angularVelocityBack = new FrameVector(estimationFrame);
      estimationLinkTwist.packAngularPart(angularVelocityBack);
      JUnitTools.assertFrameVectorEquals(angularVelocityBack, angularVelocityPort.getData(), epsilon);
   }

   private void compareAngularAcceleration(RigidBody estimationLink, ReferenceFrame estimationFrame,
           SpatialAccelerationCalculator spatialAccelerationCalculator, ControlFlowOutputPort<FrameVector> angularAccelerationPort, double epsilon)
   {
      SpatialAccelerationVector estimationLinkAcceleration = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packAccelerationOfBody(estimationLinkAcceleration, estimationLink);
      FrameVector angularAccelerationBack = new FrameVector(estimationFrame);
      estimationLinkAcceleration.packAngularPart(angularAccelerationBack);
      angularAccelerationBack.changeFrame(estimationFrame);
      JUnitTools.assertFrameVectorEquals(angularAccelerationBack, angularAccelerationPort.getData(), epsilon);

   }
}
