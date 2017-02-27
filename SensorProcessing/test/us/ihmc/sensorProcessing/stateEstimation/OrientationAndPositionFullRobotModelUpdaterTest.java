package us.ihmc.sensorProcessing.stateEstimation;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import us.ihmc.euclid.tuple3D.Vector3D;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.controlFlow.NullControlFlowElement;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePointTest;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVectorTest;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CenterOfMassAccelerationCalculator;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class OrientationAndPositionFullRobotModelUpdaterTest
{
   private static final boolean TEST_EFFICIENCY = false;

   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testModelUpdatorWithRandomFloatingChain()
   {
      Random random = new Random(1235L);
      Vector3D[] jointAxes = new Vector3D[] {X, Y, Z};
      RandomFloatingChain randomFloatingChain = new RandomFloatingChain(random, jointAxes);
      RigidBody elevator = randomFloatingChain.getElevator();
      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();

      RigidBody estimationLink = randomFloatingChain.getRevoluteJoints().get(1).getSuccessor();    // some link in the middle of the chain
      ReferenceFrame estimationFrame = estimationLink.getParentJoint().getFrameAfterJoint(); //.getBodyFixedFrame();

      ControlFlowElement controlFlowElement = new NullControlFlowElement();

      ControlFlowOutputPort<FramePoint> centerOfMassPositionPort = new ControlFlowOutputPort<FramePoint>("centerOfMassPositionPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort = new ControlFlowOutputPort<FrameVector>("centerOfMassVelocityPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort = new ControlFlowOutputPort<FrameVector>("centerOfMassAccelerationPort", controlFlowElement);

      ControlFlowOutputPort<FrameOrientation> orientationPort = new ControlFlowOutputPort<FrameOrientation>("orientationPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector> angularVelocityPort = new ControlFlowOutputPort<FrameVector>("angularVelocityPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector> angularAccelerationPort = new ControlFlowOutputPort<FrameVector>("angularAccelerationPort", controlFlowElement);

      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootJoint);
      ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort =
         new ControlFlowInputPort<FullInverseDynamicsStructure>("inverseDynamicsStructureInputPort", controlFlowElement);
      inverseDynamicsStructureInputPort.setData(inverseDynamicsStructure);

      TwistCalculator twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      SpatialAccelerationCalculator spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();

      OrientationAndPositionFullRobotModelUpdater fullRobotModelUpdater = new OrientationAndPositionFullRobotModelUpdater(inverseDynamicsStructureInputPort,
                                                                        centerOfMassPositionPort, centerOfMassVelocityPort, centerOfMassAccelerationPort,
                                                                        orientationPort, angularVelocityPort, angularAccelerationPort);

      randomFloatingChain.setRandomPositionsAndVelocities(random);
      randomFloatingChain.getElevator().updateFramesRecursively();
      inverseDynamicsStructure.updateInternalState();

      int nTests = 100;
      for (int i = 0; i < nTests; i++)
      {
         centerOfMassPositionPort.setData(new FramePoint(ReferenceFrame.getWorldFrame(), RandomGeometry.nextVector3D(random)));
         centerOfMassVelocityPort.setData(new FrameVector(ReferenceFrame.getWorldFrame(), RandomGeometry.nextVector3D(random)));
         centerOfMassAccelerationPort.setData(new FrameVector(ReferenceFrame.getWorldFrame(), RandomGeometry.nextVector3D(random)));
         RotationMatrix orientation = new RotationMatrix();
         orientation.set(RandomGeometry.nextAxisAngle(random));
         orientationPort.setData(new FrameOrientation(ReferenceFrame.getWorldFrame(), orientation));
         angularVelocityPort.setData(new FrameVector(estimationFrame, RandomGeometry.nextVector3D(random)));
         angularAccelerationPort.setData(new FrameVector(estimationFrame, RandomGeometry.nextVector3D(random)));

         // update full robot model
         fullRobotModelUpdater.run();

         // compare with ports
         double epsilon = 1e-12;
         compareCenterOfMass(elevator, centerOfMassPositionPort, epsilon);
         compareCenterOfMassVelocity(elevator, centerOfMassVelocityPort, epsilon);
         compareCenterOfMassAcceleration(elevator, spatialAccelerationCalculator, centerOfMassAccelerationPort, epsilon);
         compareOrientation(estimationFrame, orientationPort, epsilon);
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
         double millisecondsPerCall = ((endTime - startTime)) / ((double) nTests);

         System.out.println("millisecondsPerCall = " + millisecondsPerCall);
         assertTrue("Not efficient! millisecondsPerCall = " + millisecondsPerCall, millisecondsPerCall < 0.03);
      }
   }

   private void compareCenterOfMass(RigidBody elevator, ControlFlowOutputPort<FramePoint> centerOfMassPositionPort, double epsilon)
   {
      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(elevator, ReferenceFrame.getWorldFrame());
      centerOfMassCalculator.compute();
      FramePoint centerOfMassBack = new FramePoint(ReferenceFrame.getWorldFrame());
      centerOfMassCalculator.getCenterOfMass(centerOfMassBack);
      FramePointTest.assertFramePointEquals(centerOfMassPositionPort.getData(), centerOfMassBack, epsilon);
   }

   private void compareOrientation(ReferenceFrame estimationFrame, ControlFlowOutputPort<FrameOrientation> orientationPort, double epsilon)
   {
      FrameOrientation estimationFrameOrientation = new FrameOrientation(estimationFrame);
      estimationFrameOrientation.changeFrame(orientationPort.getData().getReferenceFrame());
      RotationMatrix rotationBack = estimationFrameOrientation.getMatrix3dCopy();
      RotationMatrix rotation = orientationPort.getData().getMatrix3dCopy();
      assertTrue(rotationBack.epsilonEquals(rotation, epsilon));
   }

   private void compareCenterOfMassVelocity(RigidBody elevator, ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort, double epsilon)
   {
      CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(elevator);
      centerOfMassJacobian.compute();
      FrameVector centerOfMassVelocityBack = new FrameVector(ReferenceFrame.getWorldFrame());
      centerOfMassJacobian.getCenterOfMassVelocity(centerOfMassVelocityBack);
      FrameVector centerOfMassVelocity = centerOfMassVelocityPort.getData();
      centerOfMassVelocityBack.changeFrame(centerOfMassVelocity.getReferenceFrame());
      FrameVectorTest.assertFrameVectorEquals(centerOfMassVelocity, centerOfMassVelocityBack, epsilon);
   }

   private void compareCenterOfMassAcceleration(RigidBody elevator, SpatialAccelerationCalculator spatialAccelerationCalculator,
           ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort, double epsilon)
   {
      CenterOfMassAccelerationCalculator centerOfMassAccelerationCalculator = new CenterOfMassAccelerationCalculator(elevator, spatialAccelerationCalculator);
      FrameVector centerOfMassAccelerationBack = new FrameVector();
      centerOfMassAccelerationCalculator.getCoMAcceleration(centerOfMassAccelerationBack);
      centerOfMassAccelerationBack.changeFrame(ReferenceFrame.getWorldFrame());
      FrameVector centerOfMassAcceleration = centerOfMassAccelerationPort.getData();
      FrameVectorTest.assertFrameVectorEquals(centerOfMassAcceleration, centerOfMassAccelerationBack, epsilon);
   }

   private void compareAngularVelocity(RigidBody estimationLink, ReferenceFrame estimationFrame, TwistCalculator twistCalculator,
           ControlFlowOutputPort<FrameVector> angularVelocityPort, double epsilon)
   {
      Twist estimationLinkTwist = new Twist();
      twistCalculator.getTwistOfBody(estimationLinkTwist, estimationLink);
      estimationLinkTwist.changeFrame(estimationFrame);
      FrameVector angularVelocityBack = new FrameVector(estimationFrame);
      estimationLinkTwist.getAngularPart(angularVelocityBack);
      FrameVectorTest.assertFrameVectorEquals(angularVelocityBack, angularVelocityPort.getData(), epsilon);
   }

   private void compareAngularAcceleration(RigidBody estimationLink, ReferenceFrame estimationFrame,
           SpatialAccelerationCalculator spatialAccelerationCalculator, ControlFlowOutputPort<FrameVector> angularAccelerationPort, double epsilon)
   {
      SpatialAccelerationVector estimationLinkAcceleration = new SpatialAccelerationVector();
      spatialAccelerationCalculator.getAccelerationOfBody(estimationLinkAcceleration, estimationLink);
      estimationLinkAcceleration.changeFrameNoRelativeMotion(estimationFrame);
      FrameVector angularAccelerationBack = new FrameVector(estimationFrame);
      estimationLinkAcceleration.getAngularPart(angularAccelerationBack);
      FrameVectorTest.assertFrameVectorEquals(angularAccelerationBack, angularAccelerationPort.getData(), epsilon);
   }
}
