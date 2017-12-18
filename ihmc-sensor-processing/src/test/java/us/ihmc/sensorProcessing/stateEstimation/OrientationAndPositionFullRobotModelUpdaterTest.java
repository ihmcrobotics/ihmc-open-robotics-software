package us.ihmc.sensorProcessing.stateEstimation;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.controlFlow.NullControlFlowElement;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomGeometry;
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

      ControlFlowOutputPort<FramePoint3D> centerOfMassPositionPort = new ControlFlowOutputPort<FramePoint3D>("centerOfMassPositionPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityPort = new ControlFlowOutputPort<FrameVector3D>("centerOfMassVelocityPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector3D> centerOfMassAccelerationPort = new ControlFlowOutputPort<FrameVector3D>("centerOfMassAccelerationPort", controlFlowElement);

      ControlFlowOutputPort<FrameQuaternion> orientationPort = new ControlFlowOutputPort<FrameQuaternion>("orientationPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector3D> angularVelocityPort = new ControlFlowOutputPort<FrameVector3D>("angularVelocityPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector3D> angularAccelerationPort = new ControlFlowOutputPort<FrameVector3D>("angularAccelerationPort", controlFlowElement);

      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootJoint);
      ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort =
         new ControlFlowInputPort<FullInverseDynamicsStructure>("inverseDynamicsStructureInputPort", controlFlowElement);
      inverseDynamicsStructureInputPort.setData(inverseDynamicsStructure);

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator); 
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
         centerOfMassPositionPort.setData(new FramePoint3D(ReferenceFrame.getWorldFrame(), RandomGeometry.nextVector3D(random)));
         centerOfMassVelocityPort.setData(new FrameVector3D(ReferenceFrame.getWorldFrame(), RandomGeometry.nextVector3D(random)));
         centerOfMassAccelerationPort.setData(new FrameVector3D(ReferenceFrame.getWorldFrame(), RandomGeometry.nextVector3D(random)));
         RotationMatrix orientation = new RotationMatrix();
         orientation.set(RandomGeometry.nextAxisAngle(random));
         orientationPort.setData(new FrameQuaternion(ReferenceFrame.getWorldFrame(), orientation));
         angularVelocityPort.setData(new FrameVector3D(estimationFrame, RandomGeometry.nextVector3D(random)));
         angularAccelerationPort.setData(new FrameVector3D(estimationFrame, RandomGeometry.nextVector3D(random)));

         // update full robot model
         fullRobotModelUpdater.run();
         twistCalculator.compute();

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

   private void compareCenterOfMass(RigidBody elevator, ControlFlowOutputPort<FramePoint3D> centerOfMassPositionPort, double epsilon)
   {
      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(elevator, ReferenceFrame.getWorldFrame());
      centerOfMassCalculator.compute();
      FramePoint3D centerOfMassBack = new FramePoint3D(ReferenceFrame.getWorldFrame());
      centerOfMassCalculator.getCenterOfMass(centerOfMassBack);
      EuclidFrameTestTools.assertFrameTuple3DEquals(centerOfMassPositionPort.getData(), centerOfMassBack, epsilon);
   }

   private void compareOrientation(ReferenceFrame estimationFrame, ControlFlowOutputPort<FrameQuaternion> orientationPort, double epsilon)
   {
      FrameQuaternion estimationFrameOrientation = new FrameQuaternion(estimationFrame);
      estimationFrameOrientation.changeFrame(orientationPort.getData().getReferenceFrame());
      RotationMatrix rotationBack = new RotationMatrix(estimationFrameOrientation);
      RotationMatrix rotation = new RotationMatrix(orientationPort.getData());
      assertTrue(rotationBack.epsilonEquals(rotation, epsilon));
   }

   private void compareCenterOfMassVelocity(RigidBody elevator, ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityPort, double epsilon)
   {
      CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(elevator);
      centerOfMassJacobian.compute();
      FrameVector3D centerOfMassVelocityBack = new FrameVector3D(ReferenceFrame.getWorldFrame());
      centerOfMassJacobian.getCenterOfMassVelocity(centerOfMassVelocityBack);
      FrameVector3D centerOfMassVelocity = centerOfMassVelocityPort.getData();
      centerOfMassVelocityBack.changeFrame(centerOfMassVelocity.getReferenceFrame());
      EuclidFrameTestTools.assertFrameTuple3DEquals(centerOfMassVelocity, centerOfMassVelocityBack, epsilon);
   }

   private void compareCenterOfMassAcceleration(RigidBody elevator, SpatialAccelerationCalculator spatialAccelerationCalculator,
           ControlFlowOutputPort<FrameVector3D> centerOfMassAccelerationPort, double epsilon)
   {
      CenterOfMassAccelerationCalculator centerOfMassAccelerationCalculator = new CenterOfMassAccelerationCalculator(elevator, spatialAccelerationCalculator);
      FrameVector3D centerOfMassAccelerationBack = new FrameVector3D();
      centerOfMassAccelerationCalculator.getCoMAcceleration(centerOfMassAccelerationBack);
      centerOfMassAccelerationBack.changeFrame(ReferenceFrame.getWorldFrame());
      FrameVector3D centerOfMassAcceleration = centerOfMassAccelerationPort.getData();
      EuclidFrameTestTools.assertFrameTuple3DEquals(centerOfMassAcceleration, centerOfMassAccelerationBack, epsilon);
   }

   private void compareAngularVelocity(RigidBody estimationLink, ReferenceFrame estimationFrame, TwistCalculator twistCalculator,
           ControlFlowOutputPort<FrameVector3D> angularVelocityPort, double epsilon)
   {
      Twist estimationLinkTwist = new Twist();
      twistCalculator.getTwistOfBody(estimationLink, estimationLinkTwist);
      estimationLinkTwist.changeFrame(estimationFrame);
      FrameVector3D angularVelocityBack = new FrameVector3D(estimationFrame);
      estimationLinkTwist.getAngularPart(angularVelocityBack);
      EuclidFrameTestTools.assertFrameTuple3DEquals(angularVelocityBack, angularVelocityPort.getData(), epsilon);
   }

   private void compareAngularAcceleration(RigidBody estimationLink, ReferenceFrame estimationFrame,
           SpatialAccelerationCalculator spatialAccelerationCalculator, ControlFlowOutputPort<FrameVector3D> angularAccelerationPort, double epsilon)
   {
      SpatialAccelerationVector estimationLinkAcceleration = new SpatialAccelerationVector();
      spatialAccelerationCalculator.getAccelerationOfBody(estimationLink, estimationLinkAcceleration);
      estimationLinkAcceleration.changeFrameNoRelativeMotion(estimationFrame);
      FrameVector3D angularAccelerationBack = new FrameVector3D(estimationFrame);
      estimationLinkAcceleration.getAngularPart(angularAccelerationBack);
      EuclidFrameTestTools.assertFrameTuple3DEquals(angularAccelerationBack, angularAccelerationPort.getData(), epsilon);
   }
}
