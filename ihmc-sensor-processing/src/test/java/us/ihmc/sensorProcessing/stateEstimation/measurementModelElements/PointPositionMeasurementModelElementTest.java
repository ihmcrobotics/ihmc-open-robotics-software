package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;



import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.EjmlUnitTests;
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
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.screwTheory.AfterJointReferenceFrameNameMap;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.OrientationAndPositionFullRobotModelUpdater;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;

public class PointPositionMeasurementModelElementTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      Random random = new Random(1235L);
      Vector3D[] jointAxes = new Vector3D[] {X, Y, Z};
      RandomFloatingChain randomFloatingChain = new RandomFloatingChain(random, jointAxes);
      RigidBody elevator = randomFloatingChain.getElevator();
      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();

      RigidBody estimationLink = randomFloatingChain.getRootJoint().getSuccessor();
      ReferenceFrame estimationFrame = randomFloatingChain.getRootJoint().getFrameAfterJoint();
      RigidBody measurementLink = randomFloatingChain.getRevoluteJoints().get(jointAxes.length - 1).getSuccessor();
      ReferenceFrame measurementFrame = measurementLink.getParentJoint().getFrameAfterJoint();

      ControlFlowElement controlFlowElement = new NullControlFlowElement();

      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootJoint);
      ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort =
         new ControlFlowInputPort<FullInverseDynamicsStructure>("inverseDynamicsStructureInputPort", controlFlowElement);
      inverseDynamicsStructureInputPort.setData(inverseDynamicsStructure);

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator); 
      SpatialAccelerationCalculator spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();

      String name = "test";
      YoVariableRegistry registry = new YoVariableRegistry(name);

      AfterJointReferenceFrameNameMap referenceFrameMap = new AfterJointReferenceFrameNameMap(elevator);
      ControlFlowInputPort<PointPositionDataObject> pointPositionMeasurementInputPort = new ControlFlowInputPort<PointPositionDataObject>("pointPositionMeasurementInputPort", controlFlowElement);

      ControlFlowOutputPort<FramePoint3D> centerOfMassPositionPort = new ControlFlowOutputPort<FramePoint3D>("centerOfMassPositionPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityPort = new ControlFlowOutputPort<FrameVector3D>("centerOfMassVelocityPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector3D> centerOfMassAccelerationPort = new ControlFlowOutputPort<FrameVector3D>("centerOfMassAccelerationPort", controlFlowElement);

      ControlFlowOutputPort<FrameQuaternion> orientationPort = new ControlFlowOutputPort<FrameQuaternion>("orientationPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector3D> angularVelocityPort = new ControlFlowOutputPort<FrameVector3D>("angularVelocityPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector3D> angularAccelerationPort = new ControlFlowOutputPort<FrameVector3D>("angularAccelerationPort", controlFlowElement);

      FramePoint3D stationaryPoint = new FramePoint3D(measurementFrame, RandomGeometry.nextPoint3D(random, 1.0, 1.0, 1.0));
      PointPositionMeasurementModelElement modelElement = new PointPositionMeasurementModelElement(name, pointPositionMeasurementInputPort,
            centerOfMassPositionPort, orientationPort, estimationFrame, referenceFrameMap, false, registry);

      randomFloatingChain.setRandomPositionsAndVelocities(random);
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();

      Runnable updater = new OrientationAndPositionFullRobotModelUpdater(inverseDynamicsStructureInputPort, centerOfMassPositionPort, centerOfMassVelocityPort,
                            centerOfMassAccelerationPort, orientationPort, angularVelocityPort, angularAccelerationPort);

      centerOfMassPositionPort.setData(new FramePoint3D(ReferenceFrame.getWorldFrame(), RandomGeometry.nextVector3D(random)));
      centerOfMassVelocityPort.setData(new FrameVector3D(ReferenceFrame.getWorldFrame(), RandomGeometry.nextVector3D(random)));
      centerOfMassAccelerationPort.setData(new FrameVector3D(ReferenceFrame.getWorldFrame(), RandomGeometry.nextVector3D(random)));
      RotationMatrix orientation = new RotationMatrix();
      orientation.set(RandomGeometry.nextAxisAngle(random));
      orientationPort.setData(new FrameQuaternion(ReferenceFrame.getWorldFrame(), orientation));
      angularVelocityPort.setData(new FrameVector3D(estimationFrame, RandomGeometry.nextVector3D(random)));
      angularAccelerationPort.setData(new FrameVector3D(estimationFrame, RandomGeometry.nextVector3D(random)));

      updater.run();

      setMeasuredPointPositionToActual(stationaryPoint, pointPositionMeasurementInputPort);

      DenseMatrix64F zeroResidual = modelElement.computeResidual();
      DenseMatrix64F zeroVector = new DenseMatrix64F(3, 1);
      EjmlUnitTests.assertEquals(zeroVector, zeroResidual, 1e-12);

      double perturbation = 1e-6;
      double tol = 1e-12;
      modelElement.computeMatrixBlocks();

      // CoM position perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(centerOfMassPositionPort, modelElement,
              new FramePoint3D(centerOfMassPositionPort.getData()), perturbation, tol, updater);

      // orientation perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(orientationPort, modelElement, new FrameQuaternion(orientationPort.getData()),
              perturbation, tol, updater);
   }

   private void setMeasuredPointPositionToActual(FramePoint3D point, ControlFlowInputPort<PointPositionDataObject> pointPositionMeasurementInputPort)
   {
      FramePoint3D pointInWorld = new FramePoint3D(point);
      pointInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      PointPositionDataObject pointPositionDataObject = new PointPositionDataObject();
      boolean isPointPositionValid = true;
      pointPositionDataObject.set(point, pointInWorld, isPointPositionValid);
      pointPositionMeasurementInputPort.setData(pointPositionDataObject);
   }
}
