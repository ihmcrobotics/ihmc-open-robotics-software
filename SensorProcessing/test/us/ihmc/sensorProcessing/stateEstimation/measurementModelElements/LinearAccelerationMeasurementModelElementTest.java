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
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedLinearAccelerationSensor;
import us.ihmc.sensorProcessing.stateEstimation.OrientationAndPositionFullRobotModelUpdater;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class LinearAccelerationMeasurementModelElementTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      Random random = new Random(125125523L);
      Vector3D[] jointAxes = new Vector3D[] {X, Y, Z};
      RandomFloatingChain randomFloatingChain = new RandomFloatingChain(random, jointAxes);
      RigidBody elevator = randomFloatingChain.getElevator();
      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();

      RigidBody estimationLink = randomFloatingChain.getRootJoint().getSuccessor();
      ReferenceFrame estimationFrame = randomFloatingChain.getRootJoint().getFrameAfterJoint();
      RigidBody measurementLink = randomFloatingChain.getRevoluteJoints().get(jointAxes.length - 1).getSuccessor();
      ReferenceFrame measurementFrame = measurementLink.getParentJoint().getFrameAfterJoint(); // measurementLink.getBodyFixedFrame();

      ControlFlowElement controlFlowElement = new NullControlFlowElement();

      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootJoint);
      ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort = new ControlFlowInputPort<FullInverseDynamicsStructure>("inverseDynamicsStructureInputPort", controlFlowElement);
      inverseDynamicsStructureInputPort.setData(inverseDynamicsStructure);
      
//      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator);
//      ControlFlowInputPort<TwistCalculator> twistCalculatorInputPort = new ControlFlowInputPort<TwistCalculator>(controlFlowElement);
//      twistCalculatorInputPort.setData(twistCalculator);
//      
//      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, twistCalculator, 0.0, false);
//      ControlFlowInputPort<SpatialAccelerationCalculator> spatialAccelerationCalculatorInputPort = new ControlFlowInputPort<SpatialAccelerationCalculator>(controlFlowElement);
//      spatialAccelerationCalculatorInputPort.setData(spatialAccelerationCalculator);
      
      
      String name = "test";
      YoVariableRegistry registry = new YoVariableRegistry(name);

      ControlFlowOutputPort<FramePoint> centerOfMassPositionPort = new ControlFlowOutputPort<FramePoint>("centerOfMassPositionPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort = new ControlFlowOutputPort<FrameVector>("centerOfMassVelocityPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort = new ControlFlowOutputPort<FrameVector>("centerOfMassAccelerationPort", controlFlowElement);

      ControlFlowOutputPort<FrameOrientation> orientationPort = new ControlFlowOutputPort<FrameOrientation>("orientationPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector> angularVelocityPort = new ControlFlowOutputPort<FrameVector>("angularVelocityPort", controlFlowElement);
      ControlFlowOutputPort<FrameVector> angularAccelerationPort = new ControlFlowOutputPort<FrameVector>("angularAccelerationPort", controlFlowElement);

      ControlFlowOutputPort<FrameVector> biasPort = new ControlFlowOutputPort<FrameVector>("biasPort", controlFlowElement);

      ControlFlowInputPort<Vector3D> linearAccelerationMeasurementInputPort = new ControlFlowInputPort<Vector3D>("linearAccelerationMeasurementInputPort", controlFlowElement);
      double gZ = -9.81;
      Vector3D gravitationalAcceleration = new Vector3D(0.0, 0.0, -gZ);

      SimulatedLinearAccelerationSensor sensor = new SimulatedLinearAccelerationSensor("test", measurementLink, measurementFrame, inverseDynamicsStructure.getSpatialAccelerationCalculator(), gravitationalAcceleration, registry);
      
      LinearAccelerationMeasurementModelElement modelElement = new LinearAccelerationMeasurementModelElement(name, registry, centerOfMassPositionPort,
                                                                  centerOfMassVelocityPort, centerOfMassAccelerationPort, orientationPort, angularVelocityPort,
                                                                  angularAccelerationPort, biasPort, linearAccelerationMeasurementInputPort,
                                                                  inverseDynamicsStructureInputPort, measurementLink, measurementFrame, estimationLink,
                                                                  estimationFrame, gZ);

      randomFloatingChain.setRandomPositionsAndVelocities(random);
      randomFloatingChain.setRandomAccelerations(random);
      elevator.updateFramesRecursively();
      
      inverseDynamicsStructure.updateInternalState();

      Runnable updater = new OrientationAndPositionFullRobotModelUpdater(inverseDynamicsStructureInputPort, centerOfMassPositionPort,
                            centerOfMassVelocityPort, centerOfMassAccelerationPort, orientationPort, angularVelocityPort, angularAccelerationPort);

      centerOfMassPositionPort.setData(new FramePoint(ReferenceFrame.getWorldFrame(), RandomTools.generateRandomVector(random)));
      centerOfMassVelocityPort.setData(new FrameVector(ReferenceFrame.getWorldFrame(), RandomTools.generateRandomVector(random)));
      centerOfMassAccelerationPort.setData(new FrameVector(ReferenceFrame.getWorldFrame(), RandomTools.generateRandomVector(random)));
      RotationMatrix orientation = new RotationMatrix();
      orientation.set(RandomTools.generateRandomRotation(random));
      orientationPort.setData(new FrameOrientation(ReferenceFrame.getWorldFrame(), orientation));
      angularVelocityPort.setData(new FrameVector(estimationFrame, RandomTools.generateRandomVector(random)));
      angularAccelerationPort.setData(new FrameVector(estimationFrame, RandomTools.generateRandomVector(random)));
      biasPort.setData(new FrameVector(measurementFrame, RandomTools.generateRandomVector(random)));


      updater.run();
      sensor.startComputation();
      Vector3D measurement = sensor.getLinearAccelerationOutputPort().getData();
      measurement.add(biasPort.getData().getVector());
      linearAccelerationMeasurementInputPort.setData(measurement);

      DenseMatrix64F zeroResidual = modelElement.computeResidual();
      DenseMatrix64F zeroVector = new DenseMatrix64F(3, 1);
      EjmlUnitTests.assertEquals(zeroVector, zeroResidual, 1e-12);

      double perturbation = 1e-6;
      double tol = 1.0e-6;//1e-11;
      modelElement.computeMatrixBlocks();

      // CoM velocity perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(centerOfMassVelocityPort, modelElement,
            new FrameVector(centerOfMassVelocityPort.getData()), perturbation, tol, updater);

      // CoM acceleration perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(centerOfMassAccelerationPort, modelElement,
              new FrameVector(centerOfMassAccelerationPort.getData()), perturbation, tol, updater);

      // orientation perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(orientationPort, modelElement, new FrameOrientation(orientationPort.getData()),
              perturbation, tol, updater);

      // angular velocity perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(angularVelocityPort, modelElement, new FrameVector(angularVelocityPort.getData()),
              perturbation, tol, updater);

      // angular acceleration perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(angularAccelerationPort, modelElement,
            new FrameVector(angularAccelerationPort.getData()), perturbation, tol, updater);

      // bias perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(biasPort, modelElement,
            new FrameVector(biasPort.getData()), perturbation, tol, updater);
   }
}
