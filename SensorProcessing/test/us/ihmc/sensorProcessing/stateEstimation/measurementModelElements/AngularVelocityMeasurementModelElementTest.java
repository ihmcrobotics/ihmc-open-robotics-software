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
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class AngularVelocityMeasurementModelElementTest
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

      RigidBody estimationLink = randomFloatingChain.getRootJoint().getSuccessor();
      ReferenceFrame estimationFrame = randomFloatingChain.getRootJoint().getFrameAfterJoint();
      RigidBody measurementLink = randomFloatingChain.getRevoluteJoints().get(jointAxes.length - 1).getSuccessor();
      ReferenceFrame measurementFrame = measurementLink.getBodyFixedFrame();

      ControlFlowElement controlFlowElement = new NullControlFlowElement();

      ControlFlowOutputPort<FrameVector> angularVelocityStatePort = new ControlFlowOutputPort<FrameVector>("angularVelocityStatePort", controlFlowElement);
      ControlFlowOutputPort<FrameVector> biasStatePort = new ControlFlowOutputPort<FrameVector>("biasStatePort", controlFlowElement);
      ControlFlowInputPort<Vector3D> angularVelocityMeasurementInputPort = new ControlFlowInputPort<Vector3D>("angularVelocityMeasurementInputPort", controlFlowElement);
      
      
      ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort = new ControlFlowInputPort<FullInverseDynamicsStructure>("inverseDynamicsStructureInputPort", controlFlowElement);
      
      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, randomFloatingChain.getRootJoint().getSuccessor(), randomFloatingChain.getRootJoint());
      inverseDynamicsStructureInputPort.setData(inverseDynamicsStructure);
      
      TwistCalculator twistCalculator = inverseDynamicsStructure.getTwistCalculator(); 

      String name = "test";
      YoVariableRegistry registry = new YoVariableRegistry(name);
      AngularVelocityMeasurementModelElement modelElement = new AngularVelocityMeasurementModelElement(angularVelocityStatePort, biasStatePort,
                                                               angularVelocityMeasurementInputPort, estimationLink, estimationFrame, measurementLink,
                                                               measurementFrame, inverseDynamicsStructureInputPort, name, registry);

      randomFloatingChain.setRandomPositionsAndVelocities(random);
      twistCalculator.compute();

      FrameVector measuredAngularVelocity = getAngularVelocity(twistCalculator, measurementLink, measurementFrame);
      FrameVector bias = new FrameVector(measurementFrame, RandomGeometry.nextVector3D(random));
      measuredAngularVelocity.add(bias);
      angularVelocityMeasurementInputPort.setData(measuredAngularVelocity.getVectorCopy());

      biasStatePort.setData(bias);
      FrameVector angularVelocityOfEstimationLink = getAngularVelocity(twistCalculator, estimationLink, estimationFrame);
      angularVelocityStatePort.setData(angularVelocityOfEstimationLink);

      modelElement.computeMatrixBlocks();

      DenseMatrix64F zeroResidual = modelElement.computeResidual();
      DenseMatrix64F zeroVector = new DenseMatrix64F(3, 1);
      EjmlUnitTests.assertEquals(zeroVector, zeroResidual, 1e-12);

      double perturbation = 1e-5;
      double tol = 1e-12;

      // bias perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(biasStatePort, modelElement, bias, perturbation, tol, null);

      // angular velocity perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(angularVelocityStatePort, modelElement, angularVelocityOfEstimationLink,
              perturbation, tol, null);
   }

   private FrameVector getAngularVelocity(TwistCalculator twistCalculator, RigidBody rigidBody, ReferenceFrame referenceFrame)
   {
      Twist twist = new Twist();
      twistCalculator.getTwistOfBody(twist, rigidBody);
      twist.changeFrame(referenceFrame);
      FrameVector ret = new FrameVector(referenceFrame);
      twist.getAngularPart(ret);

      return ret;
   }
}
