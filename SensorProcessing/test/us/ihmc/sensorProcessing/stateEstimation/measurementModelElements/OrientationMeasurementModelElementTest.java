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
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;
import us.ihmc.robotics.screwTheory.SixDoFJoint;

public class OrientationMeasurementModelElementTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      Random random = new Random(125125523L);
      Vector3D[] jointAxes = new Vector3D[] { X, Y, Z };
      RandomFloatingChain randomFloatingChain = new RandomFloatingChain(random, jointAxes);
      final RigidBody elevator = randomFloatingChain.getElevator();
      final SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();

      ReferenceFrame estimationFrame = randomFloatingChain.getRootJoint().getFrameAfterJoint();
      RigidBody measurementLink = randomFloatingChain.getRevoluteJoints().get(jointAxes.length - 1).getSuccessor();
      ReferenceFrame measurementFrame = measurementLink.getParentJoint().getFrameAfterJoint(); // measurementLink.getBodyFixedFrame();

      ControlFlowElement controlFlowElement = new NullControlFlowElement();

      final ControlFlowOutputPort<FrameOrientation> orientationPort = new ControlFlowOutputPort<FrameOrientation>("orientationPort", controlFlowElement);
      ControlFlowInputPort<RotationMatrix> orientationMeasurementInputPort = new ControlFlowInputPort<RotationMatrix>("orientationMeasurementInputPort", controlFlowElement);
      String name = "test";
      YoVariableRegistry registry = new YoVariableRegistry(name);

      OrientationMeasurementModelElement modelElement = new OrientationMeasurementModelElement(orientationPort, orientationMeasurementInputPort,
            estimationFrame, measurementFrame, name, registry);

      RotationMatrix orientation = new RotationMatrix();
      orientation.set(RandomTools.generateRandomRotation(random));
      orientationPort.setData(new FrameOrientation(ReferenceFrame.getWorldFrame(), orientation));

      randomFloatingChain.setRandomPositionsAndVelocities(random);

      Runnable orientationUpdater = new Runnable()
      {

         public void run()
         {
            rootJoint.setRotation(orientationPort.getData().getMatrix3dCopy());
            elevator.updateFramesRecursively();
         }
      };
      orientationUpdater.run();

      RigidBodyTransform transformFromMeasurementToWorld = measurementFrame.getTransformToDesiredFrame(elevator.getBodyFixedFrame());
      RotationMatrix rotationFromMeasurementToWorld = new RotationMatrix();
      transformFromMeasurementToWorld.getRotation(rotationFromMeasurementToWorld);
      orientationMeasurementInputPort.setData(rotationFromMeasurementToWorld);

      DenseMatrix64F zeroResidual = modelElement.computeResidual();
      DenseMatrix64F zeroVector = new DenseMatrix64F(3, 1);
      EjmlUnitTests.assertEquals(zeroVector, zeroResidual, 1e-12);

      double perturbation = 1e-3;
      double tol = 1e-11;
      modelElement.computeMatrixBlocks();

      // orientation perturbations
      MeasurementModelTestTools.assertOutputMatrixCorrectUsingPerturbation(orientationPort, modelElement, new FrameOrientation(orientationPort.getData()),
            perturbation, tol, orientationUpdater);
   }
}
