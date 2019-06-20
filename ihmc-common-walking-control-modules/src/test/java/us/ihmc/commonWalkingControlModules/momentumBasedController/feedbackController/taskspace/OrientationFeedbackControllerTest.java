package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.NormOps;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInput;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class OrientationFeedbackControllerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testCompareAgainstSpatialController() throws Exception
   {
      Random random = new Random(5641654L);

      YoVariableRegistry registry = new YoVariableRegistry("Dummy");
      int numberOfRevoluteJoints = 10;
      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, numberOfRevoluteJoints);
      List<RevoluteJoint> joints = randomFloatingChain.getRevoluteJoints();
      RigidBodyBasics elevator = randomFloatingChain.getElevator();
      RigidBodyBasics endEffector = joints.get(joints.size() - 1).getSuccessor();

      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);
      JointBasics[] jointsToOptimizeFor = MultiBodySystemTools.collectSupportAndSubtreeJoints(elevator);
      double controlDT = 0.004;

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, 0.0, null, jointsToOptimizeFor, centerOfMassFrame, null, null,
                                                                            registry);
      toolbox.setupForInverseDynamicsSolver(null);
      // Making the controllers to run with different instances of the toolbox so they don't share variables.
      OrientationFeedbackController orientationFeedbackController = new OrientationFeedbackController(endEffector, toolbox, new FeedbackControllerToolbox(new YoVariableRegistry("Dummy")), registry);
      SpatialFeedbackController spatialFeedbackController = new SpatialFeedbackController(endEffector, toolbox, new FeedbackControllerToolbox(new YoVariableRegistry("Dummy")), registry);
      orientationFeedbackController.setEnabled(true);
      spatialFeedbackController.setEnabled(true);

      OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
      orientationFeedbackControlCommand.set(elevator, endEffector);
      PID3DGains orientationGains = new DefaultPID3DGains();

      SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
      spatialFeedbackControlCommand.set(elevator, endEffector);
      spatialFeedbackControlCommand.getSpatialAccelerationCommand().setSelectionMatrixForAngularControl();

      MotionQPInputCalculator motionQPInputCalculator = toolbox.getMotionQPInputCalculator();
      QPInput orientationMotionQPInput = new QPInput(toolbox.getJointIndexHandler().getNumberOfDoFs());
      QPInput spatialMotionQPInput = new QPInput(toolbox.getJointIndexHandler().getNumberOfDoFs());

      SpatialAccelerationCommand orientationControllerOutput = orientationFeedbackController.getInverseDynamicsOutput();
      SpatialAccelerationCommand spatialControllerOutput = spatialFeedbackController.getInverseDynamicsOutput();

      for (int i = 0; i < 300; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, joints);
         joints.get(0).getPredecessor().updateFramesRecursively();
         centerOfMassFrame.update();

         double proportionalGain = RandomNumbers.nextDouble(random, 10.0, 200.0);
         double derivativeGain = RandomNumbers.nextDouble(random, 0.0, 100.0);
         double integralGain = RandomNumbers.nextDouble(random, 0.0, 100.0);
         double maxIntegralError = RandomNumbers.nextDouble(random, 0.0, 10.0);
         orientationGains.setGains(proportionalGain, derivativeGain, integralGain, maxIntegralError);
         orientationGains.setMaxProportionalError(RandomNumbers.nextDouble(random, 0.0, 10.0));
         orientationGains.setMaxDerivativeError(RandomNumbers.nextDouble(random, 0.0, 10.0));
         orientationGains.setMaxFeedbackAndFeedbackRate(RandomNumbers.nextDouble(random, 0.1, 10.0), RandomNumbers.nextDouble(random, 0.1, 10.0));
         orientationFeedbackControlCommand.setGains(orientationGains);
         spatialFeedbackControlCommand.setOrientationGains(orientationGains);

         FrameQuaternion desiredOrientation = new FrameQuaternion(worldFrame, EuclidCoreRandomTools.nextQuaternion(random));
         FrameVector3D desiredAngularVelocity = new FrameVector3D(worldFrame, EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));
         FrameVector3D feedForwardAngularAcceleration = new FrameVector3D(worldFrame, EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0));

         orientationFeedbackControlCommand.setInverseDynamics(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
         spatialFeedbackControlCommand.setInverseDynamics(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

         spatialFeedbackController.submitFeedbackControlCommand(spatialFeedbackControlCommand);
         orientationFeedbackController.submitFeedbackControlCommand(orientationFeedbackControlCommand);

         spatialFeedbackController.computeInverseDynamics();
         orientationFeedbackController.computeInverseDynamics();

         motionQPInputCalculator.convertSpatialAccelerationCommand(orientationControllerOutput, orientationMotionQPInput);
         motionQPInputCalculator.convertSpatialAccelerationCommand(spatialControllerOutput, spatialMotionQPInput);

         assertEquals(spatialMotionQPInput.taskJacobian, orientationMotionQPInput.taskJacobian, 1.0e-12);
         assertEquals(spatialMotionQPInput.taskObjective, orientationMotionQPInput.taskObjective, 1.0e-12);
         assertEquals(spatialMotionQPInput.taskWeightMatrix, orientationMotionQPInput.taskWeightMatrix, 1.0e-12);
      }
   }

   private static void assertEquals(DenseMatrix64F expected, DenseMatrix64F actual, double epsilon)
   {
      assertTrue(assertErrorMessage(expected, actual), MatrixFeatures.isEquals(expected, actual, epsilon));
   }

   private static String assertErrorMessage(DenseMatrix64F expected, DenseMatrix64F actual)
   {
      DenseMatrix64F diff = new DenseMatrix64F(expected.getNumRows(), expected.getNumCols());
      CommonOps.subtract(expected, actual, diff);
      return "Expected:\n" + expected + "\nActual:\n" + actual + ", difference: " + NormOps.normP2(diff);
   }
}
