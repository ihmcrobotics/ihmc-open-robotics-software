package us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePoseTrajectoryGenerator;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class RobotArmController implements RobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final RobotArm robotArm;
   private final DoubleYoVariable yoTime;
   private final CenterOfMassReferenceFrame centerOfMassFrame;
   private final TwistCalculator twistCalculator;
   private final GeometricJacobianHolder geometricJacobianHolder;

   private final SpatialFeedbackControlCommand handFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

   private final WholeBodyControllerCore controllerCore;

   private final DoubleYoVariable handWeight = new DoubleYoVariable("handWeight", registry);
   private final YoSymmetricSE3PIDGains handGains = new YoSymmetricSE3PIDGains("hand", registry);
   private final YoFramePoint handTargetPosition = new YoFramePoint("handTarget", worldFrame, registry);
   private final YoFrameOrientation handTargetOrientation = new YoFrameOrientation("handTarget", worldFrame, registry);
   private final BooleanYoVariable goToTarget = new BooleanYoVariable("goToTarget", registry);
   private final DoubleYoVariable trajectoryDuration = new DoubleYoVariable("handTrajectoryDuration", registry);
   private final DoubleYoVariable trajectoryStartTime = new DoubleYoVariable("handTrajectoryStartTime", registry);

   private final StraightLinePoseTrajectoryGenerator trajectory = new StraightLinePoseTrajectoryGenerator("handTrajectory", worldFrame, registry);

   private final BooleanYoVariable controlLinearX = new BooleanYoVariable("controlLinearX", registry);
   private final BooleanYoVariable controlLinearY = new BooleanYoVariable("controlLinearY", registry);
   private final BooleanYoVariable controlLinearZ = new BooleanYoVariable("controlLinearZ", registry);
   private final BooleanYoVariable controlAngularX = new BooleanYoVariable("controlAngularX", registry);
   private final BooleanYoVariable controlAngularY = new BooleanYoVariable("controlAngularY", registry);
   private final BooleanYoVariable controlAngularZ = new BooleanYoVariable("controlAngularZ", registry);

   public RobotArmController(RobotArm robotArm, double controlDT, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robotArm = robotArm;
      yoTime = robotArm.getYoTime();
      double gravityZ = robotArm.getGravity();
      RigidBody hand = robotArm.getHand();
      RigidBody elevator = robotArm.getElevator();
      InverseDynamicsJoint[] controlledJoints = ScrewTools.computeSupportAndSubtreeJoints(elevator);
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);
      twistCalculator = new TwistCalculator(worldFrame, elevator);
      geometricJacobianHolder = new GeometricJacobianHolder();
      ControllerCoreOptimizationSettings optimizationSettings = new RobotArmControllerCoreOptimizationSettings();
      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(controlDT, gravityZ, null, controlledJoints, centerOfMassFrame,
                                                                                       twistCalculator, geometricJacobianHolder, optimizationSettings,
                                                                                       yoGraphicsListRegistry, registry);
      controlCoreToolbox.setupForInverseDynamicsSolver(new ArrayList<>());
      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();

      handFeedbackControlCommand.set(elevator, hand);
      allPossibleCommands.addCommand(handFeedbackControlCommand);

      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, allPossibleCommands, registry);

      initialize();
   }

   @Override
   public void initialize()
   {
      robotArm.updateIDRobot();

      handWeight.set(1.0);
      handGains.setProportionalGain(100.0);
      handGains.setDampingRatio(1.0);
      handGains.createDerivativeGainUpdater(true);

      FramePoint initialPosition = new FramePoint(robotArm.getHandControlFrame());
      initialPosition.changeFrame(worldFrame);
      FrameOrientation initialOrientation = new FrameOrientation(robotArm.getHandControlFrame());
      initialOrientation.changeFrame(worldFrame);

      handTargetPosition.setAndMatchFrame(initialPosition);
      handTargetOrientation.setAndMatchFrame(initialOrientation);

      trajectoryDuration.set(0.5);
      trajectory.setInitialPose(initialPosition, initialOrientation);
      trajectory.setFinalPose(initialPosition, initialOrientation);
      trajectory.setTrajectoryTime(trajectoryDuration.getDoubleValue());

      controlLinearX.set(true);
      controlLinearY.set(true);
      controlLinearZ.set(true);
      controlAngularX.set(true);
      controlAngularY.set(true);
      controlAngularZ.set(true);
   }

   private final FramePoint position = new FramePoint();
   private final FrameVector linearVelocity = new FrameVector();
   private final FrameVector linearAcceleration = new FrameVector();
   private final FrameOrientation orientation = new FrameOrientation();
   private final FrameVector angularVelocity = new FrameVector();
   private final FrameVector angularAcceleration = new FrameVector();

   @Override
   public void doControl()
   {
      robotArm.updateIDRobot();
      centerOfMassFrame.update();
      twistCalculator.compute();
      geometricJacobianHolder.compute();

      FramePose controlFramePose = new FramePose(robotArm.getHandControlFrame());
      controlFramePose.changeFrame(robotArm.getHand().getBodyFixedFrame());

      updateTrajectory();
      trajectory.getAngularData(orientation, angularVelocity, angularAcceleration);
      trajectory.getLinearData(position, linearVelocity, linearAcceleration);

      handFeedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePose);
      handFeedbackControlCommand.setWeightForSolver(handWeight.getDoubleValue());
      handFeedbackControlCommand.setGains((SE3PIDGainsInterface) handGains);
      handFeedbackControlCommand.setSelectionMatrix(computeSelectionMatrix());
      handFeedbackControlCommand.set(position, linearVelocity, linearAcceleration);
      handFeedbackControlCommand.set(orientation, angularVelocity, angularAcceleration);

      controllerCoreCommand.clear();
      controllerCoreCommand.addFeedbackControlCommand(handFeedbackControlCommand);
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      robotArm.updateSCSRobot();
   }

   public void updateTrajectory()
   {
      if (goToTarget.getBooleanValue())
      {
         FramePoint initialPosition = new FramePoint(robotArm.getHandControlFrame());
         initialPosition.changeFrame(worldFrame);
         FrameOrientation initialOrientation = new FrameOrientation(robotArm.getHandControlFrame());
         initialOrientation.changeFrame(worldFrame);
         trajectory.setInitialPose(initialPosition, initialOrientation);
         FramePoint finalPosition = new FramePoint();
         FrameOrientation finalOrientation = new FrameOrientation();
         handTargetPosition.getFrameTupleIncludingFrame(finalPosition);
         handTargetOrientation.getFrameOrientationIncludingFrame(finalOrientation);
         trajectory.setFinalPose(finalPosition, finalOrientation);
         trajectory.setTrajectoryTime(trajectoryDuration.getDoubleValue());
         trajectory.initialize();
         trajectoryStartTime.set(yoTime.getDoubleValue());
         goToTarget.set(false);
      }

      trajectory.compute(yoTime.getDoubleValue() - trajectoryStartTime.getDoubleValue());
   }

   private DenseMatrix64F computeSelectionMatrix()
   {
      DenseMatrix64F selectionMatrix = CommonOps.identity(6);
      if (!controlLinearZ.getBooleanValue())
         MatrixTools.removeRow(selectionMatrix, 5);
      if (!controlLinearY.getBooleanValue())
         MatrixTools.removeRow(selectionMatrix, 4);
      if (!controlLinearX.getBooleanValue())
         MatrixTools.removeRow(selectionMatrix, 3);

      if (!controlAngularZ.getBooleanValue())
         MatrixTools.removeRow(selectionMatrix, 2);
      if (!controlAngularY.getBooleanValue())
         MatrixTools.removeRow(selectionMatrix, 1);
      if (!controlAngularX.getBooleanValue())
         MatrixTools.removeRow(selectionMatrix, 0);

      return selectionMatrix;
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return name;
   }
}
