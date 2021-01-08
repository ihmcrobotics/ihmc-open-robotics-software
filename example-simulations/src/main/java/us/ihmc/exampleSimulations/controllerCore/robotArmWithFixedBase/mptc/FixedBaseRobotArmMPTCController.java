package us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase.mptc;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePoseTrajectoryGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.exampleSimulations.controllerCore.RobotArmControllerCoreOptimizationSettings;
import us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase.FixedBaseRobotArm;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.implementations.SymmetricYoPIDSE3Gains;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.RobotJointLimitWatcher;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FixedBaseRobotArmMPTCController implements RobotController
{
   private static final boolean USE_PRIVILEGED_CONFIGURATION = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final FixedBaseRobotArm robotArm;
   private final YoDouble yoTime;
   private final CenterOfMassReferenceFrame centerOfMassFrame;

   private final SpatialFeedbackControlCommand handSpatialCommand = new SpatialFeedbackControlCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

   private final WholeBodyControlCoreToolbox controlCoreToolbox;
   private final WholeBodyControllerCore controllerCore;

   private final YoDouble handWeight = new YoDouble("handWeight", registry);
   private final SymmetricYoPIDSE3Gains handPositionGains = new SymmetricYoPIDSE3Gains("handPosition", registry);
   private final SymmetricYoPIDSE3Gains handOrientationGains = new SymmetricYoPIDSE3Gains("handOrientation", registry);
   private final YoFramePoint3D handTargetPosition = new YoFramePoint3D("handTarget", worldFrame, registry);

   private final YoFrameYawPitchRoll handTargetOrientation = new YoFrameYawPitchRoll("handTarget", worldFrame, registry);
   private final YoBoolean goToTarget = new YoBoolean("goToTarget", registry);
   private final YoDouble trajectoryDuration = new YoDouble("handTrajectoryDuration", registry);
   private final YoDouble trajectoryStartTime = new YoDouble("handTrajectoryStartTime", registry);

   private final StraightLinePoseTrajectoryGenerator trajectory;

   private final YoBoolean controlLinearX = new YoBoolean("controlLinearX", registry);
   private final YoBoolean controlLinearY = new YoBoolean("controlLinearY", registry);
   private final YoBoolean controlLinearZ = new YoBoolean("controlLinearZ", registry);
   private final YoBoolean controlAngularX = new YoBoolean("controlAngularX", registry);
   private final YoBoolean controlAngularY = new YoBoolean("controlAngularY", registry);
   private final YoBoolean controlAngularZ = new YoBoolean("controlAngularZ", registry);

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final RobotJointLimitWatcher robotJointLimitWatcher;

   private final YoBoolean setRandomConfiguration = new YoBoolean("setRandomConfiguration", registry);

   public FixedBaseRobotArmMPTCController(FixedBaseRobotArm robotArm, double controlDT, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robotArm = robotArm;

      yoTime = robotArm.getYoTime();
      double gravityZ = robotArm.getGravity();
      RigidBodyBasics hand = robotArm.getHand();
      RigidBodyBasics elevator = robotArm.getElevator();
      JointBasics[] controlledJoints = MultiBodySystemTools.collectSupportAndSubtreeJoints(elevator);
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);

      ControllerCoreOptimizationSettings optimizationSettings = new RobotArmControllerCoreOptimizationSettings();

      controlCoreToolbox = new WholeBodyControlCoreToolbox(controlDT,
                                                           gravityZ,
                                                           null,
                                                           controlledJoints,
                                                           centerOfMassFrame,
                                                           optimizationSettings,
                                                           yoGraphicsListRegistry,
                                                           registry);

      if (USE_PRIVILEGED_CONFIGURATION)
         controlCoreToolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());

      controlCoreToolbox.setupForInverseDynamicsSolver(new ArrayList<>());
      controlCoreToolbox.setupForInverseKinematicsSolver();

      FeedbackControllerTemplate template = new FeedbackControllerTemplate();

      handSpatialCommand.set(elevator, hand);
      template.enableSpatialFeedbackController(hand);

      JointDesiredOutputList lowLevelControllerCoreOutput = new JointDesiredOutputList(MultiBodySystemTools.filterJoints(controlledJoints,
                                                                                                                         OneDoFJointBasics.class));

      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, template, lowLevelControllerCoreOutput, registry);

      yoGraphicsListRegistry.registerYoGraphic("desireds",
                                               new YoGraphicCoordinateSystem("targetFrame",
                                                                             handTargetPosition,
                                                                             handTargetOrientation,
                                                                             0.15,
                                                                             YoAppearance.Red()));

      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_ZERO);
      privilegedConfigurationCommand.addJoint(robotArm.getElbowPitch(), Math.PI / 3.0);

      trajectory = new StraightLinePoseTrajectoryGenerator("handTrajectory", worldFrame, registry, true, yoGraphicsListRegistry);

      robotJointLimitWatcher = new RobotJointLimitWatcher(MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class));
      registry.addChild(robotJointLimitWatcher.getYoRegistry());

      initialize();
   }

   @Override
   public void initialize()
   {
      robotArm.updateIDRobot();

      handWeight.set(1.0);

      handPositionGains.setProportionalGains(100.0);
      handPositionGains.setDampingRatios(1.0);

      handOrientationGains.setProportionalGains(100.0);
      handOrientationGains.setDampingRatios(1.0);

      FramePoint3D initialPosition = new FramePoint3D(robotArm.getHandControlFrame());
      initialPosition.changeFrame(worldFrame);
      FrameQuaternion initialOrientation = new FrameQuaternion(robotArm.getHandControlFrame());
      initialOrientation.changeFrame(worldFrame);

      handTargetPosition.setMatchingFrame(initialPosition);
      handTargetOrientation.setMatchingFrame(initialOrientation);

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
      trajectory.showVisualization();
   }

   private final FramePoint3D position = new FramePoint3D();
   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D linearAcceleration = new FrameVector3D();
   private final FrameQuaternion orientation = new FrameQuaternion();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final FrameVector3D angularAcceleration = new FrameVector3D();

   @Override
   public void doControl()
   {
      robotArm.updateControlFrameAcceleration();
      robotArm.updateIDRobot();
      centerOfMassFrame.update();

      updateTrajectory();
      updateFeedbackCommands();

      controllerCoreCommand.clear();
      controllerCoreCommand.addFeedbackControlCommand(handSpatialCommand);
      if (USE_PRIVILEGED_CONFIGURATION)
         controllerCoreCommand.addInverseDynamicsCommand(privilegedConfigurationCommand);
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      ControllerCoreOutput controllerCoreOutput = controllerCore.getControllerCoreOutput();
      JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();

      controllerCoreCommand.setControllerCoreMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      robotArm.updateJointTaus(lowLevelOneDoFJointDesiredDataHolder);

      if (setRandomConfiguration.getBooleanValue())
      {
         robotArm.setRandomConfiguration();
         setRandomConfiguration.set(false);
      }

      robotJointLimitWatcher.doControl();
   }

   public void updateFeedbackCommands()
   {
      FramePose3D controlFramePose = new FramePose3D(robotArm.getHandControlFrame());
      controlFramePose.changeFrame(robotArm.getHand().getBodyFixedFrame());

      trajectory.getAngularData(orientation, angularVelocity, angularAcceleration);
      trajectory.getLinearData(position, linearVelocity, linearAcceleration);

      handSpatialCommand.setControlFrameFixedInEndEffector(controlFramePose);
      handSpatialCommand.setWeightForSolver(handWeight.getDoubleValue());
      handSpatialCommand.setPositionGains(handPositionGains);
      handSpatialCommand.setOrientationGains(handOrientationGains);
      handSpatialCommand.setSelectionMatrix(computeSpatialSelectionMatrix());
      handSpatialCommand.setInverseDynamics(orientation, position, angularVelocity, linearVelocity, angularAcceleration, linearAcceleration);
   }

   public void updateTrajectory()
   {
      if (goToTarget.getBooleanValue())
      {
         FramePoint3D initialPosition = new FramePoint3D(robotArm.getHandControlFrame());
         initialPosition.changeFrame(worldFrame);
         FrameQuaternion initialOrientation = new FrameQuaternion(robotArm.getHandControlFrame());
         initialOrientation.changeFrame(worldFrame);
         trajectory.setInitialPose(initialPosition, initialOrientation);
         FramePoint3D finalPosition = new FramePoint3D(handTargetPosition);
         FrameQuaternion finalOrientation = new FrameQuaternion();
         finalOrientation.setIncludingFrame(handTargetOrientation);
         trajectory.setFinalPose(finalPosition, finalOrientation);
         trajectory.setTrajectoryTime(trajectoryDuration.getDoubleValue());
         trajectory.initialize();
         trajectoryStartTime.set(yoTime.getDoubleValue());
         goToTarget.set(false);
      }

      trajectory.compute(yoTime.getDoubleValue() - trajectoryStartTime.getDoubleValue());
   }

   private SelectionMatrix6D computeSpatialSelectionMatrix()
   {
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

      selectionMatrix.selectAngularX(controlAngularX.getBooleanValue());
      selectionMatrix.selectAngularY(controlAngularY.getBooleanValue());
      selectionMatrix.selectAngularZ(controlAngularZ.getBooleanValue());

      selectionMatrix.selectLinearX(controlLinearX.getBooleanValue());
      selectionMatrix.selectLinearY(controlLinearY.getBooleanValue());
      selectionMatrix.selectLinearZ(controlLinearZ.getBooleanValue());

      return selectionMatrix;
   }

   public void setToRandomConfiguration()
   {
      setRandomConfiguration.set(true);
   }

   @Override
   public YoRegistry getYoRegistry()
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

   public YoFramePoint3D getHandTargetPosition()
   {
      return handTargetPosition;
   }

   public YoFrameYawPitchRoll getHandTargetOrientation()
   {
      return handTargetOrientation;
   }

   public YoBoolean getGoToTarget()
   {
      return goToTarget;
   }

   public WholeBodyControlCoreToolbox getControlCoreToolbox()
   {
      return controlCoreToolbox;
   }
}
