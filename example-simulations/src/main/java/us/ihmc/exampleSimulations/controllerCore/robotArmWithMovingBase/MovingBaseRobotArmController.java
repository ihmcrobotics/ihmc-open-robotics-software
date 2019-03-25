package us.ihmc.exampleSimulations.controllerCore.robotArmWithMovingBase;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
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
import us.ihmc.exampleSimulations.controllerCore.ControllerCoreModeChangedListener;
import us.ihmc.exampleSimulations.controllerCore.RobotArmControllerCoreOptimizationSettings;
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
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class MovingBaseRobotArmController implements RobotController
{
   private static final boolean USE_PRIVILEGED_CONFIGURATION = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final MovingBaseRobotArm robotArm;
   private final YoDouble yoTime;
   private final CenterOfMassReferenceFrame centerOfMassFrame;

   private final YoEnum<WholeBodyControllerCoreMode> controllerCoreMode = new YoEnum<>("controllerCoreMode", registry,
                                                                                                       WholeBodyControllerCoreMode.class);
   private final AtomicBoolean controllerCoreModeHasChanged = new AtomicBoolean(false);
   private final List<ControllerCoreModeChangedListener> controllerModeListeners = new ArrayList<>();
   private final YoDouble baseWeight = new YoDouble("baseWeight", registry);
   private final SymmetricYoPIDSE3Gains basePositionGains = new SymmetricYoPIDSE3Gains("basePosition", registry);
   private final PointFeedbackControlCommand basePointCommand = new PointFeedbackControlCommand();
   private final YoSineGenerator3D sineGenerator = new YoSineGenerator3D("baseTrajectory", worldFrame, registry);

   private final SpatialFeedbackControlCommand handSpatialCommand = new SpatialFeedbackControlCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

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
   private final ReferenceFrame baseFrame;

   public MovingBaseRobotArmController(MovingBaseRobotArm robotArm, double controlDT, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robotArm = robotArm;
      baseFrame = robotArm.getBase().getBodyFixedFrame();

      controllerCoreMode.set(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      controllerCoreMode.addVariableChangedListener(v -> controllerCoreModeHasChanged.set(true));

      yoTime = robotArm.getYoTime();
      double gravityZ = robotArm.getGravity();
      RigidBodyBasics hand = robotArm.getHand();
      RigidBodyBasics base = robotArm.getBase();
      RigidBodyBasics elevator = robotArm.getElevator();
      JointBasics[] controlledJoints = MultiBodySystemTools.collectSupportAndSubtreeJoints(elevator);
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);

      ControllerCoreOptimizationSettings optimizationSettings = new RobotArmControllerCoreOptimizationSettings();

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(controlDT, gravityZ, null, controlledJoints, centerOfMassFrame,
                                                                                       optimizationSettings, yoGraphicsListRegistry, registry);

      if (USE_PRIVILEGED_CONFIGURATION)
         controlCoreToolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());

      controlCoreToolbox.setupForInverseDynamicsSolver(new ArrayList<>());
      controlCoreToolbox.setupForInverseKinematicsSolver();

      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();

      basePointCommand.set(elevator, base);

      handSpatialCommand.set(elevator, hand);
      allPossibleCommands.addCommand(basePointCommand);
      allPossibleCommands.addCommand(handSpatialCommand);

      JointDesiredOutputList lowLevelControllerCoreOutput = new JointDesiredOutputList(MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class));


      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, allPossibleCommands, lowLevelControllerCoreOutput, registry);

      yoGraphicsListRegistry.registerYoGraphic("desireds", new YoGraphicCoordinateSystem("targetFrame", handTargetPosition, handTargetOrientation, 0.15,
                                                                                         YoAppearance.Red()));

      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_ZERO);
      privilegedConfigurationCommand.addJoint(robotArm.getElbowPitch(), Math.PI / 3.0);

      trajectory = new StraightLinePoseTrajectoryGenerator("handTrajectory", baseFrame, registry, true, yoGraphicsListRegistry);

      robotJointLimitWatcher = new RobotJointLimitWatcher(MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class));
      registry.addChild(robotJointLimitWatcher.getYoVariableRegistry());

      initialize();
   }

   public void registerControllerCoreModeChangedListener(ControllerCoreModeChangedListener listener)
   {
      controllerModeListeners.add(listener);
   }

   @Override
   public void initialize()
   {
      robotArm.updateIDRobot();

      baseWeight.set(100.0);

      basePositionGains.setProportionalGains(100.0);
      basePositionGains.setDampingRatios(1.0);

      handWeight.set(10.0);

      handPositionGains.setProportionalGains(100.0);
      handPositionGains.setDampingRatios(1.0);

      handOrientationGains.setProportionalGains(100.0);
      handOrientationGains.setDampingRatios(1.0);

      FramePoint3D initialHandPosition = new FramePoint3D(robotArm.getHandControlFrame());
      initialHandPosition.changeFrame(worldFrame);
      FrameQuaternion initialHandOrientation = new FrameQuaternion(robotArm.getHandControlFrame());
      initialHandOrientation.changeFrame(worldFrame);

      handTargetPosition.setMatchingFrame(initialHandPosition);
      handTargetOrientation.setMatchingFrame(initialHandOrientation);

      FramePoint3D initialBasePosition = new FramePoint3D(robotArm.getBase().getBodyFixedFrame());
      initialBasePosition.changeFrame(worldFrame);
      sineGenerator.setOffset(initialBasePosition);
      sineGenerator.setAmplitude(0.2, 0.2, 0.1);
      sineGenerator.setFrequency(1.5, 1.5, 1.0);
      sineGenerator.setPhase(0.0, Math.PI / 2.0, Math.PI);

      trajectoryDuration.set(0.5);
      trajectory.setInitialPose(initialHandPosition, initialHandOrientation);
      trajectory.setFinalPose(initialHandPosition, initialHandOrientation);
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

      updateBaseTrajectoryAndCommands();
      updateHandTrajectory();
      updateHandFeedbackCommands();

      controllerCoreCommand.clear();

      controllerCoreCommand.addFeedbackControlCommand(basePointCommand);
      controllerCoreCommand.addFeedbackControlCommand(handSpatialCommand);

      if (USE_PRIVILEGED_CONFIGURATION)
         controllerCoreCommand.addInverseDynamicsCommand(privilegedConfigurationCommand);
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      ControllerCoreOutput controllerCoreOutput = controllerCore.getControllerCoreOutput();
      JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();

      if (controllerCoreMode.getEnumValue() == WholeBodyControllerCoreMode.OFF
            || controllerCoreMode.getEnumValue() == WholeBodyControllerCoreMode.VIRTUAL_MODEL)
         controllerCoreMode.set(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

      if (controllerCoreModeHasChanged.getAndSet(false))
         controllerModeListeners.forEach(listener -> listener.controllerCoreModeHasChanged(controllerCoreMode.getEnumValue()));

      controllerCoreCommand.setControllerCoreMode(controllerCoreMode.getEnumValue());

      if (controllerCoreMode.getEnumValue() == WholeBodyControllerCoreMode.INVERSE_DYNAMICS)
         robotArm.updateSCSRobotJointTaus(lowLevelOneDoFJointDesiredDataHolder);
      else
         robotArm.updateSCSRobotJointConfiguration(lowLevelOneDoFJointDesiredDataHolder);

      if (setRandomConfiguration.getBooleanValue())
      {
         robotArm.setRandomConfiguration();
         setRandomConfiguration.set(false);
      }

      robotJointLimitWatcher.doControl();
   }

   private void updateBaseTrajectoryAndCommands()
   {
      basePointCommand.resetBodyFixedPoint();
      basePointCommand.setWeightForSolver(baseWeight.getDoubleValue());
      basePointCommand.setGains(basePositionGains);
      FramePoint3D desiredPosition = new FramePoint3D();
      FrameVector3D desiredLinearVelocity = new FrameVector3D();
      FrameVector3D feedForwardLinearAcceleration = new FrameVector3D();
      sineGenerator.compute(yoTime.getDoubleValue());
      sineGenerator.getLinearData(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      basePointCommand.setInverseDynamics(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      basePointCommand.setControlMode(controllerCoreMode.getValue());
   }

   public void updateHandFeedbackCommands()
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
      handSpatialCommand.setControlBaseFrame(trajectory.getCurrentReferenceFrame());
      handSpatialCommand.setInverseDynamics(orientation, position, angularVelocity, linearVelocity, angularAcceleration, linearAcceleration);
      handSpatialCommand.setControlMode(controllerCoreMode.getValue());
   }

   public void updateHandTrajectory()
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
         handTargetOrientation.getFrameOrientationIncludingFrame(finalOrientation);
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
}
