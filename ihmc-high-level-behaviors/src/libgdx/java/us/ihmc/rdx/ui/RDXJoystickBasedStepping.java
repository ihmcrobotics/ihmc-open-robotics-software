package us.ihmc.rdx.ui;

import com.badlogic.gdx.controllers.Controller;
import com.badlogic.gdx.controllers.Controllers;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.*;
import imgui.internal.ImGui;
import imgui.type.ImDouble;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.vr.RDXVRController;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.DeadbandTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class RDXJoystickBasedStepping
{
   private final SteppingParameters steppingParameters;
   private RDXVRController leftVRController;
   private RDXVRController rightVRController;
   private boolean walkingModeActive = false;
   private double forwardJoystickValue = 0.0;
   private double lateralJoystickValue = 0.0;
   private double turningJoystickValue = 0.0;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble turningVelocity = new ImDouble();
   private final ImDouble forwardVelocity = new ImDouble();
   private final ImDouble lateralVelocity = new ImDouble();
   private final ImDouble swingHeight = new ImDouble();
   private final ImDouble swingDuration = new ImDouble();
   private final ImDouble transferDuration = new ImDouble();
   private final ImDouble maxStepLength = new ImDouble();
   private final ImDouble defaultStepWidth = new ImDouble();
   private final ImDouble minStepWidth = new ImDouble();
   private final ImDouble maxStepWidth = new ImDouble();
   private final ImDouble turnStepWidth = new ImDouble();
   private final ImDouble turnMaxAngleInward = new ImDouble();
   private final ImDouble turnMaxAngleOutward = new ImDouble();
   private ROS2Input<CapturabilityBasedStatus> capturabilityBasedStatusInput;
   private ROS2ControllerHelper controllerHelper;
   private ROS2SyncedRobotModel syncedRobot;
   private ROS2Publisher<ContinuousStepGeneratorInputMessage> stepGeneratorPublisher;

   private RDXFootstepPlanGraphic footstepPlanGraphic;
   private final SideDependentList<FramePose3D> lastSupportFootPoses = new SideDependentList<>(null, null);
   private final SideDependentList<Boolean> isFootInSupport = new SideDependentList<>(false, false);
   private final ConcurrentLinkedQueue<Runnable> queuedTasksToProcess = new ConcurrentLinkedQueue<>();
   private final AtomicReference<FootstepDataListMessage> footstepsToSendReference = new AtomicReference<>(null);
   private final AtomicBoolean isWalking = new AtomicBoolean(false);
   // Set to true initially because this class shouldn't be sending pause walking messages unless it's being used
   // This prevents the controller from getting pause walking messages while the operator is still starting up
   // but they have run the UI.
   private final AtomicBoolean hasSuccessfullyStoppedWalking = new AtomicBoolean(true);
   private boolean supportFootPosesInitialized = false;
   private boolean userNotClickingAnImGuiPanel;

   public RDXJoystickBasedStepping(DRCRobotModel robotModel)
   {
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      steppingParameters = walkingControllerParameters.getSteppingParameters();
      swingHeight.set(walkingControllerParameters.getSwingTrajectoryParameters().getMinSwingHeight());
      swingDuration.set(walkingControllerParameters.getDefaultSwingTime());
      transferDuration.set(walkingControllerParameters.getDefaultTransferTime());
//      maxStepLength.set(steppingParameters.getMaxStepLength());
      maxStepLength.set(0.3);
      defaultStepWidth.set(steppingParameters.getInPlaceWidth());
      minStepWidth.set(steppingParameters.getMinStepWidth());
//      maxStepWidth.set(steppingParameters.getMaxStepWidth());
      maxStepWidth.set(0.3);
      turnStepWidth.set(steppingParameters.getTurningStepWidth());
      turnMaxAngleInward.set(steppingParameters.getMaxAngleTurnInwards());
      turnMaxAngleOutward.set(steppingParameters.getMaxAngleTurnOutwards());
   }

   public void create(RDXBaseUI baseUI, ROS2ControllerHelper controllerHelper, ROS2SyncedRobotModel syncedRobot)
   {
      this.controllerHelper = controllerHelper;
      this.syncedRobot = syncedRobot;

      ROS2Topic<?> inputTopic = StepGeneratorAPIDefinition.getInputTopic(syncedRobot.getRobotModel().getSimpleRobotName()).withTypeName(ContinuousStepGeneratorInputMessage.class);
      stepGeneratorPublisher = (ROS2Publisher<ContinuousStepGeneratorInputMessage>) controllerHelper.getROS2Node().createPublisher(inputTopic);

      syncedRobot.addRobotConfigurationDataReceivedCallback(robotConfigurationData ->
      {
         RobotMotionStatus newStatus = RobotMotionStatus.fromByte(robotConfigurationData.getRobotMotionStatus());
         // We only want to verify that the last PauseWalking sent has been successfully executed once.
         // Considering that the user may use a separate app to get the robot to walk, we do not want to interfere with the other app.
         if (hasSuccessfullyStoppedWalking.get() || isWalking.get())
            return;
         if (newStatus == null)
            return;
         if (newStatus == RobotMotionStatus.STANDING)
            hasSuccessfullyStoppedWalking.set(true);
      });

      leftVRController = baseUI.getVRManager().getContext().getController(RobotSide.LEFT);
      rightVRController = baseUI.getVRManager().getContext().getController(RobotSide.RIGHT);
      baseUI.getVRManager().getContext().addVRInputProcessor(context ->
      {
         userNotClickingAnImGuiPanel = true;
         for (RobotSide side : RobotSide.values)
            userNotClickingAnImGuiPanel =  userNotClickingAnImGuiPanel && context.getController(side).getSelectedPick() == null;
      });
   }

   public void update(boolean enabled)
   {
      if (!enabled || !leftVRController.isConnected() || !rightVRController.isConnected())
      {
         // TODO send a stop walking message?
         return;
      }

      walkingModeActive = rightVRController.getClickTriggerActionData().bState();

      double deadband = 0.1;
      forwardJoystickValue = DeadbandTools.applyDeadband(deadband, leftVRController.getJoystickActionData().y());
      lateralJoystickValue = DeadbandTools.applyDeadband(deadband, -leftVRController.getJoystickActionData().x());
      turningJoystickValue = DeadbandTools.applyDeadband(deadband, -rightVRController.getJoystickActionData().x());

      double stepTime = swingDuration.get() + transferDuration.get();
      forwardVelocity.set((maxStepLength.get() / stepTime) * MathTools.clamp(forwardJoystickValue, 1.0));
      lateralVelocity.set((maxStepWidth.get() / stepTime) * MathTools.clamp(lateralJoystickValue, 1.0));
      turningVelocity.set(((turnMaxAngleOutward.get() - turnMaxAngleInward.get()) / stepTime) * MathTools.clamp(turningJoystickValue, 1.0));

      ContinuousStepGeneratorInputMessage continuousStepGeneratorInput = new ContinuousStepGeneratorInputMessage();
      continuousStepGeneratorInput.setWalk(walkingModeActive);
      continuousStepGeneratorInput.setForwardVelocity(forwardVelocity.get());
      continuousStepGeneratorInput.setLateralVelocity(lateralVelocity.get());
      continuousStepGeneratorInput.setTurnVelocity(turningVelocity.get());
      stepGeneratorPublisher.publish(continuousStepGeneratorInput);
   }

   public void renderImGuiWidgets()
   {
      ImGui.inputDouble(labels.get("Turning velocity"), turningVelocity);
      ImGui.inputDouble(labels.get("Forward velocity"), forwardVelocity);
      ImGui.inputDouble(labels.get("Lateral velocity"), lateralVelocity);
      ImGui.inputDouble(labels.get("Swing height"), swingHeight);
      ImGui.inputDouble(labels.get("Swing duration"), swingDuration);
      ImGui.inputDouble(labels.get("Transfer duration"), transferDuration);
      ImGui.inputDouble(labels.get("Max step length"), maxStepLength);
      ImGui.inputDouble(labels.get("Default step width"), defaultStepWidth);
      ImGui.inputDouble(labels.get("Min step width"), minStepWidth);
      ImGui.inputDouble(labels.get("Max step width"), maxStepWidth);
      ImGui.inputDouble(labels.get("Turn step width"), turnStepWidth);
      ImGui.inputDouble(labels.get("Turn max angle inward"), turnMaxAngleInward);
      ImGui.inputDouble(labels.get("Turn max angle outward"), turnMaxAngleOutward);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
//      footstepPlanGraphic.getRenderables(renderables, pool);
   }

   public void destroy()
   {
//      footstepPlanGraphic.destroy();
   }
}
