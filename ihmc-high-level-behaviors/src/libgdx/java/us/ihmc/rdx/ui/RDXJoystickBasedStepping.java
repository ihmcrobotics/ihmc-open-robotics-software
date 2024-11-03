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
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
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
   private final SegmentDependentList<RobotSide, ArrayList<Point2D>> controllerFootGroundContactPoints;
   private Controller currentController;
   private boolean currentControllerConnected;
   private RDXVRController leftVRController;
   private RDXVRController rightVRController;
   private boolean walkingModeActive = false;
   private double forwardJoystickValue = 0.0;
   private double lateralJoystickValue = 0.0;
   private double turningJoystickValue = 0.0;

   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory("FootstepPublisher"));
   private final ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator();
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
      controllerFootGroundContactPoints = robotModel.getContactPointParameters().getControllerFootGroundContactPoints();
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
      continuousStepGenerator.setNumberOfFootstepsToPlan(10);
      continuousStepGenerator.setDesiredTurningVelocityProvider(turningVelocity::get);
      continuousStepGenerator.setDesiredVelocityProvider(() -> new Vector2D(forwardVelocity.get(), lateralVelocity.get()));
      continuousStepGenerator.configureWith(walkingControllerParameters);
      continuousStepGenerator.addFootstepAdjustment(this::adjustFootstep);
      continuousStepGenerator.setFootstepMessenger(this::prepareFootsteps);
      continuousStepGenerator.setFootPoseProvider(robotSide -> lastSupportFootPoses.get(robotSide));
      continuousStepGenerator.addFootstepValidityIndicator(this::isStepSnappable);
      continuousStepGenerator.addFootstepValidityIndicator(this::isSafeDistanceFromObstacle);
      continuousStepGenerator.addFootstepValidityIndicator(this::isSafeStepHeight);
   }

   public void create(RDXBaseUI baseUI, ROS2ControllerHelper controllerHelper, ROS2SyncedRobotModel syncedRobot)
   {
      this.controllerHelper = controllerHelper;
      this.syncedRobot = syncedRobot;
      capturabilityBasedStatusInput = controllerHelper.subscribeToController(CapturabilityBasedStatus.class);
      controllerHelper.subscribeToControllerViaCallback(FootstepStatusMessage.class, footstepStatus ->
      {
         queuedTasksToProcess.add(() ->
         {
            continuousStepGenerator.consumeFootstepStatus(footstepStatus);

            if (footstepStatus.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
            {
               lastSupportFootPoses.put(RobotSide.fromByte(footstepStatus.getRobotSide()),
                                        new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                        footstepStatus.getActualFootPositionInWorld(),
                                                        footstepStatus.getActualFootOrientationInWorld()));
            }
         });
      });
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

      footstepPlanGraphic = new RDXFootstepPlanGraphic(controllerFootGroundContactPoints);
      footstepPlanGraphic.setColor(RobotSide.LEFT, new Color(0.8627451f, 0.078431375f, 0.23529412f, 1.0f)); // crimson
      footstepPlanGraphic.setColor(RobotSide.RIGHT, new Color(0.6039216f, 0.8039216f, 0.19607843f, 1.0f)); //yellowgreen

      executorService.scheduleAtFixedRate(this::sendFootsteps, 0, 500, TimeUnit.MILLISECONDS);

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
      while (!queuedTasksToProcess.isEmpty())
         queuedTasksToProcess.poll().run();

      currentController = Controllers.getCurrent();
      currentControllerConnected = currentController != null;

      if (enabled && (currentControllerConnected || (leftVRController.isConnected() && rightVRController.isConnected())))
      {
         if (currentControllerConnected)
         {
            walkingModeActive = currentController.getButton(currentController.getMapping().buttonR1);
            forwardJoystickValue = -currentController.getAxis(currentController.getMapping().axisLeftY);
            lateralJoystickValue = -currentController.getAxis(currentController.getMapping().axisLeftX);
            turningJoystickValue = -currentController.getAxis(currentController.getMapping().axisRightX);
         }

         if (rightVRController.isConnected())
         {
            walkingModeActive = rightVRController.getClickTriggerActionData().bState() && userNotClickingAnImGuiPanel;
            turningJoystickValue = -rightVRController.getJoystickActionData().x();
         }

         if (leftVRController.isConnected())
         {
            forwardJoystickValue = leftVRController.getJoystickActionData().y();
            lateralJoystickValue = -leftVRController.getJoystickActionData().x();
         }

         if (capturabilityBasedStatusInput.hasReceivedFirstMessage() && syncedRobot.getDataReceptionTimerSnapshot().isRunning(1.0))
         {
            CapturabilityBasedStatus capturabilityBasedStatus = capturabilityBasedStatusInput.getLatest();
            boolean isLeftFootInSupport = !capturabilityBasedStatus.getLeftFootSupportPolygon3d().isEmpty();
            boolean isRightFootInSupport = !capturabilityBasedStatus.getRightFootSupportPolygon3d().isEmpty();
            isFootInSupport.set(RobotSide.LEFT, isLeftFootInSupport);
            isFootInSupport.set(RobotSide.RIGHT, isRightFootInSupport);
            boolean isInDoubleSupport = isLeftFootInSupport && isRightFootInSupport;

            if (!supportFootPosesInitialized && isInDoubleSupport)
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  lastSupportFootPoses.put(robotSide, new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(robotSide)));
               }

               supportFootPosesInitialized = true;
            }

            if (supportFootPosesInitialized)
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  if (isFootInSupport.get(robotSide))
                  { // Touchdown may not have been made with the foot properly settled, so we update the support foot pose if its current pose is lower.
                     MovingReferenceFrame soleFrame = syncedRobot.getReferenceFrames().getSoleFrame(robotSide);
                     double currentHeight = soleFrame.getTransformToWorldFrame().getTranslationZ();
                     if (currentHeight < lastSupportFootPoses.get(robotSide).getZ())
                        lastSupportFootPoses.put(robotSide, new FramePose3D(soleFrame));
                  }
               }

               double stepTime = swingDuration.get() + transferDuration.get();
               double deadband = 0.1;
               forwardJoystickValue = DeadbandTools.applyDeadband(deadband, forwardJoystickValue);
               forwardVelocity.set((maxStepLength.get() / stepTime) * MathTools.clamp(forwardJoystickValue, 1.0));
               lateralJoystickValue = DeadbandTools.applyDeadband(deadband, lateralJoystickValue);
               lateralVelocity.set((maxStepWidth.get() / stepTime) * MathTools.clamp(lateralJoystickValue, 1.0));
               turningJoystickValue = DeadbandTools.applyDeadband(deadband, turningJoystickValue);
               // if (forwardVelocity.get() < -0.001) // kinda like it better without this
               //    turningJoystickValue = -turningJoystickValue;
               turningVelocity.set(((turnMaxAngleOutward.get() - turnMaxAngleInward.get()) / stepTime) * MathTools.clamp(turningJoystickValue, 1.0));

               if (walkingModeActive)
               {
                  isWalking.set(true);
                  continuousStepGenerator.startWalking();
                  hasSuccessfullyStoppedWalking.set(false);
               }
               else
               {
                  disableWalking();
                  footstepPlanGraphic.clear();
               }

               continuousStepGenerator.setFootstepTiming(swingDuration.get(), transferDuration.get());
               continuousStepGenerator.setStepTurningLimits(turnMaxAngleInward.get(), turnMaxAngleOutward.get());
               continuousStepGenerator.setStepWidths(defaultStepWidth.get(), minStepWidth.get(), maxStepWidth.get());
               continuousStepGenerator.setMaxStepLength(maxStepLength.get());
               continuousStepGenerator.update(Double.NaN);
            }
         }

         footstepPlanGraphic.update();
      }
   }

   private void disableWalking()
   {
      isWalking.set(false);
      footstepsToSendReference.set(null);
      continuousStepGenerator.stopWalking();
      sendPauseWalkingToController();
   }

   private void sendFootsteps()
   {
      FootstepDataListMessage footstepsToSend = footstepsToSendReference.getAndSet(null);
      if (footstepsToSend != null && isWalking.get())
      {
         controllerHelper.publishToController(footstepsToSend);
      }

      if (!isWalking.get())
      {
         // Only send pause request if we think the command has not been executed yet. This is to be more robust in case packets are dropped.
         if (!hasSuccessfullyStoppedWalking.get())
            sendPauseWalkingToController();
      }
   }

   private void sendPauseWalkingToController()
   {
      if (currentControllerConnected)
      {
         PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
         pauseWalkingMessage.setPause(true);
         controllerHelper.publishToController(pauseWalkingMessage);
      }
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
      if (currentControllerConnected)
      {
         for (int i = currentController.getMinButtonIndex(); i < currentController.getMaxButtonIndex(); i++)
         {
            ImGui.text("Button " + i + ": " + currentController.getButton(i));
         }
         for (int i = 0; i < currentController.getAxisCount(); i++)
         {
            ImGui.text("Axis " + i + ": " + currentController.getAxis(i));
         }
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      footstepPlanGraphic.getRenderables(renderables, pool);
   }

   private boolean adjustFootstep(FramePose3DReadOnly stanceFootPose, FramePose2DReadOnly footstepPose, RobotSide footSide, FootstepDataMessage adjustedFootstep)
   {
      FramePose3D adjustedBasedOnStanceFoot = new FramePose3D();
      adjustedBasedOnStanceFoot.getPosition().set(footstepPose.getPosition());
      adjustedBasedOnStanceFoot.setZ(stanceFootPose.getZ());
      adjustedBasedOnStanceFoot.getOrientation().set(footstepPose.getOrientation());
      
      adjustedFootstep.getLocation().set(adjustedBasedOnStanceFoot.getPosition());
      adjustedFootstep.getOrientation().set(adjustedBasedOnStanceFoot.getOrientation());
      return true;
//      return adjustedBasedOnStanceFoot;
   }

   private void prepareFootsteps(FootstepDataListMessage footstepDataListMessage)
   {
      ArrayList<MinimalFootstep> minimalFootsteps = new ArrayList<>();

      for (int i = 0; i < footstepDataListMessage.getFootstepDataList().size(); i++)
      {
         FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().get(i);
         footstepDataMessage.setSwingHeight(swingHeight.get());
         Pose3D pose = new Pose3D(footstepDataMessage.getLocation(), footstepDataMessage.getOrientation());
         minimalFootsteps.add(new MinimalFootstep(RobotSide.fromByte(footstepDataMessage.getRobotSide()), pose));
      }

      footstepPlanGraphic.generateMeshesAsync(minimalFootsteps);
      footstepsToSendReference.set(new FootstepDataListMessage(footstepDataListMessage));
   }

   private boolean isStepSnappable(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      return true;
   }

   private boolean isSafeDistanceFromObstacle(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      return true;
   }

   private boolean isSafeStepHeight(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      double heightChange = touchdownPose.getZ() - stancePose.getZ();
      return heightChange < steppingParameters.getMaxStepUp() && heightChange > -steppingParameters.getMaxStepDown();
   }

   public void destroy()
   {
      sendPauseWalkingToController();
      footstepPlanGraphic.destroy();
      executorService.shutdownNow();
   }
}
