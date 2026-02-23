package us.ihmc.avatar;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.YoContinuousStepGeneratorParameters;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator.calculateNextFootstepPose2D;

public class AvatarBipedalGaitGenerator
{
   private static final int NUMBER_OF_STEPS_TO_PLAN = 3;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final String variableNameSuffix = "MCGG";

   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames humanoidReferenceFrames;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final CommandInputManager walkingCommandInputManager;

   private final YoContinuousStepGeneratorParameters parameters = new YoContinuousStepGeneratorParameters(variableNameSuffix, registry);
   private final YoBoolean walkPrev = new YoBoolean("walkMCGSPrev", registry);
   private final YoBoolean walk = new YoBoolean("walkMCGS", registry);
   private final YoDouble desiredWalkingVelocityX = new YoDouble("desiredWalkingVelocityX", registry);
   private final YoDouble desiredWalkingVelocityY = new YoDouble("desiredWalkingVelocityY", registry);
   private final YoDouble desiredWalkingVelocityYaw = new YoDouble("desiredWalkingVelocityYaw", registry);

   private final YoEnum<RobotSide> currentSupportSide = new YoEnum<>("currentSupportSide", registry, RobotSide.class, false);
   private final YoFramePose3D currentSupportPose = new YoFramePose3D("currentSupportPose", ReferenceFrame.getWorldFrame(), registry);
   private final AtomicReference<WalkingStatusMessage> walkingStatusMessage = new AtomicReference<>();
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>();

   private final FootstepDataListCommand footstepDataListCommand = new FootstepDataListCommand();

   private final FramePose2D footstepPose2D = new FramePose2D();
   private final FramePose2D nextFootstepPose2D = new FramePose2D();
   private final FramePose3D nextFootstepPose3D = new FramePose3D();
   private final FramePose3D previousFootstepPose = new FramePose3D();
   private final FramePose3D nextFootstepPose3DViz = new FramePose3D();

   public AvatarBipedalGaitGenerator(CommandInputManager commandInputManager,
                                     StatusMessageOutputManager statusOutputManager,
                                     DRCRobotModel robotModel,
                                     FullHumanoidRobotModel fullRobotModel,
                                     HumanoidReferenceFrames humanoidReferenceFrames,
                                     CommandInputManager walkingCommandInputManager,
                                     StatusMessageOutputManager walkingOutputManager,
                                     YoRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;
      this.humanoidReferenceFrames = humanoidReferenceFrames;
      this.walkingCommandInputManager = walkingCommandInputManager;
      this.parameters.set(robotModel.getWalkingControllerParameters());

      walkingOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, walkingStatusMessage::set);
      walkingOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, footstepStatusMessage::set);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      boolean walk = this.walk.getValue();
      boolean walkPrev = this.walkPrev.getValue();
      this.walkPrev.set(walk);

      if (!walk)
      {
         footstepDataListCommand.clear();
         return;
      }

      FootstepStatusMessage footstepStatus = footstepStatusMessage.getAndSet(null);
      if (footstepStatus != null && footstepStatus.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
      {
         currentSupportSide.set(RobotSide.fromByte(footstepStatus.getRobotSide()).getOppositeSide());
      }
      if (footstepStatus != null && footstepStatus.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
      {
         currentSupportSide.set(RobotSide.fromByte(footstepStatus.getRobotSide()));
      }

      currentSupportPose.setFromReferenceFrame(humanoidReferenceFrames.getSoleFrame(currentSupportSide.getValue()));
      footstepDataListCommand.getFootsteps().clear();
      RobotSide swingSide = currentSupportSide.getValue().getOppositeSide();
      footstepPose2D.setIncludingFrame(currentSupportPose);

      double maxStepLengthForwards = parameters.getMaxStepLengthForwards();
      double maxStepLengthBackwards = parameters.getMaxStepLengthBackwards();
      double maxStepWidth = parameters.getMaxStepWidth();
      double minStepWidth = parameters.getMinStepWidth();
      double defaultStepWidth = parameters.getDefaultStepWidth();
      double turnMaxAngleInward = parameters.getTurnMaxAngleInward();
      double turnMaxAngleOutward = parameters.getTurnMaxAngleOutward();
      double stepTime = getStepTime();

      for (int i = 0; i < NUMBER_OF_STEPS_TO_PLAN; i++)
      {
         calculateNextFootstepPose2D(stepTime,
                                     desiredWalkingVelocityX.getValue(),
                                     desiredWalkingVelocityY.getValue(),
                                     desiredWalkingVelocityYaw.getValue(),
                                     swingSide,
                                     maxStepLengthForwards,
                                     maxStepLengthBackwards,
                                     maxStepWidth,
                                     defaultStepWidth,
                                     minStepWidth,
                                     turnMaxAngleInward,
                                     turnMaxAngleOutward,
                                     footstepPose2D,
                                     nextFootstepPose2D);

         nextFootstepPose3D.set(nextFootstepPose2D);
         FootstepDataCommand footstep = footstepDataListCommand.getFootsteps().add();

         footstep.getPosition().set(nextFootstepPose2D.getPosition());
         footstep.getOrientation().set(nextFootstepPose2D.getOrientation());
         footstep.setRobotSide(swingSide);
         snap(footstep);

         footstepPose2D.set(nextFootstepPose2D);
         swingSide = swingSide.getOppositeSide();
      }

      footstepDataListCommand.setDefaultSwingDuration(parameters.getSwingDuration());
      footstepDataListCommand.setDefaultTransferDuration(parameters.getTransferDuration());
      footstepDataListCommand.setFinalTransferDuration(parameters.getTransferDuration());
      footstepDataListCommand.setAreFootstepsAdjustable(parameters.getStepsAreAdjustable());
      footstepDataListCommand.setOffsetFootstepsHeightWithExecutionError(parameters.getAccountForGroundDrift());
      footstepDataListCommand.setOffsetFootstepsWithExecutionError(parameters.getShiftUpcomingStepsWithTouchdown());
      footstepDataListCommand.setExecutionMode(ExecutionMode.OVERRIDE);
      walkingCommandInputManager.submitCommand(footstepDataListCommand);
   }

   public FootstepDataListCommand getFootstepDataListCommand()
   {
      return footstepDataListCommand;
   }

   private double getStepTime()
   {
      return parameters.getSwingDuration() + parameters.getTransferDuration();
   }

   private void snap(FootstepDataCommand footstep)
   {
      FramePoint3D position = footstep.getPosition();
      position.setZ(currentSupportPose.getZ());
   }

   private boolean isWalking()
   {
      WalkingStatusMessage walkingStatusMessage = this.walkingStatusMessage.get();
      if (walkingStatusMessage == null)
         return false;
      WalkingStatus walkingStatus = WalkingStatus.fromByte(walkingStatusMessage.getWalkingStatus());
      return walkingStatus == WalkingStatus.STARTED || walkingStatus == WalkingStatus.RESUMED;
   }

   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }
}