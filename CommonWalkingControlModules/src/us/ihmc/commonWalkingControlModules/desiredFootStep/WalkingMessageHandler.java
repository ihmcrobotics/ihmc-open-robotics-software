package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.CommandArrayDeque;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AdjustFootstepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingControllerFailureStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.tools.io.printing.PrintTools;

public class WalkingMessageHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // TODO Need to find something better than an ArrayList.
   private final List<Footstep> upcomingFootsteps = new ArrayList<>();
   private final List<FootstepTiming> upcomingFootstepTimings = new ArrayList<>();

   private final BooleanYoVariable hasNewFootstepAdjustment = new BooleanYoVariable("hasNewFootstepAdjustement", registry);
   private final AdjustFootstepCommand requestedFootstepAdjustment = new AdjustFootstepCommand();
   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;
   private final SideDependentList<Footstep> footstepsAtCurrentLocation = new SideDependentList<>();
   private final SideDependentList<Footstep> lastDesiredFootsteps = new SideDependentList<>();

   private final SideDependentList<CommandArrayDeque<FootTrajectoryCommand>> upcomingFootTrajectoryCommandListForFlamingoStance = new SideDependentList<>();

   private final StatusMessageOutputManager statusOutputManager;

   private final IntegerYoVariable currentFootstepIndex = new IntegerYoVariable("currentFootstepIndex", registry);
   private final IntegerYoVariable currentNumberOfFootsteps = new IntegerYoVariable("currentNumberOfFootsteps", registry);
   private final BooleanYoVariable isWalkingPaused = new BooleanYoVariable("isWalkingPaused", registry);
   private final DoubleYoVariable defaultTransferTime = new DoubleYoVariable("defaultTransferTime", registry);
   private final DoubleYoVariable finalTransferTime = new DoubleYoVariable("finalTransferTime", registry);
   private final DoubleYoVariable defaultSwingTime = new DoubleYoVariable("defaultSwingTime", registry);
   private final DoubleYoVariable defaultInitialTransferTime = new DoubleYoVariable("defaultInitialTransferTime", registry);

   private final int numberOfFootstepsToVisualize = 4;
   @SuppressWarnings("unchecked")
   private final EnumYoVariable<RobotSide>[] upcomingFoostepSide = new EnumYoVariable[numberOfFootstepsToVisualize];

   private final FootstepListVisualizer footstepListVisualizer;

   public WalkingMessageHandler(double defaultTransferTime, double defaultSwingTime, double defaultInitialTransferTime, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         StatusMessageOutputManager statusOutputManager, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.contactableFeet = contactableFeet;
      this.statusOutputManager = statusOutputManager;

      this.defaultTransferTime.set(defaultTransferTime);
      this.finalTransferTime.set(defaultTransferTime);
      this.defaultSwingTime.set(defaultSwingTime);
      this.defaultInitialTransferTime.set(defaultInitialTransferTime);

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
         RigidBody endEffector = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame(sidePrefix + "FootstepAtCurrentLocation", worldFrame);
         Footstep footstepAtCurrentLocation = new Footstep(endEffector, robotSide, soleFrame, poseReferenceFrame);
         footstepsAtCurrentLocation.put(robotSide, footstepAtCurrentLocation);

         upcomingFootTrajectoryCommandListForFlamingoStance.put(robotSide, new CommandArrayDeque<>(FootTrajectoryCommand.class));
      }

      for (int i = 0; i < numberOfFootstepsToVisualize; i++)
         upcomingFoostepSide[i] = new EnumYoVariable<>("upcomingFoostepSide" + i, registry, RobotSide.class, true);

      footstepListVisualizer = new FootstepListVisualizer(contactableFeet, yoGraphicsListRegistry, registry);
      updateVisualization();

      parentRegistry.addChild(registry);
   }

   public void handleFootstepDataListCommand(FootstepDataListCommand command)
   {
      if (command.getNumberOfFootsteps() > 0)
      {
         switch(command.getExecutionMode())
         {
         case OVERRIDE:
            upcomingFootsteps.clear();
            upcomingFootstepTimings.clear();
            currentFootstepIndex.set(0);
            clearFootTrajectory();
            currentNumberOfFootsteps.set(command.getNumberOfFootsteps());
            break;
         case QUEUE:
            currentNumberOfFootsteps.add(command.getNumberOfFootsteps());
            break;
         default:
            PrintTools.warn(this, "Unknown " + ExecutionMode.class.getSimpleName() + " value: " + command.getExecutionMode() + ". Command ignored.");
            return;
         }
      }

      isWalkingPaused.set(false);
      double commandDefaultTransferTime = command.getDefaultTransferTime();
      double commandDefaultSwingTime = command.getDefaultSwingTime();
      if (!Double.isNaN(commandDefaultSwingTime) && commandDefaultSwingTime > 1.0e-2 && !Double.isNaN(commandDefaultTransferTime) && commandDefaultTransferTime >= 0.0)
      {
         defaultTransferTime.set(commandDefaultTransferTime);
         defaultSwingTime.set(commandDefaultSwingTime);
      }

      double commandFinalTransferTime = command.getFinalTransferTime();
      if (commandFinalTransferTime >= 0.0)
         finalTransferTime.set(commandFinalTransferTime);
      else
         finalTransferTime.set(defaultTransferTime.getDoubleValue());

      for (int i = 0; i < command.getNumberOfFootsteps(); i++)
      {
         Footstep newFootstep = createFootstep(command.getFootstep(i));
         upcomingFootsteps.add(newFootstep);
         FootstepTiming newFootstepTiming = createFootstepTiming(command.getFootstep(i));
         upcomingFootstepTimings.add(newFootstepTiming);
      }

      checkTimings(upcomingFootstepTimings);
      updateVisualization();
   }

   public void handleAdjustFootstepCommand(AdjustFootstepCommand command)
   {
      if (isWalkingPaused.getBooleanValue())
      {
         PrintTools.warn(this, "Received " + AdjustFootstepCommand.class.getSimpleName() + " but walking is currently paused. Command ignored.");
         requestedFootstepAdjustment.clear();
         hasNewFootstepAdjustment.set(false);
         return;
      }

      requestedFootstepAdjustment.set(command);
      hasNewFootstepAdjustment.set(true);
   }

   public void handlePauseWalkingCommand(PauseWalkingCommand command)
   {
      isWalkingPaused.set(command.isPauseRequested());
   }

   public void handleFootTrajectoryCommand(List<FootTrajectoryCommand> commands)
   {
      for (int i = 0; i < commands.size(); i++)
      {
         FootTrajectoryCommand command = commands.get(i);
         upcomingFootTrajectoryCommandListForFlamingoStance.get(command.getRobotSide()).addLast(command);
      }
   }

   public FootstepTiming peekTiming(int i)
   {
      if (i >= upcomingFootstepTimings.size())
         return null;
      else
         return upcomingFootstepTimings.get(i);
   }

   public Footstep peek(int i)
   {
      if (i >= upcomingFootsteps.size())
         return null;
      else
         return upcomingFootsteps.get(i);
   }

   public Footstep poll()
   {
      if (upcomingFootsteps.isEmpty())
         return null;
      else
      {
         updateVisualization();
         currentNumberOfFootsteps.decrement();
         currentFootstepIndex.increment();
         upcomingFootstepTimings.remove(0);
         return upcomingFootsteps.remove(0);
      }
   }

   public FootTrajectoryCommand pollFootTrajectoryForFlamingoStance(RobotSide swingSide)
   {
      return upcomingFootTrajectoryCommandListForFlamingoStance.get(swingSide).poll();
   }

   public boolean pollRequestedFootstepAdjustment(Footstep footstepToAdjust)
   {
      if (!hasNewFootstepAdjustment.getBooleanValue())
         return false;

      if (footstepToAdjust.getRobotSide() != requestedFootstepAdjustment.getRobotSide())
      {
         PrintTools.warn(this, "RobotSide does not match: side of footstep to be adjusted: " + footstepToAdjust.getRobotSide() + ", side of adjusted footstep: " + requestedFootstepAdjustment.getRobotSide());
         hasNewFootstepAdjustment.set(false);
         requestedFootstepAdjustment.clear();
         return false;
      }

      Point3d adjustedPosition = requestedFootstepAdjustment.getPosition();
      Quat4d adjustedOrientation = requestedFootstepAdjustment.getOrientation();

      switch (requestedFootstepAdjustment.getOrigin())
      {
      case AT_ANKLE_FRAME:
         footstepToAdjust.setPose(adjustedPosition, adjustedOrientation);
         break;
      case AT_SOLE_FRAME:
         footstepToAdjust.setSolePose(adjustedPosition, adjustedOrientation);
         break;
      default:
         throw new RuntimeException("Should not get there.");
      }

      if (!requestedFootstepAdjustment.getPredictedContactPoints().isEmpty())
      {
         List<Point2d> contactPoints = new ArrayList<>();
         for (int i = 0; i < footstepToAdjust.getPredictedContactPoints().size(); i++)
            contactPoints.add(footstepToAdjust.getPredictedContactPoints().get(i));
         footstepToAdjust.setPredictedContactPointsFromPoint2ds(contactPoints);
      }

      hasNewFootstepAdjustment.set(false);
      requestedFootstepAdjustment.clear();

      return true;
   }

   public void insertNextFootstep(Footstep newNextFootstep)
   {
      if (newNextFootstep != null)
         upcomingFootsteps.add(0, newNextFootstep);
   }

   public boolean hasUpcomingFootsteps()
   {
      return !upcomingFootsteps.isEmpty() && !isWalkingPaused.getBooleanValue();
   }

   public boolean hasRequestedFootstepAdjustment()
   {
      if (isWalkingPaused.getBooleanValue())
      {
         hasNewFootstepAdjustment.set(false);
         requestedFootstepAdjustment.clear();
      }
      return hasNewFootstepAdjustment.getBooleanValue();
   }

   public boolean isNextFootstepFor(RobotSide swingSide)
   {
      if (!hasUpcomingFootsteps())
         return false;
      else
         return peek(0).getRobotSide() == swingSide;
   }

   public boolean hasFootTrajectoryForFlamingoStance()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (hasFootTrajectoryForFlamingoStance(robotSide))
            return true;
      }
      return false;
   }

   public boolean hasFootTrajectoryForFlamingoStance(RobotSide swingSide)
   {
      return !upcomingFootTrajectoryCommandListForFlamingoStance.get(swingSide).isEmpty();
   }

   public boolean isWalkingPaused()
   {
      return isWalkingPaused.getBooleanValue();
   }

   public void clearFootTrajectory(RobotSide robotSide)
   {
      upcomingFootTrajectoryCommandListForFlamingoStance.get(robotSide).clear();
   }

   public void clearFootTrajectory()
   {
      for (RobotSide robotSide : RobotSide.values)
         clearFootTrajectory(robotSide);
   }

   public void clearFootsteps()
   {
      upcomingFootsteps.clear();
      upcomingFootstepTimings.clear();
      currentNumberOfFootsteps.set(0);
      currentFootstepIndex.set(0);
      updateVisualization();
   }

   private final Point3d desiredFootPositionInWorld = new Point3d();
   private final Quat4d desiredFootOrientationInWorld = new Quat4d();
   private final Point3d actualFootPositionInWorld = new Point3d();
   private final Quat4d actualFootOrientationInWorld = new Quat4d();
   private final TextToSpeechPacket reusableSpeechPacket = new TextToSpeechPacket();
   private final WalkingControllerFailureStatusMessage failureStatusMessage = new WalkingControllerFailureStatusMessage();

   public void reportFootstepStarted(RobotSide robotSide, FramePose desiredFootPoseInWorld, FramePose actualFootPoseInWorld)
   {
      desiredFootPoseInWorld.getPose(desiredFootPositionInWorld, desiredFootOrientationInWorld);
      actualFootPoseInWorld.getPose(actualFootPositionInWorld, actualFootOrientationInWorld);
      statusOutputManager.reportStatusMessage(new FootstepStatus(FootstepStatus.Status.STARTED, currentFootstepIndex.getIntegerValue(),
            desiredFootPositionInWorld, desiredFootOrientationInWorld,
            actualFootPositionInWorld, actualFootOrientationInWorld, robotSide));
   }

   public void reportFootstepCompleted(RobotSide robotSide, FramePose actualFootPoseInWorld)
   {
      actualFootPoseInWorld.getPose(actualFootPositionInWorld, actualFootOrientationInWorld);
      statusOutputManager.reportStatusMessage(new FootstepStatus(FootstepStatus.Status.COMPLETED, currentFootstepIndex.getIntegerValue(),
            actualFootPositionInWorld, actualFootOrientationInWorld, robotSide));
//      reusableSpeechPacket.setTextToSpeak(TextToSpeechPacket.FOOTSTEP_COMPLETED);
//      statusOutputManager.reportStatusMessage(reusableSpeechPacket);
   }

   public void reportWalkingStarted()
   {
      WalkingStatusMessage walkingStatusMessage = new WalkingStatusMessage();
      walkingStatusMessage.setWalkingStatus(WalkingStatusMessage.Status.STARTED);
      statusOutputManager.reportStatusMessage(walkingStatusMessage);
      reusableSpeechPacket.setTextToSpeak(TextToSpeechPacket.WALKING);
      statusOutputManager.reportStatusMessage(reusableSpeechPacket);
   }

   public void reportWalkingComplete()
   {
      WalkingStatusMessage walkingStatusMessage = new WalkingStatusMessage();
      walkingStatusMessage.setWalkingStatus(WalkingStatusMessage.Status.COMPLETED);
      statusOutputManager.reportStatusMessage(walkingStatusMessage);
//      reusableSpeechPacket.setTextToSpeak(TextToSpeechPacket.FINISHED_WALKING);
//      statusOutputManager.reportStatusMessage(reusableSpeechPacket);
   }

   public void reportWalkingAbortRequested()
   {
      WalkingStatusMessage walkingStatusMessage = new WalkingStatusMessage();
      walkingStatusMessage.setWalkingStatus(WalkingStatusMessage.Status.ABORT_REQUESTED);
      statusOutputManager.reportStatusMessage(walkingStatusMessage);
//      reusableSpeechPacket.setTextToSpeak(TextToSpeechPacket.WALKING_ABORTED);
//      statusOutputManager.reportStatusMessage(reusableSpeechPacket);
   }

   public void reportControllerFailure(FrameVector2d fallingDirection)
   {
      fallingDirection.changeFrame(worldFrame);
      failureStatusMessage.setFallingDirection(fallingDirection);
      statusOutputManager.reportStatusMessage(failureStatusMessage);
   }

   public void registerCompletedDesiredFootstep(Footstep completedFesiredFootstep)
   {
      lastDesiredFootsteps.put(completedFesiredFootstep.getRobotSide(), completedFesiredFootstep);
   }

   public Footstep getLastDesiredFootstep(RobotSide footstepSide)
   {
      return lastDesiredFootsteps.get(footstepSide);
   }

   private final FramePose tempPose = new FramePose();

   public Footstep getFootstepAtCurrentLocation(RobotSide robotSide)
   {
      tempPose.setToZero(contactableFeet.get(robotSide).getFrameAfterParentJoint());
      tempPose.changeFrame(worldFrame);
      Footstep footstep = footstepsAtCurrentLocation.get(robotSide);
      footstep.setPose(tempPose);
      return footstep;
   }

   public void setDefaultTransferTime(double transferTime)
   {
      this.defaultTransferTime.set(transferTime);
   }

   public void setDefaultSwingTime(double swingTime)
   {
      this.defaultSwingTime.set(swingTime);
   }

   public double getDefaultTransferTime()
   {
      return defaultTransferTime.getDoubleValue();
   }

   public double getNextTransferTime()
   {
      if (upcomingFootstepTimings.isEmpty())
         return Double.NaN;
      return upcomingFootstepTimings.get(0).getTransferTime();
   }

   public double getDefaultSwingTime()
   {
      return defaultSwingTime.getDoubleValue();
   }

   public double getNextSwingTime()
   {
      if (upcomingFootstepTimings.isEmpty())
         return Double.NaN;
      return upcomingFootstepTimings.get(0).getSwingTime();
   }

   public double getFinalTransferTime()
   {
      return finalTransferTime.getDoubleValue();
   }

   public double getDefaultStepTime()
   {
      return defaultTransferTime.getDoubleValue() + defaultSwingTime.getDoubleValue();
   }

   public double getNextStepTime()
   {
      if (upcomingFootstepTimings.isEmpty())
         return Double.NaN;
      return upcomingFootstepTimings.get(0).getStepTime();
   }

   public int getCurrentNumberOfFootsteps()
   {
      return currentNumberOfFootsteps.getIntegerValue();
   }

   private void updateVisualization()
   {
      for (int i = 0; i < upcomingFootsteps.size(); i++)
      {
         if (i < numberOfFootstepsToVisualize)
            upcomingFoostepSide[i].set(upcomingFootsteps.get(i).getRobotSide());
      }

      for (int i = upcomingFootsteps.size(); i < numberOfFootstepsToVisualize; i++)
      {
         upcomingFoostepSide[i].set(null);
      }

      footstepListVisualizer.update(upcomingFootsteps);
   }

   public void updateVisualizationAfterFootstepAdjustement(Footstep adjustedFootstep)
   {
      footstepListVisualizer.updateFirstFootstep(adjustedFootstep);
   }

   public TransferToAndNextFootstepsData createTransferToAndNextFootstepDataForDoubleSupport(RobotSide transferToSide)
   {
      Footstep transferFromFootstep = getFootstepAtCurrentLocation(transferToSide.getOppositeSide());
      Footstep transferToFootstep = getFootstepAtCurrentLocation(transferToSide);

      Footstep nextFootstep;

      nextFootstep = peek(0);

      TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();
      transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
      transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);
      transferToAndNextFootstepsData.setTransferToSide(transferToSide);
      transferToAndNextFootstepsData.setNextFootstep(nextFootstep);

      return transferToAndNextFootstepsData;
   }

   public TransferToAndNextFootstepsData createTransferToAndNextFootstepDataForSingleSupport(Footstep transferToFootstep, RobotSide swingSide)
   {
      TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();

      Footstep transferFromFootstep = getFootstepAtCurrentLocation(swingSide.getOppositeSide());

      transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
      transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);

      transferToAndNextFootstepsData.setTransferToSide(swingSide);
      transferToAndNextFootstepsData.setNextFootstep(peek(0));

      return transferToAndNextFootstepsData;
   }

   private Footstep createFootstep(FootstepDataCommand footstepData)
   {
      FramePose footstepPose = new FramePose(worldFrame, footstepData.getPosition(), footstepData.getOrientation());
      PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);

      List<Point2d> contactPoints;
      if (footstepData.getPredictedContactPoints().isEmpty())
         contactPoints = null;
      else
      {
         contactPoints = new ArrayList<>();
         for (int i = 0; i < footstepData.getPredictedContactPoints().size(); i++)
            contactPoints.add(footstepData.getPredictedContactPoints().get(i));
      }

      RobotSide robotSide = footstepData.getRobotSide();
      TrajectoryType trajectoryType = footstepData.getTrajectoryType();

      ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
      ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
      RigidBody rigidBody = contactableFoot.getRigidBody();

      Footstep footstep = new Footstep(rigidBody, robotSide, soleFrame, footstepPoseFrame, true, contactPoints);
      if (trajectoryType == TrajectoryType.CUSTOM)
      {
         if (footstepData.getTrajectoryWaypoints() == null)
         {
            PrintTools.warn("Can not request custom trajectory without specifying waypoints. Using default trajectory.");
            trajectoryType = TrajectoryType.DEFAULT;
         }
         else
         {
            RecyclingArrayList<Point3d> trajectoryWaypoints = footstepData.getTrajectoryWaypoints();
            footstep.setSwingWaypoints(trajectoryWaypoints);
         }
      }

      footstep.setTrajectoryType(trajectoryType);
      footstep.setSwingHeight(footstepData.getSwingHeight());
      switch (footstepData.getOrigin())
      {
      case AT_ANKLE_FRAME:
         break;
      case AT_SOLE_FRAME:
         footstep.setSolePose(footstepPose);
         break;
      default:
         throw new RuntimeException("Should not get there.");
      }
      return footstep;
   }

   private FootstepTiming createFootstepTiming(FootstepDataCommand footstep)
   {
      FootstepTiming timing = new FootstepTiming();
      if (footstep.hasTimings())
         timing.setTimings(timing.getSwingTime(), timing.getTransferTime());
      else
      {
         if (upcomingFootstepTimings.isEmpty())
            timing.setTimings(defaultSwingTime.getDoubleValue(), defaultInitialTransferTime.getDoubleValue());
         else
            timing.setTimings(defaultSwingTime.getDoubleValue(), defaultTransferTime.getDoubleValue());
      }
      if (footstep.hasAbsoluteTime())
         timing.setAbsoluteTime(timing.getSwingStartTime());
      return timing;
   }

   private void checkTimings(List<FootstepTiming> upcomingFootstepTimings)
   {
      if (upcomingFootstepTimings.isEmpty())
         return;

      boolean timingsValid = upcomingFootstepTimings.get(0).hasAbsoluteTime();
      boolean atLeastOneFootstepHadTiming = upcomingFootstepTimings.get(0).hasAbsoluteTime();

      double lastTime = upcomingFootstepTimings.get(0).getSwingStartTime();
      timingsValid = timingsValid && lastTime > 0.0;
      for (int footstepIdx = 1; footstepIdx < upcomingFootstepTimings.size(); footstepIdx++)
      {
         FootstepTiming footstep = upcomingFootstepTimings.get(footstepIdx);
         boolean timeIncreasing = footstep.getSwingStartTime() > lastTime;
         timingsValid = timingsValid && footstep.hasAbsoluteTime() && timeIncreasing;
         atLeastOneFootstepHadTiming = atLeastOneFootstepHadTiming || footstep.hasAbsoluteTime();

         lastTime = footstep.getSwingStartTime();
         if (!timingsValid)
            break;
      }

      if (atLeastOneFootstepHadTiming && !timingsValid)
      {
         PrintTools.warn("Recieved footstep data with invalid timings. Using swing and transfer times instead.");
         for (int footstepIdx = 1; footstepIdx < upcomingFootstepTimings.size(); footstepIdx++)
            upcomingFootstepTimings.get(footstepIdx).removeAbsoluteTime();
      }
   }
}
