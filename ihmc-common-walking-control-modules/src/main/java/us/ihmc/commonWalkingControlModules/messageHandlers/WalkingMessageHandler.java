package us.ihmc.commonWalkingControlModules.messageHandlers;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.humanoidRobotics.communication.packets.momentum.TrajectoryPoint3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PlanOffsetStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingControllerFailureStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.lists.RecyclingArrayDeque;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

public class WalkingMessageHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final int maxNumberOfFootsteps = 100;
   private final RecyclingArrayList<Footstep> upcomingFootsteps = new RecyclingArrayList<>(maxNumberOfFootsteps, Footstep.class);
   private final RecyclingArrayList<FootstepTiming> upcomingFootstepTimings = new RecyclingArrayList<>(maxNumberOfFootsteps, FootstepTiming.class);

   private final YoBoolean hasNewFootstepAdjustment = new YoBoolean("hasNewFootstepAdjustement", registry);
   private final AdjustFootstepCommand requestedFootstepAdjustment = new AdjustFootstepCommand();
   private final SideDependentList<Footstep> footstepsAtCurrentLocation = new SideDependentList<>();
   private final SideDependentList<Footstep> lastDesiredFootsteps = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();

   private final SideDependentList<RecyclingArrayDeque<FootTrajectoryCommand>> upcomingFootTrajectoryCommandListForFlamingoStance = new SideDependentList<>();

   private final StatusMessageOutputManager statusOutputManager;

   private final YoInteger currentFootstepIndex = new YoInteger("currentFootstepIndex", registry);
   private final YoInteger currentNumberOfFootsteps = new YoInteger("currentNumberOfFootsteps", registry);
   private final YoBoolean isWalkingPaused = new YoBoolean("isWalkingPaused", registry);
   private final YoDouble defaultTransferTime = new YoDouble("defaultTransferTime", registry);
   private final YoDouble finalTransferTime = new YoDouble("finalTransferTime", registry);
   private final YoDouble defaultSwingTime = new YoDouble("defaultSwingTime", registry);
   private final YoDouble defaultTouchdownTime = new YoDouble("defaultTouchdownTime", registry);
   private final YoDouble defaultInitialTransferTime = new YoDouble("defaultInitialTransferTime", registry);

   private final YoDouble defaultFinalTransferTime = new YoDouble("defaultFinalTransferTime", registry);
   private final YoLong lastCommandID = new YoLong("lastFootStepDataListCommandID", registry);

   private final YoBoolean isWalking = new YoBoolean("isWalking", registry);

   private final int numberOfFootstepsToVisualize = 4;
   @SuppressWarnings("unchecked")
   private final YoEnum<RobotSide>[] upcomingFoostepSide = new YoEnum[numberOfFootstepsToVisualize];

   private final FootstepListVisualizer footstepListVisualizer;

   private final YoDouble yoTime;
   private final YoDouble footstepDataListReceivedTime = new YoDouble("footstepDataListReceivedTime", registry);
   private final YoDouble timeElapsedWhenFootstepExecuted = new YoDouble("timeElapsedWhenFootstepExecuted", registry);
   private final YoBoolean executingFootstep = new YoBoolean("ExecutingFootstep", registry);
   private final FootstepTiming lastTimingExecuted = new FootstepTiming();

   private final MomentumTrajectoryHandler momentumTrajectoryHandler;
   private final CenterOfMassTrajectoryHandler comTrajectoryHandler;
   private final PlanarRegionsListHandler planarRegionsListHandler;

   private final YoBoolean offsettingPlanWithFootstepError = new YoBoolean("offsettingPlanWithFootstepError", registry);
   private final FrameVector3D planOffsetInWorld = new FrameVector3D(ReferenceFrame.getWorldFrame());

   public WalkingMessageHandler(double defaultTransferTime, double defaultSwingTime, double defaultTouchdownTime, double defaultInitialTransferTime, double defaultFinalTransferTime, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         StatusMessageOutputManager statusOutputManager, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this(defaultTransferTime, defaultSwingTime, defaultTouchdownTime, defaultInitialTransferTime, defaultFinalTransferTime, contactableFeet, statusOutputManager, null, yoGraphicsListRegistry, parentRegistry);
   }

   public WalkingMessageHandler(double defaultTransferTime, double defaultSwingTime, double defaultTouchdownTime, double defaultInitialTransferTime, double defaultFinalTransferTime, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         StatusMessageOutputManager statusOutputManager, YoDouble yoTime, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.statusOutputManager = statusOutputManager;

      upcomingFootsteps.clear();
      upcomingFootstepTimings.clear();

      this.yoTime = yoTime;
      footstepDataListReceivedTime.setToNaN();

      this.defaultTransferTime.set(defaultTransferTime);
      this.defaultSwingTime.set(defaultSwingTime);
      this.defaultTouchdownTime.set(defaultTouchdownTime);
      this.defaultInitialTransferTime.set(defaultInitialTransferTime);
      this.defaultFinalTransferTime.set(defaultFinalTransferTime);
      this.finalTransferTime.set(defaultFinalTransferTime);

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
         Footstep footstepAtCurrentLocation = new Footstep(robotSide);
         footstepsAtCurrentLocation.put(robotSide, footstepAtCurrentLocation);
         soleFrames.put(robotSide, contactableFoot.getSoleFrame());

         upcomingFootTrajectoryCommandListForFlamingoStance.put(robotSide, new RecyclingArrayDeque<>(FootTrajectoryCommand.class));
      }

      for (int i = 0; i < numberOfFootstepsToVisualize; i++)
         upcomingFoostepSide[i] = new YoEnum<>("upcomingFoostepSide" + i, registry, RobotSide.class, true);

      footstepListVisualizer = new FootstepListVisualizer(contactableFeet, yoGraphicsListRegistry, registry);
      updateVisualization();

      momentumTrajectoryHandler = new MomentumTrajectoryHandler(yoTime);
      comTrajectoryHandler = new CenterOfMassTrajectoryHandler(yoTime);
      planarRegionsListHandler = new PlanarRegionsListHandler(statusOutputManager, registry);

      parentRegistry.addChild(registry);
   }

   public void handleFootstepDataListCommand(FootstepDataListCommand command)
   {
      offsettingPlanWithFootstepError.set(command.isOffsetFootstepsWithExecutionError());
      if (!offsettingPlanWithFootstepError.getBooleanValue())
      {
         planOffsetInWorld.setToZero(ReferenceFrame.getWorldFrame());
      }

      if (command.getNumberOfFootsteps() > 0)
      {
         switch(command.getExecutionMode())
         {
         case OVERRIDE:
            clearFootsteps();
            clearFootTrajectory();
            if (yoTime != null)
            {
               footstepDataListReceivedTime.set(yoTime.getDoubleValue());
            }
            break;
         case QUEUE:
            if (currentNumberOfFootsteps.getIntegerValue() < 1 && !executingFootstep.getBooleanValue())
            {
               //if we queued a command and the previous already finished, just continue as if everything is normal
               if(command.getPreviousCommandId() == lastCommandID.getValue())
               {
                  break;
               }
               else
               {
                  PrintTools.warn("Can not queue footsteps if no footsteps are present. Send an override message instead. Command ignored.");
               }
               return;
            }
            break;
         default:
            PrintTools.warn(this, "Unknown " + ExecutionMode.class.getSimpleName() + " value: " + command.getExecutionMode() + ". Command ignored.");
            return;
         }
      }

      if (command.getNumberOfFootsteps() + currentNumberOfFootsteps.getIntegerValue() > maxNumberOfFootsteps)
      {
         PrintTools.warn("Can not exceed " + maxNumberOfFootsteps + " footsteps stopping execution.");
         clearFootsteps();
         return;
      }

      lastCommandID.set(command.getCommandId());
      isWalkingPaused.set(false);
      double commandDefaultTransferTime = command.getDefaultTransferDuration();
      double commandDefaultSwingTime = command.getDefaultSwingDuration();
      if (!Double.isNaN(commandDefaultSwingTime) && commandDefaultSwingTime > 1.0e-2 && !Double.isNaN(commandDefaultTransferTime) && commandDefaultTransferTime >= 0.0)
      {
         defaultTransferTime.set(commandDefaultTransferTime);
         defaultSwingTime.set(commandDefaultSwingTime);
      }

      double commandFinalTransferTime = command.getFinalTransferDuration();

      if (commandFinalTransferTime > 0.0)
         finalTransferTime.set(commandFinalTransferTime);
      else
         finalTransferTime.set(defaultFinalTransferTime.getDoubleValue());

      boolean trustHeightOfFootsteps = command.isTrustHeightOfFootsteps();

      for (int i = 0; i < command.getNumberOfFootsteps(); i++)
      {
         setFootstepTiming(command.getFootstep(i), command.getExecutionTiming(), upcomingFootstepTimings.add(), command.getExecutionMode());
         setFootstep(command.getFootstep(i), trustHeightOfFootsteps, upcomingFootsteps.add());
         currentNumberOfFootsteps.increment();
      }

      if (!checkTimings(upcomingFootstepTimings))
         clearFootsteps();
      updateTransferTimes(upcomingFootstepTimings);

      updateVisualization();
   }

   public void handlePlanarRegionsListCommand(PlanarRegionsListCommand planarRegionsListCommand)
   {
      planarRegionsListHandler.handlePlanarRegionsListCommand(planarRegionsListCommand);
   }

   public PlanarRegionsListHandler getPlanarRegionsListHandler()
   {
      return planarRegionsListHandler;
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

   public void handleMomentumTrajectoryCommand(MomentumTrajectoryCommand command)
   {
      momentumTrajectoryHandler.handleMomentumTrajectory(command);
   }

   /**
    * This method will pack the angular momentum trajectory for planning the ICP trajectory. The parameters {@code startTime} and {@code endTime} refer
    * to absolute controller time. To get the angular momentum trajectory from the current time to 1.0 seconds in the future the start time must
    * be the value of yoTime and the end time must be the value of yoTime + 1.0. The {@code numberOfPoints} parameter defines in how many points the
    * trajectory will be sampled. The packed trajectory will include the end points of the interval, therefore, the number of points must be equal
    * or grater then two. If the interval of interest is not available the trajectory to pack will be empty. The times of the packed trajectory points
    * will be relative to the start time of the interval.
    *
    * @param startTime is the controller time for the start of the interval for which the trajectory is packed
    * @param endTime is the controller time for the end of the interval for which the trajectory is packed
    * @param numberOfPoints the number of sampling points of the trajectory
    * @param trajectoryToPack the trajectory will be packed in here
    */
   public void getAngularMomentumTrajectory(double startTime, double endTime, int numberOfPoints, RecyclingArrayList<TrajectoryPoint3D> trajectoryToPack)
   {
      momentumTrajectoryHandler.getAngularMomentumTrajectory(startTime, endTime, numberOfPoints, trajectoryToPack);
   }

   public CenterOfMassTrajectoryHandler getComTrajectoryHandler()
   {
      return comTrajectoryHandler;
   }

   public void handleComTrajectoryCommand(CenterOfMassTrajectoryCommand command)
   {
      comTrajectoryHandler.handleComTrajectory(command);
   }

   /**
    * This method will set the provided timing to the timing of the {@code i}'th upcoming
    * footstep. If there is less then {@code i} upcoming steps this method will throw a
    * {@link RuntimeException}. To check how many footsteps are upcoming use
    * {@link #getCurrentNumberOfFootsteps()}.
    *
    * @param i is the index of the upcoming footstep timing that will be packed.
    * @param timingToPack will be set to the timing of footstep i in the list of upcoming steps.
    */
   public void peekTiming(int i, FootstepTiming timingToPack)
   {
      if (i >= upcomingFootstepTimings.size())
      {
         throw new RuntimeException("Can not get timing " + i + " since there are only " + upcomingFootstepTimings.size() + " upcoming timings.");
      }
      timingToPack.set(upcomingFootstepTimings.get(i));
   }

   /**
    * This method will set the provided footstep to the {@code i}'th upcoming
    * footstep. If there is less then {@code i} upcoming steps this method will throw a
    * {@link RuntimeException}. To check how many footsteps are upcoming use
    * {@link #getCurrentNumberOfFootsteps()}.
    *
    * @param i is the index of the upcoming footstep that will be packed.
    * @param footstepToPack will be set to the footstep i in the list of upcoming steps.
    */
   public void peekFootstep(int i, Footstep footstepToPack)
   {
      if (i >= upcomingFootsteps.size())
      {
         throw new RuntimeException("Can not get footstep " + i + " since there are only " + upcomingFootsteps.size() + " upcoming steps.");
      }
      footstepToPack.set(upcomingFootsteps.get(i));
   }

   /**
    * This method will pack the provided footstep and timing with the next upcoming
    * step. It will then remove that footstep from the list of upcoming footsteps.
    * Use this method if you wish to remove the next upcoming step because the execution
    * has started. If there is no upcoming step this method will throw a {@link RuntimeException}.
    *
    * @param footstepToPack will be set to the next footstep in the list of upcoming steps.
    * @param timingToPack will be set to the next footsteps timing in the list of upcoming steps.
    */
   public void poll(Footstep footstepToPack, FootstepTiming timingToPack)
   {
      if (upcomingFootsteps.isEmpty())
      {
         throw new RuntimeException("Can not poll footstep since there are no upcoming steps.");
      }

      footstepToPack.set(upcomingFootsteps.get(0));
      timingToPack.set(upcomingFootstepTimings.get(0));
      lastTimingExecuted.set(upcomingFootstepTimings.get(0));

      updateVisualization();
      currentNumberOfFootsteps.decrement();
      currentFootstepIndex.increment();

      upcomingFootstepTimings.remove(0);
      upcomingFootsteps.remove(0);
   }

   /**
    * This method can be used if the timing of a upcoming footstep needs to be adjusted. It
    * will throw a {@link RuntimeException} if there is less upcoming footstep timings then
    * {@code stepIndex}.
    *
    * @param stepIndex is the index of the timing that will be adjusted in the list of upcoming steps.
    * @param newSwingDuration is the new swing duration for the adjusted timing
    * @param newTransferDuration is the new transfer duration for the adjusted timing.
    */
   public void adjustTimings(int stepIndex, double newSwingDuration, double newTouchdownDuration, double newTransferDuration)
   {
      if (upcomingFootstepTimings.size() <= stepIndex)
      {
         throw new RuntimeException("Can not adjust timing of upciming step " + stepIndex + ", only have " + upcomingFootstepTimings.size() + " upcoming steps.");
      }
      upcomingFootstepTimings.get(stepIndex).setTimings(newSwingDuration, newTouchdownDuration, newTransferDuration);
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

      FramePoint3D adjustedPosition = requestedFootstepAdjustment.getPosition();
      FrameQuaternion adjustedOrientation = requestedFootstepAdjustment.getOrientation();
      footstepToAdjust.setPose(adjustedPosition, adjustedOrientation);

      if (!requestedFootstepAdjustment.getPredictedContactPoints().isEmpty())
      {
         List<Point2D> contactPoints = new ArrayList<>();
         for (int i = 0; i < footstepToAdjust.getPredictedContactPoints().size(); i++)
            contactPoints.add(footstepToAdjust.getPredictedContactPoints().get(i));
         footstepToAdjust.setPredictedContactPoints(contactPoints);
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
   
   public boolean isNextFootstepUsingAbsoluteTiming()
   {
      if (!hasUpcomingFootsteps())
      {
         return false;
      }

      return upcomingFootstepTimings.get(0).hasAbsoluteTime();
   }

   public boolean isNextFootstepFor(RobotSide swingSide)
   {
      if (!hasUpcomingFootsteps())
      {
         return false;
      }

      return upcomingFootsteps.get(0).getRobotSide() == swingSide;
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

   private final Point3D desiredFootPositionInWorld = new Point3D();
   private final Quaternion desiredFootOrientationInWorld = new Quaternion();
   private final Point3D actualFootPositionInWorld = new Point3D();
   private final Quaternion actualFootOrientationInWorld = new Quaternion();
   private final TextToSpeechPacket reusableSpeechPacket = new TextToSpeechPacket();
   private final WalkingControllerFailureStatusMessage failureStatusMessage = new WalkingControllerFailureStatusMessage();
   private final FootstepStatus footstepStatus = new FootstepStatus();

   public void reportFootstepStarted(RobotSide robotSide, FramePose3D desiredFootPoseInWorld, FramePose3D actualFootPoseInWorld)
   {
      desiredFootPoseInWorld.get(desiredFootPositionInWorld, desiredFootOrientationInWorld);
      actualFootPoseInWorld.get(actualFootPositionInWorld, actualFootOrientationInWorld);

      footstepStatus.setStatus(FootstepStatus.Status.STARTED);
      footstepStatus.setRobotSide(robotSide);
      footstepStatus.setFootstepIndex(currentFootstepIndex.getIntegerValue());
      footstepStatus.setActualFootOrientationInWorld(actualFootOrientationInWorld);
      footstepStatus.setActualFootPositionInWorld(actualFootPositionInWorld);
      footstepStatus.setDesiredFootOrientationInWorld(desiredFootOrientationInWorld);
      footstepStatus.setDesiredFootPositionInWorld(desiredFootPositionInWorld);
      statusOutputManager.reportStatusMessage(footstepStatus);

      executingFootstep.set(true);

      if (yoTime != null)
         timeElapsedWhenFootstepExecuted.set(yoTime.getDoubleValue() - footstepDataListReceivedTime.getDoubleValue());
   }

   public void reportFootstepCompleted(RobotSide robotSide, FramePose3D actualFootPoseInWorld)
   {
      actualFootPoseInWorld.get(actualFootPositionInWorld, actualFootOrientationInWorld);
      desiredFootOrientationInWorld.setToNaN();
      desiredFootPositionInWorld.setToNaN();

      footstepStatus.setStatus(FootstepStatus.Status.COMPLETED);
      footstepStatus.setRobotSide(robotSide);
      footstepStatus.setFootstepIndex(currentFootstepIndex.getIntegerValue());
      footstepStatus.setActualFootOrientationInWorld(actualFootOrientationInWorld);
      footstepStatus.setActualFootPositionInWorld(actualFootPositionInWorld);
      footstepStatus.setDesiredFootOrientationInWorld(desiredFootOrientationInWorld);
      footstepStatus.setDesiredFootPositionInWorld(desiredFootPositionInWorld);
      statusOutputManager.reportStatusMessage(footstepStatus);

//      reusableSpeechPacket.setTextToSpeak(TextToSpeechPacket.FOOTSTEP_COMPLETED);
//      statusOutputManager.reportStatusMessage(reusableSpeechPacket);
      executingFootstep.set(false);
   }

   private final WalkingStatusMessage walkingStatusMessage = new WalkingStatusMessage();

   public void reportWalkingStarted()
   {
      walkingStatusMessage.setWalkingStatus(WalkingStatusMessage.Status.STARTED);
      statusOutputManager.reportStatusMessage(walkingStatusMessage);
      reusableSpeechPacket.setTextToSpeak(TextToSpeechPacket.WALKING);
      statusOutputManager.reportStatusMessage(reusableSpeechPacket);
      isWalking.set(true);
   }

   public void reportWalkingComplete()
   {
      walkingStatusMessage.setWalkingStatus(WalkingStatusMessage.Status.COMPLETED);
      statusOutputManager.reportStatusMessage(walkingStatusMessage);
      isWalking.set(false);
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

   public void reportControllerFailure(FrameVector2D fallingDirection)
   {
      fallingDirection.changeFrame(worldFrame);
      failureStatusMessage.setFallingDirection(fallingDirection);
      statusOutputManager.reportStatusMessage(failureStatusMessage);
   }

   public void requestPlanarRegions()
   {
      planarRegionsListHandler.requestPlanarRegions();
   }

   public void registerCompletedDesiredFootstep(Footstep completedFesiredFootstep)
   {
      lastDesiredFootsteps.put(completedFesiredFootstep.getRobotSide(), completedFesiredFootstep);
   }

   public Footstep getLastDesiredFootstep(RobotSide footstepSide)
   {
      return lastDesiredFootsteps.get(footstepSide);
   }

   private final FramePose3D tempPose = new FramePose3D();

   public Footstep getFootstepAtCurrentLocation(RobotSide robotSide)
   {
      tempPose.setToZero(soleFrames.get(robotSide));
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
         return getDefaultTransferTime();
      return upcomingFootstepTimings.get(0).getTransferTime();
   }

   public double getDefaultSwingTime()
   {
      return defaultSwingTime.getDoubleValue();
   }
   
   public double getNextSwingTime()
   {
      if (upcomingFootstepTimings.isEmpty())
         return getDefaultSwingTime();
      return upcomingFootstepTimings.get(0).getSwingTime();
   }
   
   public double getDefaultTouchdownTime()
   {
      return defaultTouchdownTime.getDoubleValue();
   }
   
   public double getNextTouchdownDuration()
   {
      if (upcomingFootstepTimings.isEmpty())
         return getDefaultTouchdownTime();
      return upcomingFootstepTimings.get(0).getTouchdownDuration();
   }

   public double getInitialTransferTime()
   {
      return defaultInitialTransferTime.getDoubleValue();
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
         return getDefaultStepTime();
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

   private final TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();

   public TransferToAndNextFootstepsData createTransferToAndNextFootstepDataForDoubleSupport(RobotSide transferToSide)
   {
      Footstep transferFromFootstep = getFootstepAtCurrentLocation(transferToSide.getOppositeSide());
      Footstep transferToFootstep = getFootstepAtCurrentLocation(transferToSide);

      transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
      transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);
      transferToAndNextFootstepsData.setTransferToSide(transferToSide);
      transferToAndNextFootstepsData.setTransferFromDesiredFootstep(null);

      if (getCurrentNumberOfFootsteps() > 0)
      {
         transferToAndNextFootstepsData.setNextFootstep(upcomingFootsteps.get(0));
      }
      else
      {
         transferToAndNextFootstepsData.setNextFootstep(null);
      }

      return transferToAndNextFootstepsData;
   }

   public TransferToAndNextFootstepsData createTransferToAndNextFootstepDataForSingleSupport(Footstep transferToFootstep, RobotSide swingSide)
   {
      Footstep transferFromFootstep = getFootstepAtCurrentLocation(swingSide.getOppositeSide());

      transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
      transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);
      transferToAndNextFootstepsData.setTransferToSide(swingSide);
      transferToAndNextFootstepsData.setTransferFromDesiredFootstep(null);

      if (getCurrentNumberOfFootsteps() > 0)
      {
         transferToAndNextFootstepsData.setNextFootstep(upcomingFootsteps.get(0));
      }
      else
      {
         transferToAndNextFootstepsData.setNextFootstep(null);
      }

      return transferToAndNextFootstepsData;
   }

   private void setFootstep(FootstepDataCommand footstepData, boolean trustHeight, Footstep footstepToSet)
   {
      footstepToSet.set(footstepData, trustHeight);

      if (offsettingPlanWithFootstepError.getBooleanValue())
      {
         footstepToSet.addOffset(planOffsetInWorld);
      }
   }

   private void setFootstepTiming(FootstepDataCommand footstep, ExecutionTiming executionTiming, FootstepTiming timingToSet, ExecutionMode executionMode)
   {
      int stepsInQueue = getCurrentNumberOfFootsteps();

      double swingDuration = footstep.getSwingDuration();
      if (Double.isNaN(swingDuration) || swingDuration <= 0.0)
      {
         swingDuration = defaultSwingTime.getDoubleValue();
      }

      double transferDuration = footstep.getTransferDuration();
      if (Double.isNaN(transferDuration) || transferDuration <= 0.0)
      {
         if (stepsInQueue == 0 && !isWalking.getBooleanValue() && executionMode != ExecutionMode.QUEUE)
            transferDuration = defaultInitialTransferTime.getDoubleValue();
         else
            transferDuration = defaultTransferTime.getDoubleValue();
      }
      
      double touchdownDuration = footstep.getTouchdownDuration();
      if(Double.isNaN(touchdownDuration) || touchdownDuration < 0.0)
      {
         touchdownDuration = defaultTouchdownTime.getDoubleValue();
      }

      timingToSet.setTimings(swingDuration, touchdownDuration, transferDuration);

      switch (executionTiming)
      {
      case CONTROL_DURATIONS:
         break;
      case CONTROL_ABSOLUTE_TIMINGS:
         if (stepsInQueue == 0 && !executingFootstep.getBooleanValue() && executionMode != ExecutionMode.QUEUE)
         {
            timingToSet.setAbsoluteTime(transferDuration, footstepDataListReceivedTime.getDoubleValue());
         }
         else if (stepsInQueue == 0)
         {
            double swingStartTime = lastTimingExecuted.getSwingStartTime() + lastTimingExecuted.getSwingTime() + transferDuration;
            timingToSet.setAbsoluteTime(swingStartTime, footstepDataListReceivedTime.getDoubleValue());
         }
         else
         {
            FootstepTiming previousTiming = upcomingFootstepTimings.get(stepsInQueue - 1);
            double swingStartTime = previousTiming.getSwingStartTime() + previousTiming.getSwingTime() + transferDuration;
            timingToSet.setAbsoluteTime(swingStartTime, footstepDataListReceivedTime.getDoubleValue());
         }
         break;
      default:
         throw new RuntimeException("Timing mode not implemented.");
      }
   }

   private void updateTransferTimes(List<FootstepTiming> upcomingFootstepTimings)
   {
      if (upcomingFootstepTimings.isEmpty())
         return;

      FootstepTiming firstTiming = upcomingFootstepTimings.get(0);
      if (!firstTiming.hasAbsoluteTime())
         return;

      double lastSwingStart = firstTiming.getSwingStartTime();
      double lastSwingTime = firstTiming.getSwingTime();
      double touchdownDuration = firstTiming.getTouchdownDuration();
      firstTiming.setTimings(lastSwingTime, touchdownDuration, lastSwingStart);

      for (int footstepIdx = 1; footstepIdx < upcomingFootstepTimings.size(); footstepIdx++)
      {
         FootstepTiming timing = upcomingFootstepTimings.get(footstepIdx);
         double swingStart = timing.getSwingStartTime();
         double swingTime = timing.getSwingTime();
         double transferTime = swingStart - (lastSwingStart + lastSwingTime);
         touchdownDuration = timing.getTouchdownDuration();
         timing.setTimings(swingTime, touchdownDuration, transferTime);

         lastSwingStart = swingStart;
         lastSwingTime = swingTime;
      }
   }

   private boolean checkTimings(List<FootstepTiming> upcomingFootstepTimings)
   {
      // TODO: This is somewhat duplicated in the PacketValidityChecker.
      // The reason it has to be here is that this also checks that the timings are monotonically increasing if messages
      // are queued. It also rejects the message if this class was not created with time in which case absolute footstep
      // timings can not be executed.

      if (upcomingFootstepTimings.isEmpty())
         return true;

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
         return false;
      }

      if (atLeastOneFootstepHadTiming && yoTime == null)
      {
         PrintTools.warn("Recieved absolute footstep timings but " + getClass().getSimpleName() + " was created with no yoTime.");
         return false;
      }

      return true;
   }

   public void addOffsetVector(FrameVector3D offset)
   {
      if (!offsettingPlanWithFootstepError.getBooleanValue())
      {
         return;
      }

      for (int stepIdx = 0; stepIdx < upcomingFootsteps.size(); stepIdx++)
      {
         Footstep footstep = upcomingFootsteps.get(stepIdx);
         footstep.addOffset(offset);
      }

      this.planOffsetInWorld.add(offset);
      comTrajectoryHandler.setPositionOffset(this.planOffsetInWorld);

      updateVisualization();
      statusOutputManager.reportStatusMessage(new PlanOffsetStatus(planOffsetInWorld));
   }
}
