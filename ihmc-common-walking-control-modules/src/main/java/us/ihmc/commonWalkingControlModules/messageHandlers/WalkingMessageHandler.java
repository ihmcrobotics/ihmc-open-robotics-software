package us.ihmc.commonWalkingControlModules.messageHandlers;

import java.util.List;

import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.mutable.MutableDouble;

import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegionsList;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.trajectories.trajectorypoints.EuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

public class WalkingMessageHandler implements SCS2YoGraphicHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private static final int maxNumberOfFootsteps = 100;
   private final RecyclingArrayList<Footstep> upcomingFootsteps = new RecyclingArrayList<>(maxNumberOfFootsteps, Footstep.class);
   private final RecyclingArrayList<FootstepTiming> upcomingFootstepTimings = new RecyclingArrayList<>(maxNumberOfFootsteps, FootstepTiming.class);
   private final RecyclingArrayList<StepConstraintRegionsList> upcomingStepConstraints = new RecyclingArrayList<>(maxNumberOfFootsteps,
                                                                                                                  StepConstraintRegionsList::new);

   private final RecyclingArrayList<MutableDouble> pauseDurationAfterStep = new RecyclingArrayList<>(maxNumberOfFootsteps, MutableDouble.class);

   private final YoBoolean isPausedWithSteps = new YoBoolean("IsPausedWithSteps", registry);
   private final YoDouble timeToContinueWalking = new YoDouble("TimeToContinueWalking", registry);
   private final DoubleProvider minimumPauseTime = new DoubleParameter("MinimumPauseTime", registry, 0.0);

   private final SideDependentList<Footstep> footstepsAtCurrentLocation = new SideDependentList<>();
   private final SideDependentList<Footstep> lastDesiredFootsteps = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();

   private final SideDependentList<RecyclingArrayDeque<FootTrajectoryCommand>> upcomingFootTrajectoryCommandListForFlamingoStance = new SideDependentList<>();
   private final SideDependentList<RecyclingArrayDeque<LegTrajectoryCommand>> upcomingLegTrajectoryCommandListForFlamingoStance = new SideDependentList<>();

   private final FootstepQueueStatusMessage footstepQueueStatusMessage = new FootstepQueueStatusMessage();
   private final StatusMessageOutputManager statusOutputManager;
   private final PlanOffsetStatus planOffsetStatus = new PlanOffsetStatus();

   private final YoInteger currentFootstepIndex = new YoInteger("currentFootstepIndex", registry);
   private final YoInteger currentNumberOfFootsteps = new YoInteger("currentNumberOfFootsteps", registry);
   private final YoBoolean isWalkingPaused = new YoBoolean("isWalkingPaused", registry);
   private final YoBoolean isWalkingResuming = new YoBoolean("isWalkingResuming", registry);
   private final YoBoolean clearFootstepsAfterPause = new YoBoolean("clearFootstepsAfterPause", registry);
   private final YoDouble defaultTransferTime = new YoDouble("defaultTransferTime", registry);
   private final YoDouble finalTransferTime = new YoDouble("finalTransferTime", registry);
   private final YoDouble defaultSwingTime = new YoDouble("defaultSwingTime", registry);
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

   private final YoBoolean offsettingXYPlanWithFootstepError = new YoBoolean("offsettingXYPlanWithFootstepError", registry);
   private final YoBoolean offsettingHeightPlanWithFootstepError = new YoBoolean("offsettingHeightPlanWithFootstepError", registry);

   private final YoFrameVector3D planOffsetInWorld = new YoFrameVector3D("planOffsetInWorld", worldFrame, registry);
   private final YoFrameVector3D planOffsetInWorldPrevious = new YoFrameVector3D("planOffsetInWorldPrevious", worldFrame, registry);

   private final DoubleProvider maxStepDistance = new DoubleParameter("MaxStepDistance", registry, Double.POSITIVE_INFINITY);
   private final DoubleProvider maxStepHeightChange = new DoubleParameter("MaxStepHeightChange", registry, Double.POSITIVE_INFINITY);
   private final DoubleProvider maxSwingDistance = new DoubleParameter("MaxSwingDistance", registry, Double.POSITIVE_INFINITY);

   public WalkingMessageHandler(double defaultTransferTime,
                                double defaultSwingTime,
                                double defaultInitialTransferTime,
                                double defaultFinalTransferTime,
                                SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                StatusMessageOutputManager statusOutputManager,
                                YoDouble yoTime,
                                YoGraphicsListRegistry yoGraphicsListRegistry,
                                YoRegistry parentRegistry)
   {
      this.statusOutputManager = statusOutputManager;

      this.yoTime = yoTime;
      footstepDataListReceivedTime.setToNaN();

      this.defaultTransferTime.set(defaultTransferTime);
      this.defaultSwingTime.set(defaultSwingTime);
      this.defaultInitialTransferTime.set(defaultInitialTransferTime);
      this.defaultFinalTransferTime.set(defaultFinalTransferTime);
      this.finalTransferTime.set(defaultFinalTransferTime);

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
         Footstep footstepAtCurrentLocation = new Footstep(robotSide);
         footstepsAtCurrentLocation.put(robotSide, footstepAtCurrentLocation);
         soleFrames.put(robotSide, contactableFoot.getSoleFrame());

         upcomingFootTrajectoryCommandListForFlamingoStance.put(robotSide, new RecyclingArrayDeque<>(FootTrajectoryCommand.class, FootTrajectoryCommand::set));
         upcomingLegTrajectoryCommandListForFlamingoStance.put(robotSide, new RecyclingArrayDeque<>(LegTrajectoryCommand.class, LegTrajectoryCommand::set));
      }

      for (int i = 0; i < numberOfFootstepsToVisualize; i++)
         upcomingFoostepSide[i] = new YoEnum<>("upcomingFootstepSide" + i, registry, RobotSide.class, true);

      if (yoGraphicsListRegistry != null)
         footstepListVisualizer = new FootstepListVisualizer(contactableFeet, yoGraphicsListRegistry, registry);
      else
         footstepListVisualizer = null;
      updateVisualization();

      momentumTrajectoryHandler = new MomentumTrajectoryHandler(yoTime, registry);
      comTrajectoryHandler = new CenterOfMassTrajectoryHandler(yoTime, registry);

      parentRegistry.addChild(registry);
   }

   public void setFinalTransferTime(double finalTransferTime)
   {
      this.finalTransferTime.set(finalTransferTime);
   }

   public void handleFootstepDataListCommand(FootstepDataListCommand command)
   {
      if (command.getNumberOfFootsteps() > 0)
      {
         switch (command.getExecutionMode())
         {
            case OVERRIDE:
               offsettingXYPlanWithFootstepError.set(command.isOffsetFootstepsWithExecutionError());
               offsettingHeightPlanWithFootstepError.set(command.isOffsetFootstepsHeightWithExecutionError());
               planOffsetInWorld.setToZero();
               planOffsetInWorldPrevious.setToZero();
               clearFootsteps();
               clearFlamingoCommands();
               break;
            case QUEUE:
               // TODO review the use of this. 
               boolean checkForInconsistencies = !upcomingFootsteps.isEmpty() || currentNumberOfFootsteps.getIntegerValue() > 0;
               if (checkForInconsistencies)
               {
                  if (offsettingXYPlanWithFootstepError.getValue() != command.isOffsetFootstepsWithExecutionError())
                  {
                     LogTools.warn("Recieved a queued message that has a different setting for offsetting footsteps with execution error!");
                  }
                  if (offsettingHeightPlanWithFootstepError.getValue() != command.isOffsetFootstepsHeightWithExecutionError())
                  {
                     LogTools.warn("Recieved a queued message that has a different setting for offsetting height of footsteps with execution error!");
                  }
                  if (currentNumberOfFootsteps.getIntegerValue() < 1 && !executingFootstep.getBooleanValue())
                  {
                     if (command.getExecutionTiming() == ExecutionTiming.CONTROL_ABSOLUTE_TIMINGS)
                     {
                        LogTools.warn("Can not command a queued message with absolute timings if all footsteps were already exectuted. You gotta send faster!");
                        return;
                     }

                     if (command.getPreviousCommandId() == lastCommandID.getValue())
                     {
                        // If we queued a command and the previous already finished, just continue as if everything is normal
                        break;
                     }
                     else if (upcomingFootsteps.isEmpty())
                     {
                        // No footstep, let's do it.
                        break;
                     }
                     else
                     {
                        LogTools.warn("Queued footstep previous command id {} != controller's previous command id {}."
                                      + " Send an override message instead. Command ignored.", command.getPreviousCommandId(), lastCommandID.getValue());
                     }
                     return;
                  }
               }
               else
               {
                  // We don't have any steps in the queue, so it's effectively the same thing as overriding.
                  offsettingXYPlanWithFootstepError.set(command.isOffsetFootstepsWithExecutionError());
                  offsettingHeightPlanWithFootstepError.set(command.isOffsetFootstepsHeightWithExecutionError());
                  planOffsetInWorld.setToZero();
                  planOffsetInWorldPrevious.setToZero();
               }
               break;
            default:
               LogTools.warn("Unknown " + ExecutionMode.class.getSimpleName() + " value: " + command.getExecutionMode() + ". Command ignored.");
               return;
         }
      }

      if (command.getNumberOfFootsteps() + currentNumberOfFootsteps.getIntegerValue() > maxNumberOfFootsteps)
      {
         LogTools.warn("Can not exceed " + maxNumberOfFootsteps + " footsteps stopping execution.");
         clearFootsteps();
         return;
      }

      lastCommandID.set(command.getCommandId());

      if (isWalkingPaused.getValue())
      {
         /*
          * The walking was paused, when paused isWalking remains true. We're receiving a new series of
          * footsteps, let's reset isWalking so the controller reports that it starts walking.
          */
         isWalking.set(false);
      }

      isWalkingPaused.set(false);
      double commandDefaultTransferTime = command.getDefaultTransferDuration();
      double commandDefaultSwingTime = command.getDefaultSwingDuration();
      if (!Double.isNaN(commandDefaultSwingTime) && commandDefaultSwingTime > 1.0e-2 && !Double.isNaN(commandDefaultTransferTime)
          && commandDefaultTransferTime >= 0.0)
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
      boolean areFootstepsAdjustable = command.areFootstepsAdjustable();
      boolean shouldCheckPlanForReachability = command.getShouldCheckForReachability();

      for (int i = 0; i < command.getNumberOfFootsteps(); i++)
      {
         boolean shouldCheckStepForReachability = shouldCheckPlanForReachability || command.getFootstep(i).getShouldCheckForReachability();
         setFootstepTiming(command.getFootstep(i),
                           command.getExecutionTiming(),
                           upcomingFootstepTimings.add(),
                           pauseDurationAfterStep.add(),
                           command.getExecutionMode());
         setFootstep(command.getFootstep(i), trustHeightOfFootsteps, areFootstepsAdjustable, shouldCheckStepForReachability, upcomingFootsteps.add());
         if (command.getFootstep(i).getStepConstraints().getNumberOfConstraints() > 0)
            command.getFootstep(i).getStepConstraints().get(upcomingStepConstraints.add());
         else
            command.getDefaultStepConstraints().get(upcomingStepConstraints.add());
         currentNumberOfFootsteps.increment();
      }

      if (!checkTimings(upcomingFootstepTimings, yoTime))
      {
         clearFootsteps();
      }

      if (!checkFootsteps(upcomingFootsteps,
                          soleFrames,
                          maxStepDistance.getValue(),
                          maxStepHeightChange.getValue(),
                          maxSwingDistance.getValue(),
                          tempStanceLocation,
                          tempStepOrigin))
      {
         clearFootsteps();
      }

      checkForPause();

      updateVisualization();
   }

   public void handlePauseWalkingCommand(PauseWalkingCommand command)
   {
      if (!command.isPauseRequested() && isWalkingPaused.getValue())
         isWalkingResuming.set(true);

      isWalkingPaused.set(command.isPauseRequested());
      clearFootstepsAfterPause.set(command.getClearRemainingFootstepQueue());
   }

   public void handleFootTrajectoryCommand(List<FootTrajectoryCommand> commands)
   {
      for (int i = 0; i < commands.size(); i++)
      {
         FootTrajectoryCommand command = commands.get(i);
         upcomingFootTrajectoryCommandListForFlamingoStance.get(command.getRobotSide()).addLast(command);
      }
   }

   public void handleLegTrajectoryCommand(List<LegTrajectoryCommand> commands)
   {
      for (int i = 0; i < commands.size(); i++)
      {
         LegTrajectoryCommand command = commands.get(i);
         upcomingLegTrajectoryCommandListForFlamingoStance.get(command.getRobotSide()).addLast(command);
      }
   }

   public void handleMomentumTrajectoryCommand(MomentumTrajectoryCommand command)
   {
      momentumTrajectoryHandler.handleMomentumTrajectory(command);
   }

   /**
    * This method will pack the angular momentum trajectory for planning the ICP trajectory. The
    * parameters {@code startTime} and {@code endTime} refer to absolute controller time. To get the
    * angular momentum trajectory from the current time to 1.0 seconds in the future the start time
    * must be the value of yoTime and the end time must be the value of yoTime + 1.0. The
    * {@code numberOfPoints} parameter defines in how many points the trajectory will be sampled. The
    * packed trajectory will include the end points of the interval, therefore, the number of points
    * must be equal or grater then two. If the interval of interest is not available the trajectory to
    * pack will be empty. The times of the packed trajectory points will be relative to the start time
    * of the interval.
    *
    * @param startTime        is the controller time for the start of the interval for which the
    *                         trajectory is packed
    * @param endTime          is the controller time for the end of the interval for which the
    *                         trajectory is packed
    * @param numberOfPoints   the number of sampling points of the trajectory
    * @param trajectoryToPack the trajectory will be packed in here
    */
   public void getAngularMomentumTrajectory(double startTime, double endTime, int numberOfPoints, RecyclingArrayList<EuclideanTrajectoryPoint> trajectoryToPack)
   {
      momentumTrajectoryHandler.getAngularMomentumTrajectory(startTime, endTime, numberOfPoints, trajectoryToPack);
   }

   public FootstepQueueStatusMessage updateAndReturnFootstepQueueStatus(Footstep footstepBeingExecuted,
                                                                        FootstepTiming footstepTimingBeingExecuted,
                                                                        List<StepConstraintRegion> stepConstraintsBeingExecuted,
                                                                        double timeInSupportSequence,
                                                                        boolean isFirstSTepCurrentlyInSwing)
   {
      footstepQueueStatusMessage.setTimeInSupportSequence(timeInSupportSequence);
      footstepQueueStatusMessage.setIsFirstStepInSwing(isFirstSTepCurrentlyInSwing);

      footstepQueueStatusMessage.getQueuedFootstepList().clear();
      // This foot is currently being taken. Add it to the front of the queue.
      if (footstepBeingExecuted != null)
      {
         QueuedFootstepStatusMessage queuedFootstepStatusMessage = footstepQueueStatusMessage.getQueuedFootstepList().add();
         queuedFootstepStatusMessage.setSequenceId(footstepBeingExecuted.getSequenceID());
         packQueuedFootstepStatus(queuedFootstepStatusMessage, footstepBeingExecuted, footstepTimingBeingExecuted, stepConstraintsBeingExecuted);

      }
      // Add the other steps to the queue.
      for (int i = 0; i < upcomingFootsteps.size(); i++)
      {
         QueuedFootstepStatusMessage queuedFootstepStatusMessage = footstepQueueStatusMessage.getQueuedFootstepList().add();
         Footstep upcomingFootstep = upcomingFootsteps.get(i);
         FootstepTiming upcomingTiming = upcomingFootstepTimings.get(i);
         StepConstraintRegionsList stepConstraints = upcomingStepConstraints.get(i);

         queuedFootstepStatusMessage.setSequenceId(upcomingFootstep.getSequenceID());
         packQueuedFootstepStatus(queuedFootstepStatusMessage, upcomingFootstep, upcomingTiming, stepConstraints);
      }

      return footstepQueueStatusMessage;
   }

   private static void packQueuedFootstepStatus(QueuedFootstepStatusMessage messasgeToPack,
                                                Footstep footstep,
                                                FootstepTiming footstepTiming,
                                                StepConstraintRegionsList stepConstraints)
   {
      packQueuedFootstepStatus(messasgeToPack, footstep, footstepTiming, stepConstraints.getAsList());
   }

   private static void packQueuedFootstepStatus(QueuedFootstepStatusMessage messasgeToPack,
                                                Footstep footstep,
                                                FootstepTiming footstepTiming,
                                                List<StepConstraintRegion> stepConstraints)
   {
      messasgeToPack.setRobotSide(footstep.getRobotSide().toByte());
      messasgeToPack.getLocation().set(footstep.getFootstepPose().getPosition());
      messasgeToPack.getOrientation().set(footstep.getFootstepPose().getOrientation());
      messasgeToPack.setSwingDuration(footstepTiming.getSwingTime());
      messasgeToPack.setTransferDuration(footstepTiming.getTransferTime());

      // TODO : don't include this stuff for now. It makes the message too large to get sent for some reason.
//      messasgeToPack.getPredictedContactPoints2d().clear();
//      if (footstep.getPredictedContactPoints() != null)
//      {
//         for (int i = 0; i < footstep.getPredictedContactPoints().size(); i++)
//            messasgeToPack.getPredictedContactPoints2d().add().set(footstep.getPredictedContactPoints().get(i));
//      }

//      StepConstraintRegionsList.getAsMessage(stepConstraints, messasgeToPack.getStepConstraints());
   }

   public MomentumTrajectoryHandler getMomentumTrajectoryHandler()
   {
      return momentumTrajectoryHandler;
   }

   public CenterOfMassTrajectoryHandler getComTrajectoryHandler()
   {
      return comTrajectoryHandler;
   }

   public void handleComTrajectoryCommand(CenterOfMassTrajectoryCommand command)
   {
      comTrajectoryHandler.handleComTrajectory(command);
   }

   public void setUpcomingFootstepTransferDuration(double duration)
   {
      if (upcomingFootstepTimings.isEmpty())
         throw new RuntimeException("Can not get timing, no upcoming footstep.");

      upcomingFootstepTimings.get(0).setTransferTime(duration);
   }

   /**
    * This method will set the provided timing to the timing of the {@code i}'th upcoming footstep. If
    * there is less then {@code i} upcoming steps this method will throw a {@link RuntimeException}. To
    * check how many footsteps are upcoming use {@link #getCurrentNumberOfFootsteps()}.
    *
    * @param i            is the index of the upcoming footstep timing that will be packed.
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
    * This method will set the provided footstep to the {@code i}'th upcoming footstep. If there is
    * less then {@code i} upcoming steps this method will throw a {@link RuntimeException}. To check
    * how many footsteps are upcoming use {@link #getCurrentNumberOfFootsteps()}.
    *
    * @param i              is the index of the upcoming footstep that will be packed.
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
    * This method will pack the provided footstep and timing with the next upcoming step. It will then
    * remove that footstep from the list of upcoming footsteps. Use this method if you wish to remove
    * the next upcoming step because the execution has started. If there is no upcoming step this
    * method will throw a {@link RuntimeException}.
    *
    * @param footstepToPack will be set to the next footstep in the list of upcoming steps.
    * @param timingToPack   will be set to the next footsteps timing in the list of upcoming steps.
    */
   public void poll(Footstep footstepToPack, FootstepTiming timingToPack)
   {
      if (getStepsBeforePause() == 0)
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
      pauseDurationAfterStep.remove(0);
      upcomingFootsteps.remove(0);
   }

   public void pollStepConstraints(StepConstraintRegionsList commandToPack)
   {
      commandToPack.set(upcomingStepConstraints.get(0));
      upcomingStepConstraints.remove(0);
   }

   /**
    * This method can be used to adjust the timing of the upcoming footstep. It will throw a
    * {@link RuntimeException} if there are no footsteps in the queue.
    *
    * @param newSwingDuration    is the new swing duration for the adjusted timing
    * @param newTransferDuration is the new transfer duration for the adjusted timing.
    */
   public void adjustTiming(double newSwingDuration, double newTransferDuration)
   {
      if (upcomingFootstepTimings.isEmpty())
      {
         throw new RuntimeException("Can not adjust timing of upciming step - have no steps.");
      }
      upcomingFootstepTimings.get(0).setTimings(newSwingDuration, newTransferDuration);
   }

   public FootTrajectoryCommand pollFootTrajectoryForFlamingoStance(RobotSide swingSide)
   {
      return upcomingFootTrajectoryCommandListForFlamingoStance.get(swingSide).poll();
   }

   public LegTrajectoryCommand pollLegTrajectoryForFlamingoStance(RobotSide swingSide)
   {
      return upcomingLegTrajectoryCommandListForFlamingoStance.get(swingSide).poll();
   }

   public boolean hasUpcomingFootsteps()
   {
      return getStepsBeforePause() > 0 && !isWalkingPaused.getBooleanValue();
   }

   private void checkForPause()
   {
      if (!pauseDurationAfterStep.isEmpty() && Double.isFinite(pauseDurationAfterStep.get(0).doubleValue()))
      {
         double pause = pauseDurationAfterStep.get(0).doubleValue();
         timeToContinueWalking.set(yoTime.getValue() + pause);
         isPausedWithSteps.set(true);
         pauseDurationAfterStep.get(0).setValue(Double.NaN);
      }
   }

   private int getStepsBeforePause()
   {
      // Check if we can continue walking if we are paused.
      if (isPausedWithSteps.getValue() && yoTime.getValue() >= timeToContinueWalking.getValue())
      {
         isPausedWithSteps.set(false);
      }

      if (isWalkingPaused.getValue() || isPausedWithSteps.getValue())
      {
         return 0;
      }

      int stepIndex = 0;
      while (stepIndex < upcomingFootsteps.size())
      {
         if (Double.isFinite(pauseDurationAfterStep.get(stepIndex).doubleValue()))
         {
            return stepIndex;
         }
         stepIndex++;
      }

      return stepIndex;
   }

   public boolean isNextFootstepFor(RobotSide swingSide)
   {
      if (!hasUpcomingFootsteps())
      {
         return false;
      }

      return upcomingFootsteps.get(0).getRobotSide() == swingSide;
   }

   public boolean hasFootTrajectoryForFlamingoStance(RobotSide swingSide)
   {
      return !upcomingFootTrajectoryCommandListForFlamingoStance.get(swingSide).isEmpty();
   }

   public boolean hasLegTrajectoryForFlamingoStance(RobotSide swingSide)
   {
      return !upcomingLegTrajectoryCommandListForFlamingoStance.get(swingSide).isEmpty();
   }

   public void clearFlamingoCommands(RobotSide robotSide)
   {
      upcomingFootTrajectoryCommandListForFlamingoStance.get(robotSide).clear();
      upcomingLegTrajectoryCommandListForFlamingoStance.get(robotSide).clear();
   }

   public void clearFlamingoCommands()
   {
      for (RobotSide robotSide : RobotSide.values)
         clearFlamingoCommands(robotSide);
   }

   public void clearFootsteps()
   {
      upcomingFootsteps.clear();
      upcomingFootstepTimings.clear();
      upcomingStepConstraints.clear();
      pauseDurationAfterStep.clear();
      currentNumberOfFootsteps.set(0);
      currentFootstepIndex.set(0);
      updateVisualization();
   }

   private final TextToSpeechPacket reusableSpeechPacket = new TextToSpeechPacket();
   private final WalkingControllerFailureStatusMessage failureStatusMessage = new WalkingControllerFailureStatusMessage();
   private final FootstepStatusMessage footstepStatus = new FootstepStatusMessage();

   public void reportFootstepStarted(RobotSide robotSide,
                                     FramePose3DReadOnly desiredFootPoseInWorld,
                                     FramePose3DReadOnly actualFootPoseInWorld,
                                     double swingDuration,
                                     long sequenceID)
   {
      reportFootstepStatus(robotSide, FootstepStatus.STARTED, desiredFootPoseInWorld, actualFootPoseInWorld, swingDuration, sequenceID);
      executingFootstep.set(true);

      if (yoTime != null)
         timeElapsedWhenFootstepExecuted.set(yoTime.getDoubleValue() - footstepDataListReceivedTime.getDoubleValue());
   }

   public void reportFootstepCompleted(RobotSide robotSide,
                                       FramePose3DReadOnly desiredFootPoseInWorld,
                                       FramePose3DReadOnly actualFootPoseInWorld,
                                       double swingDuration,
                                       long sequenceID)
   {
      reportFootstepStatus(robotSide, FootstepStatus.COMPLETED, desiredFootPoseInWorld, actualFootPoseInWorld, swingDuration, sequenceID);
      executingFootstep.set(false);
   }

   private void reportFootstepStatus(RobotSide robotSide,
                                     FootstepStatus status,
                                     FramePose3DReadOnly desiredFootPoseInWorld,
                                     FramePose3DReadOnly actualFootPoseInWorld,
                                     double swingDuration,
                                     long sequenceID)
   {
      desiredFootPoseInWorld.checkReferenceFrameMatch(worldFrame);
      actualFootPoseInWorld.checkReferenceFrameMatch(worldFrame);

      footstepStatus.setFootstepStatus(status.toByte());
      footstepStatus.setSequenceId(sequenceID);
      footstepStatus.setRobotSide(robotSide.toByte());
      footstepStatus.setFootstepIndex(currentFootstepIndex.getIntegerValue());
      footstepStatus.getActualFootOrientationInWorld().set(actualFootPoseInWorld.getOrientation());
      footstepStatus.getActualFootPositionInWorld().set(actualFootPoseInWorld.getPosition());
      footstepStatus.getDesiredFootOrientationInWorld().set(desiredFootPoseInWorld.getOrientation());
      footstepStatus.getDesiredFootPositionInWorld().set(desiredFootPoseInWorld.getPosition());
      footstepStatus.setSwingDuration(swingDuration);
      statusOutputManager.reportStatusMessage(footstepStatus);
   }

   private final WalkingStatusMessage walkingStatusMessage = new WalkingStatusMessage();

   public void reportWalkingStarted()
   {
      if (isWalking.getValue())
      {
         if (isWalkingResuming.getValue())
         {
            walkingStatusMessage.setWalkingStatus(WalkingStatus.RESUMED.toByte());
            statusOutputManager.reportStatusMessage(walkingStatusMessage);
            isWalkingResuming.set(false);
         }

         return;
      }

      walkingStatusMessage.setWalkingStatus(WalkingStatus.STARTED.toByte());
      statusOutputManager.reportStatusMessage(walkingStatusMessage);
      reusableSpeechPacket.setTextToSpeak(TextToSpeechPacket.WALKING);
      statusOutputManager.reportStatusMessage(reusableSpeechPacket);
      isWalking.set(true);
   }

   public void reportWalkingComplete()
   {
      if (!isWalking.getValue())
      {
         return;
      }

      // If we have transitioned to standing this will be called. However, we might just be taking a break because of a long
      // transfer. In that case do not report walking complete. Instead compute when to continue walking.
      if (!upcomingFootsteps.isEmpty())
      {
         walkingStatusMessage.setWalkingStatus(WalkingStatus.PAUSED.toByte());
         statusOutputManager.reportStatusMessage(walkingStatusMessage);
         if (clearFootstepsAfterPause.getBooleanValue())
         {
            clearFootsteps();
            clearFootstepsAfterPause.set(false);
         }
         checkForPause();
         return;
      }

      walkingStatusMessage.setWalkingStatus(WalkingStatus.COMPLETED.toByte());
      statusOutputManager.reportStatusMessage(walkingStatusMessage);
      isWalking.set(false);
   }

   public void reportWalkingAbortRequested()
   {
      WalkingStatusMessage walkingStatusMessage = new WalkingStatusMessage();
      walkingStatusMessage.setWalkingStatus(WalkingStatus.ABORT_REQUESTED.toByte());
      statusOutputManager.reportStatusMessage(walkingStatusMessage);
   }

   public void reportControllerFailure(FrameVector3D fallingDirection)
   {
      fallingDirection.changeFrame(worldFrame);
      failureStatusMessage.getFallingDirection().set(fallingDirection);
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
      return getStepsBeforePause();
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

      if (footstepListVisualizer != null)
         footstepListVisualizer.update(upcomingFootsteps);
   }

   public void updateVisualizationAfterFootstepAdjustement(Footstep adjustedFootstep)
   {
      if (footstepListVisualizer != null)
         footstepListVisualizer.updateFirstFootstep(adjustedFootstep);
   }

   private final TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();

   public TransferToAndNextFootstepsData createTransferToAndNextFootstepDataForDoubleSupport(RobotSide transferToSide)
   {
      transferToAndNextFootstepsData.setTransferToPosition(soleFrames.get(transferToSide));
      transferToAndNextFootstepsData.setTransferToSide(transferToSide);

      return transferToAndNextFootstepsData;
   }

   public TransferToAndNextFootstepsData createTransferToAndNextFootstepDataForSingleSupport(Footstep transferToFootstep, RobotSide swingSide)
   {
      transferToAndNextFootstepsData.setTransferToPosition(transferToFootstep.getFootstepPose().getPosition());
      transferToAndNextFootstepsData.setTransferToSide(swingSide);

      return transferToAndNextFootstepsData;
   }

   private void setFootstep(FootstepDataCommand footstepData,
                            boolean trustHeight,
                            boolean isAdjustable,
                            boolean shouldCheckForReachability,
                            Footstep footstepToSet)
   {
      footstepToSet.set(footstepData, trustHeight, isAdjustable, shouldCheckForReachability);
      footstepToSet.addOffset(planOffsetInWorld);
   }

   private void setFootstepTiming(FootstepDataCommand footstep,
                                  ExecutionTiming executionTiming,
                                  FootstepTiming timingToSet,
                                  MutableDouble pauseDurationToSet,
                                  ExecutionMode executionMode)
   {
      int stepsInQueue = upcomingFootsteps.size();

      double swingDuration = footstep.getSwingDuration();
      if (Double.isNaN(swingDuration) || swingDuration <= 0.0)
      {
         swingDuration = defaultSwingTime.getDoubleValue();
      }

      double transferDuration = footstep.getTransferDuration();
      if (Double.isNaN(transferDuration) || transferDuration <= 0.0)
      {
         // There are no upcoming steps, we are not walking, and this is an overwrite message:
         if (stepsInQueue == 0 && !isWalking.getBooleanValue())
            transferDuration = defaultInitialTransferTime.getDoubleValue();
         else
            transferDuration = defaultTransferTime.getDoubleValue();
      }

      timingToSet.setTimings(swingDuration, transferDuration);
      timingToSet.setTouchdownDuration(footstep.getTouchdownDuration());
      timingToSet.setLiftoffDuration(footstep.getLiftoffDuration());

      switch (executionTiming)
      {
         case CONTROL_DURATIONS:
            timingToSet.removeAbsoluteTime();
            break;
         case CONTROL_ABSOLUTE_TIMINGS:
            if (stepsInQueue == 0 && !executingFootstep.getBooleanValue() && executionMode == ExecutionMode.OVERRIDE)
            {
               // There are no upcoming steps, we are not executing a step, and this is an overwrite message:
               footstepDataListReceivedTime.set(yoTime.getDoubleValue());
               timingToSet.setAbsoluteTime(transferDuration, footstepDataListReceivedTime.getDoubleValue());
            }
            else if (stepsInQueue == 0 && !executingFootstep.getBooleanValue())
            {
               // This message should have been rejected above.
               throw new RuntimeException("This should not happen. We are not executing a footstep and there is no steps in "
                                          + "queue so there is no way to compute absolute timings for the step as was requested.");
            }
            else if (stepsInQueue == 0)
            {
               // In this case the step we are executing right now is the last step we have. So compute the timings from that step.
               double swingStartTime = lastTimingExecuted.getSwingStartTime() + lastTimingExecuted.getSwingTime() + transferDuration;
               timingToSet.setAbsoluteTime(swingStartTime, footstepDataListReceivedTime.getDoubleValue());
            }
            else
            {
               // In this case we still have steps in queue so we compute the timings from the last step in the queue.
               FootstepTiming previousTiming = upcomingFootstepTimings.get(stepsInQueue - 1);
               double swingStartTime = previousTiming.getSwingStartTime() + previousTiming.getSwingTime() + transferDuration;
               timingToSet.setAbsoluteTime(swingStartTime, footstepDataListReceivedTime.getDoubleValue());
            }
            break;
         default:
            throw new RuntimeException("Timing mode not implemented.");
      }

      // The transfer is long enough that we can go to standing, pause, and then keep walking:
      double timeToGoToStanding = stepsInQueue == 0 ? 0.0 : finalTransferTime.getValue();
      double pauseTime = timingToSet.getTransferTime() - timeToGoToStanding - defaultInitialTransferTime.getValue();
      if (pauseTime >= minimumPauseTime.getValue())
      {
         timingToSet.setTransferTime(defaultInitialTransferTime.getValue());
         pauseDurationToSet.setValue(pauseTime);
      }
      else
      {
         pauseDurationToSet.setValue(Double.NaN);
      }
   }

   private static boolean checkTimings(List<FootstepTiming> upcomingFootstepTimings, YoDouble yoTime)
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
         LogTools.warn("Recieved footstep data with invalid timings. Using swing and transfer times instead.");
         return false;
      }

      if (atLeastOneFootstepHadTiming && yoTime == null)
      {
         LogTools.warn("Recieved absolute footstep timings but " + WalkingMessageHandler.class.getSimpleName() + " was created with no yoTime.");
         return false;
      }

      return true;
   }

   private final FramePoint3DBasics tempStanceLocation = new FramePoint3D();
   private final FramePoint3DBasics tempStepOrigin = new FramePoint3D();

   /**
    * Does sanity checks on the provided footsteps:
    * <p>
    * Will return false if this method finds</br>
    * - a step that is too far from the stance foot in XY</br>
    * - a step that has a swing trajectory that is too far from a straight line swing in XY</br>
    * - a step that has a swing trajectory that is too far from the step origin or destination in
    * Z</br>
    *
    * @return if the footsteps were found to be safe
    */
   private static boolean checkFootsteps(List<Footstep> footsteps,
                                         SideDependentList<ReferenceFrame> soleFrames,
                                         double maxStepDistance,
                                         double maxStepHeightChange,
                                         double maxSwingDistance,
                                         FramePoint3DBasics tempStanceLocation,
                                         FramePoint3DBasics tempStepOrigin)
   {
      if (footsteps.isEmpty())
      {
         return true;
      }

      tempStanceLocation.setToZero(soleFrames.get(footsteps.get(0).getRobotSide().getOppositeSide()));
      tempStepOrigin.setToZero(soleFrames.get(footsteps.get(0).getRobotSide()));
      tempStanceLocation.changeFrame(worldFrame);
      tempStepOrigin.changeFrame(worldFrame);

      for (int i = 0; i < footsteps.size(); i++)
      {
         Footstep footstep = footsteps.get(i);
         FixedFramePoint3DBasics stepPosition = footstep.getFootstepPose().getPosition();
         double distance = stepPosition.distanceXY(tempStanceLocation);
         if (distance > maxStepDistance)
         {
            LogTools.warn("Received step that was too far to be executed safely. Distance from previous step was " + distance
                          + ". If that is acceptable increase the MaxStepDistance parameter.");
            return false;
         }

         double heightChange = Math.abs(stepPosition.getZ() - tempStanceLocation.getZ());
         if (heightChange > maxStepHeightChange)
         {
            LogTools.warn("Received step that was too far to be executed safely. Height change w.r.t. previous step was " + heightChange
                          + ". If that is acceptable increase the MaxStepHeightChange parameter.");
            return false;
         }

         // Check the swing trajectory
         if (footstep.getTrajectoryType() == TrajectoryType.WAYPOINTS)
         {
            List<FrameSE3TrajectoryPoint> swingTrajectory = footstep.getSwingTrajectory();
            for (int j = 0; j < swingTrajectory.size(); j++)
            {
               FramePoint3DReadOnly position = swingTrajectory.get(j).getPosition();
               if (!checkPositionIsValid(position, tempStepOrigin, stepPosition, maxSwingDistance))
               {
                  return false;
               }
            }
         }

         // Check position waypoints
         if (footstep.getTrajectoryType() == TrajectoryType.CUSTOM)
         {
            List<FramePoint3D> positionWaypoints = footstep.getCustomPositionWaypoints();
            for (int j = 0; j < positionWaypoints.size(); j++)
            {
               FramePoint3DReadOnly position = positionWaypoints.get(j);
               if (!checkPositionIsValid(position, tempStepOrigin, stepPosition, maxSwingDistance))
               {
                  return false;
               }
            }
         }

         if (i < footsteps.size() - 1)
         {
            if (footsteps.get(i + 1).getRobotSide() != footstep.getRobotSide())
            {
               // If the next step is with the other foot the step origin is the previous stance location.
               tempStepOrigin.set(tempStanceLocation);
               tempStanceLocation.set(stepPosition);
            }
            else
            {
               // If the next step is with the same foot the step origin is the previous step location.
               tempStepOrigin.set(stepPosition);
            }
         }
      }

      return true;
   }

   private static boolean checkPositionIsValid(FramePoint3DReadOnly positionToCheck,
                                               FramePoint3DReadOnly location1,
                                               FramePoint3DReadOnly location2,
                                               double maxDistance)
   {
      // Check the distance to a straight line on the ground from location1 to location 2
      double distanceXY = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(positionToCheck.getX(),
                                                                                 positionToCheck.getY(),
                                                                                 location1.getX(),
                                                                                 location1.getY(),
                                                                                 location2.getX(),
                                                                                 location2.getY());
      if (distanceXY > maxDistance)
      {
         LogTools.warn(
               "Got a footstep with a trajectory that is far from the step origin and goal location. The XY distance from a straight line trajectory was "
               + distanceXY + ". If that is acceptable increase the MaxSwingDistance parameter.");
         return false;
      }

      // Check the smaller distance in z from the locations
      double distanceZ = Math.min(Math.abs(location1.getZ() - positionToCheck.getZ()), Math.abs(location2.getZ() - positionToCheck.getZ()));
      if (distanceZ > maxDistance)
      {
         LogTools.warn(
               "Got a footstep with a trajectory that is far from the step origin and goal location. The Z distance from the closer location was " + distanceZ
               + ". If that is acceptable increase the MaxSwingDistance parameter.");
         return false;
      }

      return true;
   }

   private final FrameVector3D footstepOffsetVector = new FrameVector3D();
   private final FrameVector3D footstepUpdateVector = new FrameVector3D();

   public void updateFootTouchdownError(FrameVector3DReadOnly offset)
   {
      if (!offsettingXYPlanWithFootstepError.getValue() && !offsettingHeightPlanWithFootstepError.getValue())
      {
         return;
      }

      footstepOffsetVector.setIncludingFrame(offset);

      if (!offsettingXYPlanWithFootstepError.getValue())
      {
         footstepOffsetVector.setX(0.0);
         footstepOffsetVector.setY(0.0);
      }
      if (!offsettingHeightPlanWithFootstepError.getValue())
      {
         footstepOffsetVector.setZ(0.0);
      }

      footstepUpdateVector.setIncludingFrame(planOffsetInWorld);
      footstepUpdateVector.negate();
      this.planOffsetInWorld.add(planOffsetInWorldPrevious, footstepOffsetVector);
      footstepUpdateVector.add(planOffsetInWorld);

      for (int stepIdx = 0; stepIdx < upcomingFootsteps.size(); stepIdx++)
      {
         Footstep footstep = upcomingFootsteps.get(stepIdx);
         footstep.addOffset(footstepUpdateVector);
         StepConstraintRegionsList stepConstraints = upcomingStepConstraints.get(stepIdx);
         stepConstraints.addOffset(footstepUpdateVector);
      }

      setPlanOffsetInternal(planOffsetInWorld);

      updateVisualization();
   }

   public void commitToFootTouchdownError()
   {
      planOffsetInWorldPrevious.set(planOffsetInWorld);
   }

   private void setPlanOffsetInternal(FrameVector3DReadOnly planOffset)
   {
      comTrajectoryHandler.setPositionOffset(planOffset);
      planOffsetStatus.getOffsetVector().set(planOffset);
      statusOutputManager.reportStatusMessage(planOffsetStatus);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(footstepListVisualizer.getSCS2YoGraphics());
      return group;
   }
}
