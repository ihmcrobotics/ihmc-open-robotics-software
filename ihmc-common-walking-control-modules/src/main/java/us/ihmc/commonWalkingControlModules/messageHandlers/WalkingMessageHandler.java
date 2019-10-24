package us.ihmc.commonWalkingControlModules.messageHandlers;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableDouble;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PlanOffsetStatus;
import controller_msgs.msg.dds.TextToSpeechPacket;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AdjustFootstepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CenterOfMassTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.MomentumTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.trajectories.trajectorypoints.EuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

public class WalkingMessageHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final int maxNumberOfFootsteps = 100;
   private final RecyclingArrayList<Footstep> upcomingFootsteps = new RecyclingArrayList<>(maxNumberOfFootsteps, Footstep.class);
   private final RecyclingArrayList<FootstepTiming> upcomingFootstepTimings = new RecyclingArrayList<>(maxNumberOfFootsteps, FootstepTiming.class);
   private final RecyclingArrayList<FootstepShiftFractions> upcomingFootstepShiftFractions = new RecyclingArrayList<>(maxNumberOfFootsteps, FootstepShiftFractions.class);
   private final RecyclingArrayList<MutableDouble> pauseDurationAfterStep = new RecyclingArrayList<>(maxNumberOfFootsteps, MutableDouble.class);

   private final YoBoolean isPausedWithSteps = new YoBoolean("IsPausedWithSteps", registry);
   private final YoDouble timeToContinueWalking = new YoDouble("TimeToContinueWalking", registry);
   private final DoubleProvider minimumPauseTime = new DoubleParameter("MinimumPauseTime", registry, 0.0);

   private final YoBoolean hasNewFootstepAdjustment = new YoBoolean("hasNewFootstepAdjustement", registry);
   private final AdjustFootstepCommand requestedFootstepAdjustment = new AdjustFootstepCommand();
   private final SideDependentList<Footstep> footstepsAtCurrentLocation = new SideDependentList<>();
   private final SideDependentList<Footstep> lastDesiredFootsteps = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();

   private final SideDependentList<RecyclingArrayDeque<FootTrajectoryCommand>> upcomingFootTrajectoryCommandListForFlamingoStance = new SideDependentList<>();

   private final StatusMessageOutputManager statusOutputManager;
   private final PlanOffsetStatus planOffsetStatus = new PlanOffsetStatus();

   private final YoInteger currentFootstepIndex = new YoInteger("currentFootstepIndex", registry);
   private final YoInteger currentNumberOfFootsteps = new YoInteger("currentNumberOfFootsteps", registry);
   private final YoBoolean isWalkingPaused = new YoBoolean("isWalkingPaused", registry);
   private final YoBoolean isWalkingResuming = new YoBoolean("isWalkingResuming", registry);
   private final YoDouble defaultTransferTime = new YoDouble("defaultTransferTime", registry);
   private final YoDouble finalTransferTime = new YoDouble("finalTransferTime", registry);
   private final YoDouble defaultSwingTime = new YoDouble("defaultSwingTime", registry);
   private final YoDouble defaultInitialTransferTime = new YoDouble("defaultInitialTransferTime", registry);

   private final YoDouble defaultSwingSplitFraction = new YoDouble("defaultSwingSplitFraction", registry);
   private final YoDouble defaultSwingDurationShiftFraction = new YoDouble("defaultSwingDurationShiftFraction", registry);
   private final YoDouble defaultTransferSplitFraction = new YoDouble("defaultTransferSplitFraction", registry);
   private final YoDouble defaultFinalTransferSplitFraction = new YoDouble("defaultFinalTransferSplitFraction", registry);
   private final YoDouble finalTransferSplitFraction = new YoDouble("finalTransferSplitFraction", registry);

   private final YoDouble defaultTransferWeightDistribution = new YoDouble("defaultTransferWeightDistribution", registry);
   private final YoDouble defaultFinalTransferWeightDistribution = new YoDouble("defaultFinalTransferWeightDistribution", registry);
   private final YoDouble finalTransferWeightDistribution = new YoDouble("finalTransferWeightDistribution", registry);

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

   private final YoBoolean offsettingXYPlanWithFootstepError = new YoBoolean("offsettingXYPlanWithFootstepError", registry);
   private final YoBoolean offsettingHeightPlanWithFootstepError = new YoBoolean("offsettingHeightPlanWithFootstepError", registry);

   private final YoFrameVector3D planOffsetInWorld = new YoFrameVector3D("planOffsetInWorld", worldFrame, registry);
   private final YoFrameVector3D planOffsetFromAdjustment = new YoFrameVector3D("comPlanOffsetFromAdjustment", worldFrame, registry);

   private final DoubleProvider maxStepDistance = new DoubleParameter("MaxStepDistance", registry, Double.POSITIVE_INFINITY);
   private final DoubleProvider maxStepHeightChange = new DoubleParameter("MaxStepHeightChange", registry, Double.POSITIVE_INFINITY);
   private final DoubleProvider maxSwingDistance = new DoubleParameter("MaxSwingDistance", registry, Double.POSITIVE_INFINITY);

   public WalkingMessageHandler(double defaultTransferTime, double defaultSwingTime, double defaultInitialTransferTime, double defaultFinalTransferTime,
                                double defaultSwingDurationShiftFraction, double defaultSwingSplitFraction, double defaultTransferSplitFraction,
                                double defaultFinalTransferSplitFraction, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                StatusMessageOutputManager statusOutputManager, YoDouble yoTime, YoGraphicsListRegistry yoGraphicsListRegistry,
                                YoVariableRegistry parentRegistry)
   {
      this.statusOutputManager = statusOutputManager;

      this.yoTime = yoTime;
      footstepDataListReceivedTime.setToNaN();

      this.defaultTransferTime.set(defaultTransferTime);
      this.defaultSwingTime.set(defaultSwingTime);
      this.defaultInitialTransferTime.set(defaultInitialTransferTime);
      this.defaultFinalTransferTime.set(defaultFinalTransferTime);
      this.finalTransferTime.set(defaultFinalTransferTime);

      this.defaultSwingSplitFraction.set(defaultSwingSplitFraction);
      this.defaultSwingDurationShiftFraction.set(defaultSwingDurationShiftFraction);
      this.defaultTransferSplitFraction.set(defaultTransferSplitFraction);
      this.defaultFinalTransferSplitFraction.set(defaultFinalTransferSplitFraction);
      this.finalTransferSplitFraction.set(defaultFinalTransferSplitFraction);

      defaultTransferWeightDistribution.set(0.5);
      defaultFinalTransferWeightDistribution.set(0.5);
      finalTransferWeightDistribution.set(defaultFinalTransferWeightDistribution.getDoubleValue());

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
         Footstep footstepAtCurrentLocation = new Footstep(robotSide);
         footstepsAtCurrentLocation.put(robotSide, footstepAtCurrentLocation);
         soleFrames.put(robotSide, contactableFoot.getSoleFrame());

         upcomingFootTrajectoryCommandListForFlamingoStance.put(robotSide, new RecyclingArrayDeque<>(FootTrajectoryCommand.class, FootTrajectoryCommand::set));
      }

      for (int i = 0; i < numberOfFootstepsToVisualize; i++)
         upcomingFoostepSide[i] = new YoEnum<>("upcomingFoostepSide" + i, registry, RobotSide.class, true);

      if (yoGraphicsListRegistry != null)
         footstepListVisualizer = new FootstepListVisualizer(contactableFeet, yoGraphicsListRegistry, registry);
      else
         footstepListVisualizer = null;
      updateVisualization();

      momentumTrajectoryHandler = new MomentumTrajectoryHandler(yoTime, registry);
      comTrajectoryHandler = new CenterOfMassTrajectoryHandler(yoTime, registry);
      planarRegionsListHandler = new PlanarRegionsListHandler(statusOutputManager, registry);

      parentRegistry.addChild(registry);
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
            clearFootsteps();
            clearFootTrajectory();
            break;
         case QUEUE:
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
               else
               {
                  LogTools.warn("Can not queue footsteps if no footsteps are present. Send an override message instead. Command ignored.");
               }
               return;
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
          * The walking was paused, when paused isWalking remains true. We're
          * receiving a new series of footsteps, let's reset isWalking so the
          * controller reports that it starts walking.
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

      double commandDefaultSwingSplitFraction = command.getDefaultSwingSplitFraction();
      double commandDefaultSwingDurationShiftFraction = command.getDefaultSwingDurationShiftFraction();
      double commandDefaultTransferSplitFraction = command.getDefaultTransferSplitFraction();
      if (!Double.isNaN(commandDefaultSwingSplitFraction) && MathTools.intervalContains(commandDefaultSwingSplitFraction, 0.0, 1.0, false, false))
         defaultSwingSplitFraction.set(commandDefaultSwingSplitFraction);
      if (!Double.isNaN(commandDefaultSwingDurationShiftFraction) && MathTools.intervalContains(commandDefaultSwingDurationShiftFraction, 0.0, 1.0, false, false))
         defaultSwingDurationShiftFraction.set(commandDefaultSwingDurationShiftFraction);
      if (!Double.isNaN(commandDefaultTransferSplitFraction) && MathTools.intervalContains(commandDefaultTransferSplitFraction, 0.0, 1.0, false, false))
         defaultTransferSplitFraction.set(commandDefaultTransferSplitFraction);

      double commandFinalTransferSplitFraction = command.getFinalTransferSplitFraction();

      if (commandFinalTransferSplitFraction > 0.0)
         finalTransferSplitFraction.set(commandFinalTransferSplitFraction);
      else
         finalTransferSplitFraction.set(defaultFinalTransferSplitFraction.getDoubleValue());


      double commandDefaultTransferWeightDistribution = command.getDefaultTransferWeightDistribution();
      if (!Double.isNaN(commandDefaultTransferWeightDistribution) && MathTools.intervalContains(commandDefaultTransferWeightDistribution, 0.0, 1.0, false, false))
         defaultTransferWeightDistribution.set(commandDefaultSwingDurationShiftFraction);
      double commandFinalTransferWeightDistribution = command.getFinalTransferWeightDistribution();
      if (!Double.isNaN(commandFinalTransferWeightDistribution) && MathTools.intervalContains(commandFinalTransferSplitFraction, 0.0, 1.0, false, false))
         finalTransferWeightDistribution.set(commandFinalTransferWeightDistribution);
      else
         finalTransferWeightDistribution.set(defaultFinalTransferWeightDistribution.getDoubleValue());

      boolean trustHeightOfFootsteps = command.isTrustHeightOfFootsteps();
      boolean areFootstepsAdjustable = command.areFootstepsAdjustable();

      for (int i = 0; i < command.getNumberOfFootsteps(); i++)
      {
         setFootstepTiming(command.getFootstep(i), command.getExecutionTiming(), upcomingFootstepTimings.add(), pauseDurationAfterStep.add(),
                           command.getExecutionMode());
         setFootstepShiftFractions(command.getFootstep(i), upcomingFootstepShiftFractions.add());
         setFootstep(command.getFootstep(i), trustHeightOfFootsteps, areFootstepsAdjustable, upcomingFootsteps.add());
         currentNumberOfFootsteps.increment();
      }

      if (!checkTimings(upcomingFootstepTimings, yoTime))
      {
         clearFootsteps();
      }

      if (!checkFootsteps(upcomingFootsteps, soleFrames, maxStepDistance.getValue(), maxStepHeightChange.getValue(), maxSwingDistance.getValue(), tempStanceLocation, tempStepOrigin))
      {
         clearFootsteps();
      }

      checkForPause();

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
         LogTools.warn("Received " + AdjustFootstepCommand.class.getSimpleName() + " but walking is currently paused. Command ignored.");
         requestedFootstepAdjustment.clear();
         hasNewFootstepAdjustment.set(false);
         return;
      }

      requestedFootstepAdjustment.set(command);
      hasNewFootstepAdjustment.set(true);
   }

   public void handlePauseWalkingCommand(PauseWalkingCommand command)
   {
      if (!command.isPauseRequested() && isWalkingPaused.getValue())
         isWalkingResuming.set(true);

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
   public void getAngularMomentumTrajectory(double startTime, double endTime, int numberOfPoints,
                                            RecyclingArrayList<EuclideanTrajectoryPoint> trajectoryToPack)
   {
      momentumTrajectoryHandler.getAngularMomentumTrajectory(startTime, endTime, numberOfPoints, trajectoryToPack);
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
    * This method will set the provided shift fraction to the shift fraction of the {@code i}'th upcoming
    * footstep. If there is less then {@code i} upcoming steps this method will throw a
    * {@link RuntimeException}. To check how many footsteps are upcoming use
    * {@link #getCurrentNumberOfFootsteps()}.
    *
    * @param i is the index of the upcoming footstep shift fraction that will be packed.
    * @param shiftFractionToPack will be set to the shift fraction of footstep i in the list of upcoming steps.
    */
   public void peekShiftFraction(int i, FootstepShiftFractions shiftFractionToPack)
   {
      if (i >= upcomingFootstepShiftFractions.size())
      {
         throw new RuntimeException("Can not get shift fraction " + i + " since there are only " + upcomingFootstepShiftFractions.size() + " upcoming shift fractions.");
      }
      shiftFractionToPack.set(upcomingFootstepShiftFractions.get(i));
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
   public void poll(Footstep footstepToPack, FootstepTiming timingToPack, FootstepShiftFractions shiftFractions)
   {
      if (getStepsBeforePause() == 0)
      {
         throw new RuntimeException("Can not poll footstep since there are no upcoming steps.");
      }

      footstepToPack.set(upcomingFootsteps.get(0));
      timingToPack.set(upcomingFootstepTimings.get(0));
      shiftFractions.set(upcomingFootstepShiftFractions.get(0));
      lastTimingExecuted.set(upcomingFootstepTimings.get(0));

      updateVisualization();
      currentNumberOfFootsteps.decrement();
      currentFootstepIndex.increment();

      upcomingFootstepTimings.remove(0);
      upcomingFootstepShiftFractions.remove(0);
      pauseDurationAfterStep.remove(0);
      upcomingFootsteps.remove(0);
   }

   /**
    * This method can be used to adjust the timing of the upcoming footstep. It will throw a {@link RuntimeException} if
    * there are no footsteps in the queue.
    *
    * @param newSwingDuration is the new swing duration for the adjusted timing
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

   public boolean pollRequestedFootstepAdjustment(Footstep footstepToAdjust)
   {
      if (!hasNewFootstepAdjustment.getBooleanValue())
         return false;

      if (footstepToAdjust.getRobotSide() != requestedFootstepAdjustment.getRobotSide())
      {
         LogTools.warn("RobotSide does not match: side of footstep to be adjusted: " + footstepToAdjust.getRobotSide() + ", side of adjusted footstep: "
               + requestedFootstepAdjustment.getRobotSide());
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

      if (isPausedWithSteps.getValue())
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
      pauseDurationAfterStep.clear();
      currentNumberOfFootsteps.set(0);
      currentFootstepIndex.set(0);
      updateVisualization();
   }

   private final TextToSpeechPacket reusableSpeechPacket = new TextToSpeechPacket();
   private final WalkingControllerFailureStatusMessage failureStatusMessage = new WalkingControllerFailureStatusMessage();
   private final FootstepStatusMessage footstepStatus = new FootstepStatusMessage();

   public void reportFootstepStarted(RobotSide robotSide, FramePose3DReadOnly desiredFootPoseInWorld, FramePose3DReadOnly actualFootPoseInWorld)
   {
      reportFootstepStatus(robotSide, FootstepStatus.STARTED, desiredFootPoseInWorld, actualFootPoseInWorld);
      executingFootstep.set(true);

      if (yoTime != null)
         timeElapsedWhenFootstepExecuted.set(yoTime.getDoubleValue() - footstepDataListReceivedTime.getDoubleValue());
   }

   public void reportFootstepCompleted(RobotSide robotSide, FramePose3DReadOnly desiredFootPoseInWorld, FramePose3DReadOnly actualFootPoseInWorld)
   {
      reportFootstepStatus(robotSide, FootstepStatus.COMPLETED, desiredFootPoseInWorld, actualFootPoseInWorld);
      executingFootstep.set(false);
   }

   private void reportFootstepStatus(RobotSide robotSide, FootstepStatus status, FramePose3DReadOnly desiredFootPoseInWorld,
                                     FramePose3DReadOnly actualFootPoseInWorld)
   {
      desiredFootPoseInWorld.checkReferenceFrameMatch(worldFrame);
      actualFootPoseInWorld.checkReferenceFrameMatch(worldFrame);

      footstepStatus.setFootstepStatus(status.toByte());
      footstepStatus.setRobotSide(robotSide.toByte());
      footstepStatus.setFootstepIndex(currentFootstepIndex.getIntegerValue());
      footstepStatus.getActualFootOrientationInWorld().set(actualFootPoseInWorld.getOrientation());
      footstepStatus.getActualFootPositionInWorld().set(actualFootPoseInWorld.getPosition());
      footstepStatus.getDesiredFootOrientationInWorld().set(desiredFootPoseInWorld.getOrientation());
      footstepStatus.getDesiredFootPositionInWorld().set(desiredFootPoseInWorld.getPosition());
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

   public double getInitialTransferTime()
   {
      return defaultInitialTransferTime.getDoubleValue();
   }

   public double getFinalTransferTime()
   {
      return finalTransferTime.getDoubleValue();
   }

   public double getDefaultSwingDurationShiftFraction()
   {
      return defaultSwingDurationShiftFraction.getDoubleValue();
   }

   public double getDefaultSwingSplitFraction()
   {
      return defaultSwingSplitFraction.getDoubleValue();
   }

   public double getDefaultTransferSplitFraction()
   {
      return defaultTransferSplitFraction.getDoubleValue();
   }

   public double getFinalTransferSplitFraction()
   {
      return finalTransferSplitFraction.getDoubleValue();
   }

   public double getFinalTransferWeightDistribution()
   {
      return finalTransferWeightDistribution.getDoubleValue();
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

   private void setFootstep(FootstepDataCommand footstepData, boolean trustHeight, boolean isAdjustable, Footstep footstepToSet)
   {
      footstepToSet.set(footstepData, trustHeight, isAdjustable);
      footstepToSet.addOffset(planOffsetInWorld);
   }

   private void setFootstepShiftFractions(FootstepDataCommand footstepData, FootstepShiftFractions shiftFractionsToSet)
   {
      double swingDurationShiftFraction = footstepData.getSwingDurationShiftFraction();
      double swingSplitFraction = footstepData.getSwingSplitFraction();
      double transferSplitFraction = footstepData.getTransferSplitFraction();

      if (Double.isNaN(transferSplitFraction) || !MathTools.intervalContains(transferSplitFraction, 0.0, 1.0, false, false))
         transferSplitFraction = defaultTransferSplitFraction.getDoubleValue();

      if (Double.isNaN(swingSplitFraction) || !MathTools.intervalContains(swingSplitFraction, 0.0, 1.0, false, false))
         swingSplitFraction = defaultSwingSplitFraction.getDoubleValue();

      if (Double.isNaN(swingDurationShiftFraction) || !MathTools.intervalContains(swingDurationShiftFraction, 0.0, 1.0, false, false))
         swingDurationShiftFraction = defaultSwingDurationShiftFraction.getDoubleValue();

      shiftFractionsToSet.setShiftFractions(swingDurationShiftFraction, swingSplitFraction, transferSplitFraction);

      double transferWeightDistribution = footstepData.getTransferWeightDistribution();

      if (Double.isNaN(transferWeightDistribution) || !MathTools.intervalContains(transferWeightDistribution, 0.0, 1.0))
         transferWeightDistribution = defaultTransferWeightDistribution.getDoubleValue();

      shiftFractionsToSet.setTransferWeightDistribution(transferWeightDistribution);
   }

   private void setFootstepTiming(FootstepDataCommand footstep, ExecutionTiming executionTiming, FootstepTiming timingToSet, MutableDouble pauseDurationToSet,
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
         if (stepsInQueue == 0 && !isWalking.getBooleanValue() && executionMode == ExecutionMode.OVERRIDE)
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
    *  - a step that is too far from the stance foot in XY</br>
    *  - a step that has a swing trajectory that is too far from a straight line swing in XY</br>
    *  - a step that has a swing trajectory that is too far from the step origin or destination in Z</br>
    *
    * @return if the footsteps were found to be safe
    */
   private static boolean checkFootsteps(List<Footstep> footsteps, SideDependentList<ReferenceFrame> soleFrames, double maxStepDistance,
                                         double maxStepHeightChange, double maxSwingDistance, FramePoint3DBasics tempStanceLocation,
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

   private static boolean checkPositionIsValid(FramePoint3DReadOnly positionToCheck, FramePoint3DReadOnly location1, FramePoint3DReadOnly location2,
                                               double maxDistance)
   {
      // Check the distance to a straight line on the ground from location1 to location 2
      double distanceXY = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(positionToCheck.getX(), positionToCheck.getY(), location1.getX(),
                                                                                 location1.getY(), location2.getX(), location2.getY());
      if (distanceXY > maxDistance)
      {
         LogTools.warn("Got a footstep with a trajectory that is far from the step origin and goal location. The XY distance from a straight line trajectory was "
               + distanceXY + ". If that is acceptable increase the MaxSwingDistance parameter.");
         return false;
      }

      // Check the smaller distance in z from the locations
      double distanceZ = Math.min(Math.abs(location1.getZ() - positionToCheck.getZ()), Math.abs(location2.getZ() - positionToCheck.getZ()));
      if (distanceZ > maxDistance)
      {
         LogTools.warn("Got a footstep with a trajectory that is far from the step origin and goal location. The Z distance from the closer location was "
               + distanceZ + ". If that is acceptable increase the MaxSwingDistance parameter.");
         return false;
      }

      return true;
   }

   private final FrameVector3D footstepOffsetVector = new FrameVector3D();

   public void addOffsetVectorOnTouchdown(FrameVector3DReadOnly offset)
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

      for (int stepIdx = 0; stepIdx < upcomingFootsteps.size(); stepIdx++)
      {
         Footstep footstep = upcomingFootsteps.get(stepIdx);
         footstep.addOffset(footstepOffsetVector);
      }

      this.planOffsetInWorld.add(footstepOffsetVector);
      setPlanOffsetInternal(planOffsetInWorld);

      planOffsetFromAdjustment.setToZero();

      updateVisualization();
   }

   private final FrameVector3D totalOffset = new FrameVector3D();

   public void setPlanOffsetFromAdjustment(FrameVector3DReadOnly planOffsetFromAdjustment)
   {
      this.planOffsetFromAdjustment.set(planOffsetFromAdjustment);
      totalOffset.add(planOffsetInWorld, planOffsetFromAdjustment);
      setPlanOffsetInternal(totalOffset);
   }

   private void setPlanOffsetInternal(FrameVector3DReadOnly planOffset)
   {
      comTrajectoryHandler.setPositionOffset(planOffset);
      planOffsetStatus.getOffsetVector().set(planOffset);
      statusOutputManager.reportStatusMessage(planOffsetStatus);

   }
}
