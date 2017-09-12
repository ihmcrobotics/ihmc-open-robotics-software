package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AdjustFootstepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CenterOfMassTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.MomentumTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionTiming;
import us.ihmc.humanoidRobotics.communication.packets.momentum.TrajectoryPoint3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PlanOffsetStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingControllerFailureStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.lists.RecyclingArrayDeque;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class WalkingMessageHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // TODO Need to find something better than an ArrayList.
   private final List<Footstep> upcomingFootsteps = new ArrayList<>();
   private final List<FootstepTiming> upcomingFootstepTimings = new ArrayList<>();

   private final YoBoolean hasNewFootstepAdjustment = new YoBoolean("hasNewFootstepAdjustement", registry);
   private final AdjustFootstepCommand requestedFootstepAdjustment = new AdjustFootstepCommand();
   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;
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
   private final YoDouble defaultInitialTransferTime = new YoDouble("defaultInitialTransferTime", registry);

   private final YoBoolean isWalking = new YoBoolean("isWalking", registry);

   private final int numberOfFootstepsToVisualize = 4;
   @SuppressWarnings("unchecked")
   private final YoEnum<RobotSide>[] upcomingFoostepSide = new YoEnum[numberOfFootstepsToVisualize];

   private final FootstepListVisualizer footstepListVisualizer;

   private final YoDouble yoTime;
   private final YoDouble footstepDataListRecievedTime = new YoDouble("footstepDataListRecievedTime", registry);
   private final YoBoolean executingFootstep = new YoBoolean("ExecutingFootstep", registry);
   private final FootstepTiming lastTimingExecuted = new FootstepTiming();

   private final MomentumTrajectoryHandler momentumTrajectoryHandler;
   private final CenterOfMassTrajectoryHandler comTrajectoryHandler;

   private final YoBoolean offsettingPlanWithFootstepError = new YoBoolean("offsettingPlanWithFootstepError", registry);
   private final FrameVector3D planOffsetInWorld = new FrameVector3D(ReferenceFrame.getWorldFrame());

   public WalkingMessageHandler(double defaultTransferTime, double defaultSwingTime, double defaultInitialTransferTime, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         StatusMessageOutputManager statusOutputManager, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this(defaultTransferTime, defaultSwingTime, defaultInitialTransferTime, contactableFeet, statusOutputManager, null, yoGraphicsListRegistry, parentRegistry);
   }

   public WalkingMessageHandler(double defaultTransferTime, double defaultSwingTime, double defaultInitialTransferTime, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         StatusMessageOutputManager statusOutputManager, YoDouble yoTime, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.contactableFeet = contactableFeet;
      this.statusOutputManager = statusOutputManager;

      this.yoTime = yoTime;
      footstepDataListRecievedTime.setToNaN();

      this.defaultTransferTime.set(defaultTransferTime);
      this.finalTransferTime.set(defaultTransferTime);
      this.defaultSwingTime.set(defaultSwingTime);
      this.defaultInitialTransferTime.set(defaultInitialTransferTime);

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
         RigidBody endEffector = contactableFoot.getRigidBody();
         Footstep footstepAtCurrentLocation = new Footstep(endEffector, robotSide);
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
            upcomingFootsteps.clear();
            upcomingFootstepTimings.clear();
            currentFootstepIndex.set(0);
            clearFootTrajectory();
            currentNumberOfFootsteps.set(command.getNumberOfFootsteps());
            if (yoTime != null)
            {
               footstepDataListRecievedTime.set(yoTime.getDoubleValue());
            }
            break;
         case QUEUE:
            if (currentNumberOfFootsteps.getIntegerValue() < 1 && !executingFootstep.getBooleanValue())
            {
               PrintTools.warn("Can not queue footsteps if no footsteps are present. Send an override message instead. Command ignored.");
               return;
            }
            currentNumberOfFootsteps.add(command.getNumberOfFootsteps());
            break;
         default:
            PrintTools.warn(this, "Unknown " + ExecutionMode.class.getSimpleName() + " value: " + command.getExecutionMode() + ". Command ignored.");
            return;
         }
      }

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
         finalTransferTime.set(defaultTransferTime.getDoubleValue());

      boolean trustHeightOfFootsteps = command.isTrustHeightOfFootsteps();

      for (int i = 0; i < command.getNumberOfFootsteps(); i++)
      {
         FootstepTiming newFootstepTiming = createFootstepTiming(command.getFootstep(i), command.getExecutionTiming());
         upcomingFootstepTimings.add(newFootstepTiming);
         Footstep newFootstep = createFootstep(command.getFootstep(i), trustHeightOfFootsteps, newFootstepTiming.getSwingTime());
         upcomingFootsteps.add(newFootstep);
      }

      if (!checkTimings(upcomingFootstepTimings))
         clearFootsteps();
      updateTransferTimes(upcomingFootstepTimings);

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
         lastTimingExecuted.set(upcomingFootstepTimings.remove(0));
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

      FramePoint3D adjustedPosition = requestedFootstepAdjustment.getPosition();
      FrameOrientation adjustedOrientation = requestedFootstepAdjustment.getOrientation();
      footstepToAdjust.setPose(adjustedPosition, adjustedOrientation);

      if (!requestedFootstepAdjustment.getPredictedContactPoints().isEmpty())
      {
         List<Point2D> contactPoints = new ArrayList<>();
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

   private final Point3D desiredFootPositionInWorld = new Point3D();
   private final Quaternion desiredFootOrientationInWorld = new Quaternion();
   private final Point3D actualFootPositionInWorld = new Point3D();
   private final Quaternion actualFootOrientationInWorld = new Quaternion();
   private final TextToSpeechPacket reusableSpeechPacket = new TextToSpeechPacket();
   private final WalkingControllerFailureStatusMessage failureStatusMessage = new WalkingControllerFailureStatusMessage();

   public void reportFootstepStarted(RobotSide robotSide, FramePose desiredFootPoseInWorld, FramePose actualFootPoseInWorld)
   {
      desiredFootPoseInWorld.getPose(desiredFootPositionInWorld, desiredFootOrientationInWorld);
      actualFootPoseInWorld.getPose(actualFootPositionInWorld, actualFootOrientationInWorld);
      statusOutputManager.reportStatusMessage(new FootstepStatus(FootstepStatus.Status.STARTED, currentFootstepIndex.getIntegerValue(),
            desiredFootPositionInWorld, desiredFootOrientationInWorld,
            actualFootPositionInWorld, actualFootOrientationInWorld, robotSide));
      executingFootstep.set(true);
   }

   public void reportFootstepCompleted(RobotSide robotSide, FramePose actualFootPoseInWorld)
   {
      actualFootPoseInWorld.getPose(actualFootPositionInWorld, actualFootOrientationInWorld);
      statusOutputManager.reportStatusMessage(new FootstepStatus(FootstepStatus.Status.COMPLETED, currentFootstepIndex.getIntegerValue(),
            actualFootPositionInWorld, actualFootOrientationInWorld, robotSide));
//      reusableSpeechPacket.setTextToSpeak(TextToSpeechPacket.FOOTSTEP_COMPLETED);
//      statusOutputManager.reportStatusMessage(reusableSpeechPacket);
      executingFootstep.set(false);
   }

   public void reportWalkingStarted()
   {
      WalkingStatusMessage walkingStatusMessage = new WalkingStatusMessage();
      walkingStatusMessage.setWalkingStatus(WalkingStatusMessage.Status.STARTED);
      statusOutputManager.reportStatusMessage(walkingStatusMessage);
      reusableSpeechPacket.setTextToSpeak(TextToSpeechPacket.WALKING);
      statusOutputManager.reportStatusMessage(reusableSpeechPacket);
      isWalking.set(true);
   }

   public void reportWalkingComplete()
   {
      WalkingStatusMessage walkingStatusMessage = new WalkingStatusMessage();
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

   private Footstep createFootstep(FootstepDataCommand footstepData, boolean trustHeight, double swingTime)
   {
      FramePose footstepPose = new FramePose(footstepData.getPosition(), footstepData.getOrientation());

      List<Point2D> contactPoints;
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
      RigidBody rigidBody = contactableFoot.getRigidBody();

      Footstep footstep = new Footstep(rigidBody, robotSide, footstepPose, trustHeight, contactPoints);

      if (trajectoryType == TrajectoryType.CUSTOM)
      {
         if (footstepData.getCustomPositionWaypoints() == null)
         {
            PrintTools.warn("Can not request custom trajectory without specifying waypoints. Using default trajectory.");
            trajectoryType = TrajectoryType.DEFAULT;
         }
         else
         {
            RecyclingArrayList<FramePoint3D> positionWaypoints = footstepData.getCustomPositionWaypoints();
            footstep.setCustomPositionWaypoints(positionWaypoints);
         }
      }
      if (trajectoryType == TrajectoryType.WAYPOINTS)
      {
         RecyclingArrayList<FrameSE3TrajectoryPoint> swingTrajectory = footstepData.getSwingTrajectory();
         if (swingTrajectory == null)
         {
            PrintTools.warn("Can not request custom trajectory without specifying waypoints. Using default trajectory.");
            trajectoryType = TrajectoryType.DEFAULT;
         }
         if (swingTrajectory.getLast().getTime() > swingTime)
         {
            PrintTools.warn("Last waypoint in custom trajectory has time greater then the swing time. Using default trajectory.");
            trajectoryType = TrajectoryType.DEFAULT;
         }
         else
         {
            footstep.setSwingTrajectory(swingTrajectory);
         }
      }

      footstep.setTrajectoryType(trajectoryType);
      footstep.setSwingHeight(footstepData.getSwingHeight());
      footstep.setSwingTrajectoryBlendDuration(footstepData.getSwingTrajectoryBlendDuration());
      if (offsettingPlanWithFootstepError.getBooleanValue())
      {
         footstep.addOffset(planOffsetInWorld);
      }
      return footstep;
   }

   private FootstepTiming createFootstepTiming(FootstepDataCommand footstep, ExecutionTiming executionTiming)
   {
      FootstepTiming timing = new FootstepTiming();

      double swingDuration = footstep.getSwingDuration();
      if (Double.isNaN(swingDuration) || swingDuration <= 0.0)
         swingDuration = defaultSwingTime.getDoubleValue();

      double transferDuration = footstep.getTransferDuration();
      if (Double.isNaN(transferDuration) || transferDuration <= 0.0)
      {
         if (upcomingFootstepTimings.isEmpty() && !isWalking.getBooleanValue())
            transferDuration = defaultInitialTransferTime.getDoubleValue();
         else
            transferDuration = defaultTransferTime.getDoubleValue();
      }

      timing.setTimings(swingDuration, transferDuration);

      switch (executionTiming)
      {
      case CONTROL_DURATIONS:
         break;
      case CONTROL_ABSOLUTE_TIMINGS:
         int stepsInQueue = upcomingFootstepTimings.size();
         if (stepsInQueue == 0 && !executingFootstep.getBooleanValue())
         {
            timing.setAbsoluteTime(transferDuration, footstepDataListRecievedTime.getDoubleValue());
         }
         else if (stepsInQueue == 0)
         {
            double swingStartTime = lastTimingExecuted.getSwingStartTime() + lastTimingExecuted.getSwingTime() + transferDuration;
            timing.setAbsoluteTime(swingStartTime, footstepDataListRecievedTime.getDoubleValue());
         }
         else
         {
            FootstepTiming previousTiming = upcomingFootstepTimings.get(stepsInQueue - 1);
            double swingStartTime = previousTiming.getSwingStartTime() + previousTiming.getSwingTime() + transferDuration;
            timing.setAbsoluteTime(swingStartTime, footstepDataListRecievedTime.getDoubleValue());
         }
         break;
      default:
         throw new RuntimeException("Timing mode not implemented.");
      }

      return timing;
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
      firstTiming.setTimings(lastSwingTime, lastSwingStart);

      for (int footstepIdx = 1; footstepIdx < upcomingFootstepTimings.size(); footstepIdx++)
      {
         FootstepTiming timing = upcomingFootstepTimings.get(footstepIdx);
         double swingStart = timing.getSwingStartTime();
         double swingTime = timing.getSwingTime();
         double transferTime = swingStart - (lastSwingStart + lastSwingTime);
         timing.setTimings(swingTime, transferTime);

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
      statusOutputManager.reportStatusMessage(new PlanOffsetStatus(planOffsetInWorld.getVector()));
   }
}
