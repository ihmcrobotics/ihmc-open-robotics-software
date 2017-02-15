package us.ihmc.humanoidBehaviors.behaviors.planFootstepsToLocation;

import com.github.quickhull3d.Point3d;
import com.github.quickhull3d.Vector3d;
import org.opencv.core.Mat;
import us.ihmc.communication.packets.*;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.tools.time.Timer;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;

public class FootStepPlannerToLocationBehavior extends AbstractBehavior
{

   private final long fiducialToTrack;
   private final String prefix = "toLocation";

   private final HumanoidReferenceFrames referenceFrames;

   private final FramePose tempStanceFootPose = new FramePose();
   private final FramePose tempLeftFootPose = new FramePose();
   private final FramePose tempRightFootPose = new FramePose();

   private final ConcurrentListeningQueue<RobotConfigurationData> robotConfigurationDataQueue;
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue;
   private final ConcurrentListeningQueue<WalkingStatusMessage> walkingStatusQueue;
   private final ConcurrentListeningQueue<PlanarRegionsListMessage> planarRegionsListQueue = new ConcurrentListeningQueue<>(10);

   private final SideDependentList<FootstepStatus> latestFootstepStatus;
   private final SideDependentList<EnumYoVariable<FootstepStatus.Status>> latestFootstepStatusEnum;
   private final SideDependentList<YoFramePose> actualFootStatusPoses;
   private final SideDependentList<YoFramePose> desiredFootStatusPoses;

   private final EnumYoVariable <RobotSide> nextSideToSwing;
   private final EnumYoVariable<RobotSide> currentlySwingingFoot;

   private final IntegerYoVariable planarRegionsListCount = new IntegerYoVariable(prefix + "PlanarRegionsListCount", registry);
   private final DoubleYoVariable headPitchToFindFucdicial = new DoubleYoVariable(prefix + "HeadPitchToFindFucdicial", registry);

   private final YoFramePose footstepPlannerInitialStancePose;

   private final Timer footstepSentTimer;

   private final FramePose tempFirstFootstepPose = new FramePose();
   private final javax.vecmath.Point3d tempFootstepPosePosition = new javax.vecmath.Point3d();
   private final Quat4d tempFirstFootstepPoseOrientation = new Quat4d();

   private FootstepPlanner planner;
   private final FramePose tempFootstepPlannerGoalPose = new FramePose();

   private final FootstepPlannerGoal footstepPlannerGoal;
   private FootstepPlanningResult footstepPlanningResult;

   private final FiducialDetectorBehaviorService fiducialDetectorBehaviorService;

   private final DoubleYoVariable swingTime = new DoubleYoVariable(prefix + "SwingTime", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable(prefix + "TransferTime", registry);

   public FootStepPlannerToLocationBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames, FiducialDetectorBehaviorService fiducialDetectorBehaviorService, long fiducialToTrack)
   {
      super(communicationBridge);

      this.referenceFrames = referenceFrames;
      this.planner = createFootstepPlanner();
      this.fiducialDetectorBehaviorService = fiducialDetectorBehaviorService;
      this.fiducialToTrack = fiducialToTrack;
      fiducialDetectorBehaviorService.setTargetIDToLocate(this.fiducialToTrack);

      footstepPlannerGoal = new FootstepPlannerGoal();

      headPitchToFindFucdicial.set(1.0);

      nextSideToSwing = new EnumYoVariable<RobotSide>(prefix + "nextSideToSwing", registry, RobotSide.class, true);
      currentlySwingingFoot = new EnumYoVariable<RobotSide>(prefix + "currentlySwingingFoot", registry, RobotSide.class, true);
      footstepPlannerInitialStancePose = new YoFramePose(prefix + "footstepPlannerInitialStancePose", ReferenceFrame.getWorldFrame(), registry);

      footstepSentTimer = new Timer();
      footstepSentTimer.start();

      latestFootstepStatus = new SideDependentList<>();
      EnumYoVariable<FootstepStatus.Status> leftFootstepStatus = new EnumYoVariable<FootstepStatus.Status>(prefix + "leftFootstepStatus", registry, FootstepStatus.Status.class);
      EnumYoVariable<FootstepStatus.Status> rightFootstepStatus = new EnumYoVariable<FootstepStatus.Status>(prefix + "rightFootstepStatus", registry, FootstepStatus.Status.class);
      latestFootstepStatusEnum = new SideDependentList<>(leftFootstepStatus, rightFootstepStatus);

      YoFramePose desiredLeftFootstepStatusPose = new YoFramePose(prefix + "DesiredLeftFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      YoFramePose desiredRightFootstepStatusPose = new YoFramePose(prefix + "DesiredRightFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      desiredFootStatusPoses = new SideDependentList<>(desiredLeftFootstepStatusPose, desiredRightFootstepStatusPose);

      YoFramePose leftFootstepStatusPose = new YoFramePose(prefix + "LeftFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      YoFramePose rightFootstepStatusPose = new YoFramePose(prefix + "RightFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      actualFootStatusPoses = new SideDependentList<>(leftFootstepStatusPose, rightFootstepStatusPose);

      footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>(40);
      robotConfigurationDataQueue = new ConcurrentListeningQueue<RobotConfigurationData>(40);
      walkingStatusQueue = new ConcurrentListeningQueue<WalkingStatusMessage>(10);
      attachNetworkListeningQueue(robotConfigurationDataQueue, RobotConfigurationData.class);
      attachNetworkListeningQueue(footstepStatusQueue, FootstepStatus.class);
      attachNetworkListeningQueue(walkingStatusQueue, WalkingStatusMessage.class);
      attachNetworkListeningQueue(planarRegionsListQueue, PlanarRegionsListMessage.class);
      
      swingTime.set(1.0);
      transferTime.set(0.3);
   }

   private void sendFootstepDataListMessage(FootstepDataListMessage footstepDataListMessage)
   {
      footstepDataListMessage.setDestination(PacketDestination.UI);
      sendPacket(footstepDataListMessage);

      footstepDataListMessage.setDestination(PacketDestination.CONTROLLER);
      sendPacketToController(footstepDataListMessage);
      footstepSentTimer.reset();
   }

   private void setGoalAndInitialFootClosestToGoal(FramePose goalPose)
   {
      tempLeftFootPose.setToZero(referenceFrames.getFootFrame(RobotSide.LEFT));
      tempRightFootPose.setToZero(referenceFrames.getFootFrame(RobotSide.RIGHT));
      tempLeftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      tempRightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      javax.vecmath.Point3d temp = new javax.vecmath.Point3d();
      javax.vecmath.Point3d pointBetweenFeet = new javax.vecmath.Point3d();
      javax.vecmath.Vector3d vectorFromFeetToGoal = new javax.vecmath.Vector3d();

      tempLeftFootPose.getPosition(temp);
      pointBetweenFeet.set(temp);
      tempLeftFootPose.getPosition(temp);
      pointBetweenFeet.add(temp);
      pointBetweenFeet.scale(0.5);

      goalPose.getPosition(vectorFromFeetToGoal);
      vectorFromFeetToGoal.sub(pointBetweenFeet);

      double headingFromFeetToGoal = Math.atan2(vectorFromFeetToGoal.getY(), vectorFromFeetToGoal.getX());
      AxisAngle4d goalOrientation = new AxisAngle4d(0.0, 0.0, 1.0, headingFromFeetToGoal);
      goalPose.setOrientation(goalOrientation);

      RobotSide stanceSide = nextSideToSwing.getEnumValue().getOppositeSide();

      if (stanceSide == RobotSide.LEFT)
      {
         tempStanceFootPose.set(tempLeftFootPose);
         goalPose.setZ(tempLeftFootPose.getZ());
      }
      else
      {
         tempStanceFootPose.set(tempRightFootPose);
         goalPose.setZ(tempRightFootPose.getZ());
      }

      sendTextToSpeechPacket("Planning footsteps from " + tempStanceFootPose + " to " + goalPose);
      footstepPlannerGoal.setGoalPoseBetweenFeet(goalPose);
      planner.setGoal(footstepPlannerGoal);

      planner.setInitialStanceFoot(tempStanceFootPose, stanceSide);
      footstepPlannerInitialStancePose.set(tempStanceFootPose);
   }

   private void checkFootstepStatusAndDetermineSwingingFoot()
   {
      while(footstepStatusQueue.isNewPacketAvailable())
      {
         FootstepStatus poll = footstepStatusQueue.poll();
         latestFootstepStatus.set(poll.getRobotSide(), poll);
         nextSideToSwing.set(poll.getRobotSide().getOppositeSide());
      }

      currentlySwingingFoot.set(null);

      for (RobotSide side: RobotSide.values())
      {
         FootstepStatus status = latestFootstepStatus.get(side);
         if(status != null)
         {
            latestFootstepStatusEnum.get(side).set(status.getStatus());

            if (status.getStatus() == FootstepStatus.Status.STARTED)
            {
               currentlySwingingFoot.set(side);
            }
         }
      }

      for (RobotSide side : RobotSide.values)
      {
         FootstepStatus status = latestFootstepStatus.get(side);
         if (status != null)
         {
            javax.vecmath.Point3d desiredFootPositionInWorld = status.getDesiredFootPositionInWorld();
            Quat4d desiredFootOrientationInWorld = status.getDesiredFootOrientationInWorld();

            desiredFootStatusPoses.get(side).setPosition(desiredFootPositionInWorld);
            desiredFootStatusPoses.get(side).setOrientation(desiredFootOrientationInWorld);

            javax.vecmath.Point3d actualFootPositionInWorld = status.getActualFootPositionInWorld();
            Quat4d actualFootOrientationInWorld = status.getActualFootOrientationInWorld();

            actualFootStatusPoses.get(side).setPosition(actualFootPositionInWorld);
            actualFootStatusPoses.get(side).setOrientation(actualFootOrientationInWorld);
         }
      }

   }


   private void requestPlanarRegionsList()
   {
      RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = new RequestPlanarRegionsListMessage(RequestPlanarRegionsListMessage.RequestType.SINGLE_UPDATE);
      requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE);
      sendPacket(requestPlanarRegionsListMessage);
   }

   private void updatePlannerIfPlanarRegionsListIsAvailable()
   {
      if (planarRegionsListQueue.isNewPacketAvailable())
      {
         planarRegionsListCount.increment();

         PlanarRegionsListMessage planarRegionsListMessage = planarRegionsListQueue.getLatestPacket();
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         planner.setPlanarRegions(planarRegionsList);
      }
   }

   private FootstepPlanner createFootstepPlanner()
   {
      FootstepPlanner planner = new TurnWalkTurnPlanner();

      return planner;
   }

   private void sendTextToSpeechPacket(String message)
   {
      TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket(message);
      sendPacketToUI(textToSpeechPacket);
   }

   private void pitchHeadToFindFiducial()
   {

   }

   private void pitchHeadToCenterFiducial()
   {

   }

   private FootstepDataListMessage createFootstepDataListFromPlan(FootstepPlan plan, int maxNumberOfStepsToTake)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingTime(swingTime.getDoubleValue());
      footstepDataListMessage.setDefaultTransferTime(transferTime.getDoubleValue());
      int lastStepIndex = Math.min(maxNumberOfStepsToTake + 1, plan.getNumberOfSteps());
      for (int i = 1; i < lastStepIndex; i++)
      {
         SimpleFootstep footstep = plan.getFootstep(i);
         footstep.getSoleFramePose(tempFirstFootstepPose);
         tempFirstFootstepPose.getPosition(tempFootstepPosePosition);
         tempFirstFootstepPose.getOrientation(tempFirstFootstepPoseOrientation);

         //         sendTextToSpeechPacket("Sending footstep " + footstep.getRobotSide() + " " + tempFootstepPosePosition + " " + tempFirstFootstepPoseOrientation);

         FootstepDataMessage firstFootstepMessage = new FootstepDataMessage(footstep.getRobotSide(), new javax.vecmath.Point3d(tempFootstepPosePosition), new Quat4d(tempFirstFootstepPoseOrientation));
         footstepDataListMessage.add(firstFootstepMessage);
      }

      footstepDataListMessage.setExecutionMode(ExecutionMode.OVERRIDE);
      return footstepDataListMessage;
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void doControl()
   {
      checkFootstepStatusAndDetermineSwingingFoot();

      if (footstepSentTimer.totalElapsed() < 0.5)
      {
         return;
      }

      updatePlannerIfPlanarRegionsListIsAvailable();
      requestPlanarRegionsList();

      if (!fiducialDetectorBehaviorService.getGoalHasBeenLocated())
      {
         sendTextToSpeechPacket("Fiducial not located.");
         footstepSentTimer.reset();
         pitchHeadToFindFiducial();
         return;
      }

      pitchHeadToCenterFiducial();

      fiducialDetectorBehaviorService.getReportedGoalPoseWorldFrame(tempFootstepPlannerGoalPose);
      setGoalAndInitialFootClosestToGoal(tempFootstepPlannerGoalPose);


      planner.plan();
      FootstepPlan plan = planner.getPlan();

      if (plan == null)
      {
         sendTextToSpeechPacket("No Plan was found!");
         footstepSentTimer.reset();
         return;
      }

      int maxNumberOfStepsToTake = 5;
      FootstepDataListMessage footstepDataListMessage = createFootstepDataListFromPlan(plan, maxNumberOfStepsToTake);
      sendFootstepDataListMessage(footstepDataListMessage);
   }

   @Override
   public void onBehaviorEntered()
   {
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

   @Override
   public void onBehaviorExited()
   {
   }
}