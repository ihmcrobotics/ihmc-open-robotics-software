package us.ihmc.humanoidBehaviors.behaviors.fiducialLocation;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlanner;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage.Status;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.time.Timer;

public class FollowFiducialBehavior extends AbstractBehavior
{
   private final FiducialDetectorBehaviorService fiducialDetectorBehaviorService;
   private final long fiducialToTrack;
   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;

   private final ConcurrentListeningQueue<RobotConfigurationData> robotConfigurationDataQueue;
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue;
   private final ConcurrentListeningQueue<WalkingStatusMessage> walkingStatusQueue;

   private SideDependentList<FootstepStatus> latestFootstepStatus;

   private final FootstepPlanner footstepPlanner;
   private final FootstepPlannerGoal footstepPlannerGoal;
   private final YoFramePose footstepPlannerInitialStepPose;
   private final YoFramePose footstepPlannerGoalPose;
   private final FramePose tempFootstepPlannerGoalPose;
   private final FramePose tempLeftFootPose;
   private final FramePose tempRightFootPose;
   private final FramePose tempStanceFootPose;
   private final FramePose tempFirstFootstepPose;
   private final Point3d tempFirstFootstepPosePosition;
   private final Quat4d tempFirstFootstepPoseOrientation;
   private final Timer footstepSentTimer;

   public FollowFiducialBehavior(CommunicationBridge behaviorCommunicationBridge, FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, FiducialDetectorBehaviorService fiducialDetectorBehaviorService,
                                 long fiducialToTrack)
   {
      super(FollowFiducialBehavior.class.getSimpleName(), behaviorCommunicationBridge);

      this.fiducialToTrack = fiducialToTrack;
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;

      this.fiducialDetectorBehaviorService = fiducialDetectorBehaviorService;
      fiducialDetectorBehaviorService.setLocationEnabled(true);
      fiducialDetectorBehaviorService.setTargetIDToLocate(this.fiducialToTrack);

      footstepPlanner = createFootstepPlanner();

      footstepPlannerGoal = new FootstepPlannerGoal();
      tempFootstepPlannerGoalPose = new FramePose();
      tempStanceFootPose = new FramePose();
      tempFirstFootstepPose = new FramePose();
      tempLeftFootPose = new FramePose();
      tempRightFootPose = new FramePose();
      tempFirstFootstepPosePosition = new Point3d();
      tempFirstFootstepPoseOrientation = new Quat4d();
      footstepSentTimer = new Timer();
      footstepSentTimer.start();

      String prefix = "followFiducial";
      footstepPlannerGoalPose = new YoFramePose(prefix + "FootstepGoalPose", ReferenceFrame.getWorldFrame(), registry);
      footstepPlannerInitialStepPose = new YoFramePose(prefix + "InitialStepPose", ReferenceFrame.getWorldFrame(), registry);

      robotConfigurationDataQueue = new ConcurrentListeningQueue<RobotConfigurationData>();
      footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>();
      latestFootstepStatus = new SideDependentList<>();
      walkingStatusQueue = new ConcurrentListeningQueue<WalkingStatusMessage>();
      behaviorCommunicationBridge.attachNetworkListeningQueue(robotConfigurationDataQueue, RobotConfigurationData.class);
      behaviorCommunicationBridge.attachNetworkListeningQueue(footstepStatusQueue, FootstepStatus.class);
      behaviorCommunicationBridge.attachNetworkListeningQueue(walkingStatusQueue, WalkingStatusMessage.class);
   }

   private FootstepPlanner createFootstepPlanner()
   {
      PlanarRegionBipedalFootstepPlanner planner = new PlanarRegionBipedalFootstepPlanner();

      planner.setMaximumStepReach(0.4);
      planner.setMaximumStepZ(0.25);
      planner.setMaximumStepYaw(0.25);
      planner.setMinimumStepWidth(0.15);
      planner.setMinimumFootholdPercent(0.8);

      double idealFootstepLength = 0.3;
      double idealFootstepWidth = 0.2;
      planner.setIdealFootstep(idealFootstepLength, idealFootstepWidth);

      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = PlanningTestTools.createDefaultFootPolygons();
      planner.setFeetPolygons(footPolygonsInSoleFrame);

      planner.setMaximumNumberOfNodesToExpand(1000);

      return planner;
   }

   @Override
   public void doControl()
   {
      if (footstepSentTimer.totalElapsed() < 0.5)
      {
         return;
      }

      WalkingStatusMessage walkingStatusLatestPacket = walkingStatusQueue.getLatestPacket();
      if (walkingStatusLatestPacket != null && walkingStatusLatestPacket.getWalkingStatus() != Status.COMPLETED)
      {
         return;
      }

      boolean okToSendMoreFootsteps = getLatestFootstepStatusAndCheckIfOkToSendMoreFootsteps();
      if (!okToSendMoreFootsteps)
         return;

      if (!fiducialDetectorBehaviorService.getTargetIDHasBeenLocated())
      {
         sendTextToSpeechPacket("Fiducial not located.");
         return;
      }

      fiducialDetectorBehaviorService.getReportedFiducialPoseWorldFrame(tempFootstepPlannerGoalPose);
      setGoalAndInitialStanceFootToBeClosestToGoal(tempFootstepPlannerGoalPose);

      footstepPlanner.plan();
      FootstepPlan plan = footstepPlanner.getPlan();

      if (plan == null)
      {
//         sendTextToSpeechPacket("No Plan was found!");
         footstepSentTimer.reset();
         return;
      }

      FootstepDataListMessage footstepDataListMessage = createFootstepDataListFromPlan(plan);
      sendFootstepDataListMessage(footstepDataListMessage);
   }

   private void sendTextToSpeechPacket(String message)
   {
      TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket(message);
      sendPacketToUI(textToSpeechPacket);
   }

   private boolean getLatestFootstepStatusAndCheckIfOkToSendMoreFootsteps()
   {
      while (footstepStatusQueue.isNewPacketAvailable())
      {
         FootstepStatus poll = footstepStatusQueue.poll();
         latestFootstepStatus.set(poll.getRobotSide(), poll);
      }

      for (RobotSide side : RobotSide.values)
      {
         if (latestFootstepStatus.get(side) != null && latestFootstepStatus.get(side).getStatus() == FootstepStatus.Status.STARTED)
         {
            PrintTools.debug(this, side + " footstep not complete");
            return false;
         }
      }

      return true;
   }

   private void setGoalAndInitialStanceFootToBeClosestToGoal(FramePose goalPose)
   {
      tempLeftFootPose.setToZero(referenceFrames.getFootFrame(RobotSide.LEFT));
      tempRightFootPose.setToZero(referenceFrames.getFootFrame(RobotSide.RIGHT));
      tempLeftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      tempRightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3d temp = new Point3d();
      Point3d pointBetweenFeet = new Point3d();
      Vector3d vectorFromFeetToGoal = new Vector3d();

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

      double distanceFromGoalToLeftFoot = tempLeftFootPose.getPositionDistance(goalPose);
      double distanceFromGoalToRightFoot = tempRightFootPose.getPositionDistance(goalPose);

      RobotSide stanceSide;
      if (distanceFromGoalToLeftFoot > distanceFromGoalToRightFoot)
      {
         stanceSide = RobotSide.LEFT;
         tempStanceFootPose.set(tempLeftFootPose);
         goalPose.setZ(tempLeftFootPose.getZ());
      }
      else
      {
         stanceSide = RobotSide.RIGHT;
         tempStanceFootPose.set(tempRightFootPose);
         goalPose.setZ(tempRightFootPose.getZ());
      }

      sendTextToSpeechPacket("Planning footsteps from " + tempStanceFootPose + " to " + goalPose);
      footstepPlannerGoal.setGoalPoseBetweenFeet(goalPose);
      footstepPlanner.setGoal(footstepPlannerGoal);

      footstepPlanner.setInitialStanceFoot(tempStanceFootPose, stanceSide);

      footstepPlannerGoalPose.set(goalPose);
      footstepPlannerInitialStepPose.set(tempStanceFootPose);
   }

   private void sendFootstepDataListMessage(FootstepDataListMessage footstepDataListMessage)
   {
      footstepDataListMessage.setDestination(PacketDestination.UI);
      sendPacket(footstepDataListMessage);

      footstepDataListMessage.setDestination(PacketDestination.CONTROLLER);
      sendPacketToController(footstepDataListMessage);
      footstepSentTimer.reset();
   }

   private FootstepDataListMessage createFootstepDataListFromPlan(FootstepPlan plan)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setSwingTime(1.0);
      footstepDataListMessage.setTransferTime(1.0);
      int maxNumberOfStepsToTake = 3;
      int lastStepIndex = Math.min(maxNumberOfStepsToTake + 1, plan.getNumberOfSteps());
      for (int i = 1; i < lastStepIndex; i++)
      {
         SimpleFootstep firstFootstep = plan.getFootstep(i);
         firstFootstep.getSoleFramePose(tempFirstFootstepPose);
         tempFirstFootstepPose.getPosition(tempFirstFootstepPosePosition);
         tempFirstFootstepPose.getOrientation(tempFirstFootstepPoseOrientation);

         TextToSpeechPacket foostepTextToSpeechPacket = new TextToSpeechPacket("Sending footstep " + firstFootstep.getRobotSide() + " " + tempFirstFootstepPosePosition
               + " " + tempFirstFootstepPoseOrientation);
         sendPacketToUI(foostepTextToSpeechPacket);

         FootstepDataMessage firstFootstepMessage = new FootstepDataMessage(firstFootstep.getRobotSide(), new Point3d(tempFirstFootstepPosePosition),
               new Quat4d(tempFirstFootstepPoseOrientation));
         footstepDataListMessage.add(firstFootstepMessage);

      }
      return footstepDataListMessage;
   }

   @Override
   public void initialize()
   {
      super.initialize();
      fiducialDetectorBehaviorService.initialize();
   }

   @Override
   public void pause()
   {
      super.pause();
      fiducialDetectorBehaviorService.pause();
   }

   @Override
   public void abort()
   {
      super.abort();
      fiducialDetectorBehaviorService.stop();
   }

   @Override
   public void resume()
   {
      super.resume();
      fiducialDetectorBehaviorService.resume();
   }

   @Override
   public boolean isDone()
   {
      return false;
   }
}
