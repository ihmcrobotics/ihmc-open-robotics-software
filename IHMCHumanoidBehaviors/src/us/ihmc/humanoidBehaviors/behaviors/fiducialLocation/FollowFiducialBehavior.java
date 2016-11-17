package us.ihmc.humanoidBehaviors.behaviors.fiducialLocation;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
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

   private final TurnWalkTurnPlanner turnWalkTurnPlanner;
   private final FootstepPlannerGoal footstepPlannerGoal;
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

      turnWalkTurnPlanner = new TurnWalkTurnPlanner();
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

      robotConfigurationDataQueue = new ConcurrentListeningQueue<RobotConfigurationData>();
      footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>();
      latestFootstepStatus = new SideDependentList<>();
      walkingStatusQueue = new ConcurrentListeningQueue<WalkingStatusMessage>();
      behaviorCommunicationBridge.attachNetworkListeningQueue(robotConfigurationDataQueue, RobotConfigurationData.class);
      behaviorCommunicationBridge.attachNetworkListeningQueue(footstepStatusQueue, FootstepStatus.class);
      behaviorCommunicationBridge.attachNetworkListeningQueue(walkingStatusQueue, WalkingStatusMessage.class);
   }

   @Override
   public void doControl()
   {
      if (footstepSentTimer.totalElapsed() < 0.5)
      {
         return;
      }
      
      
      WalkingStatusMessage walkingStatusLatestPacket = walkingStatusQueue.getLatestPacket();
      if (walkingStatusLatestPacket == null || walkingStatusLatestPacket.getWalkingStatus() == Status.COMPLETED)
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
               return;
            }
         }
         

         if (!fiducialDetectorBehaviorService.getTargetIDHasBeenLocated())
         {
            TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket("Fiducial not located.");
            sendPacketToUI(textToSpeechPacket);
            return;
         }

         fiducialDetectorBehaviorService.getReportedFiducialPoseWorldFrame(tempFootstepPlannerGoalPose);
         footstepPlannerGoalPose.set(tempFootstepPlannerGoalPose);

         TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket("Planning footstep to " + tempFootstepPlannerGoalPose);
         sendPacketToUI(textToSpeechPacket);

         footstepPlannerGoal.setGoalPoseBetweenFeet(tempFootstepPlannerGoalPose);
         turnWalkTurnPlanner.setGoal(footstepPlannerGoal);

         tempLeftFootPose.setToZero(referenceFrames.getFootFrame(RobotSide.LEFT));
         tempRightFootPose.setToZero(referenceFrames.getFootFrame(RobotSide.RIGHT));
         tempLeftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
         tempRightFootPose.changeFrame(ReferenceFrame.getWorldFrame());
         
         double distanceFromGoalToLeftFoot = tempLeftFootPose.getPositionDistance(tempFootstepPlannerGoalPose);
         double distanceFromGoalToRightFoot = tempRightFootPose.getPositionDistance(tempFootstepPlannerGoalPose);

         RobotSide stanceSide;
         if (distanceFromGoalToLeftFoot > distanceFromGoalToRightFoot)
         {
            stanceSide = RobotSide.LEFT;
            tempStanceFootPose.set(tempLeftFootPose);
         }
         else
         {
            stanceSide = RobotSide.RIGHT;
            tempStanceFootPose.set(tempRightFootPose);
         }

         turnWalkTurnPlanner.setInitialStanceFoot(tempStanceFootPose, stanceSide);
         turnWalkTurnPlanner.plan();
         FootstepPlan plan = turnWalkTurnPlanner.getPlan();

         FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
         footstepDataListMessage.setSwingTime(1.0);
         footstepDataListMessage.setTransferTime(1.0);
         int maxNumberOfStepsToTake = 2;
         int lastStepIndex = Math.max(maxNumberOfStepsToTake + 1, plan.getNumberOfSteps());
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
         
         footstepDataListMessage.setDestination(PacketDestination.UI);
         sendPacket(footstepDataListMessage);
         
         footstepDataListMessage.setDestination(PacketDestination.CONTROLLER);
         sendPacketToController(footstepDataListMessage);
         footstepSentTimer.reset();
      }
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
