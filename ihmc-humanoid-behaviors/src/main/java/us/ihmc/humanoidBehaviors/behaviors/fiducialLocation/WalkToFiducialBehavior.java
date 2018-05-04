package us.ihmc.humanoidBehaviors.behaviors.fiducialLocation;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.variable.YoBoolean;

public class WalkToFiducialBehavior extends AbstractBehavior
{
   private final YoBoolean sentPlanningRequest = new YoBoolean("SentPlanningRequest", registry);
   private final YoBoolean recievedPlan = new YoBoolean("RecievedPlan", registry);
   private final YoBoolean planValid = new YoBoolean("PlanValid", registry);

   private final FramePose3D goalPose = new FramePose3D();
   private final ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus> footstepPlanQueue = new ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus>(40);

   private final FootstepPlannerType plannerToUse = FootstepPlannerType.PLANAR_REGION_BIPEDAL;

   public WalkToFiducialBehavior(CommunicationBridgeInterface communicationBridge)
   {
      super(communicationBridge);
      attachNetworkListeningQueue(footstepPlanQueue, FootstepPlanningToolboxOutputStatus.class);
   }

   @Override
   public void doControl()
   {
      if (!sentPlanningRequest.getBooleanValue())
      {
         FootstepPlanningRequestPacket request = HumanoidMessageTools.createFootstepPlanningRequestPacket(new FramePose3D(ReferenceFrame.getWorldFrame()), RobotSide.LEFT, goalPose, plannerToUse);
         request.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE.ordinal());
         sendPacket(request);
      }

      if (!recievedPlan.getBooleanValue() && footstepPlanQueue.isNewPacketAvailable())
      {
         FootstepPlanningToolboxOutputStatus latestPacket = footstepPlanQueue.getLatestPacket();
         planValid.set(FootstepPlanningResult.fromByte(latestPacket.getFootstepPlanningResult()).validForExecution());

         if (planValid.getBooleanValue())
         {
            FootstepDataListMessage footstepDataList = latestPacket.getFootstepDataList();
            sendPacketToUI(footstepDataList);
         }

         recievedPlan.set(true);
      }
   }

   @Override
   public boolean isDone()
   {
      return false; //recievedPlan.getBooleanValue();
   }

   @Override
   public void onBehaviorEntered()
   {
      footstepPlanQueue.clear();
      sentPlanningRequest.set(false);
      recievedPlan.set(false);
      planValid.set(false);
   }

   public void setGoalPose(FramePose3D goalPose)
   {
      goalPose.setIncludingFrame(goalPose);
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
