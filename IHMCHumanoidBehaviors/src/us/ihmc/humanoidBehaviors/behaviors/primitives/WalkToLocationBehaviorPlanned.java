package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.packets.ToolboxStateMessage.ToolboxState;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.taskExecutor.PipeLine;

public class WalkToLocationBehaviorPlanned extends AbstractBehavior
{
   //wakeup, request plan
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final boolean DEBUG = false;
   private final FullRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private boolean planningSuccess = false;
   private PipeLine pipeLine = new PipeLine();
   private FramePose goalPose = null;
   

   public WalkToLocationBehaviorPlanned(CommunicationBridgeInterface outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel,
         HumanoidReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters)
   {
      super(outgoingCommunicationBridge);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;

      setupPipeline();
   }
   
   private void setGoalPose(FramePose goalPose)
   {
      this.goalPose = goalPose; 
   }
   

   private void setupPipeline()
   {

      pipeLine.clearAll();

      BehaviorAction wakeup = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            ToolboxStateMessage wakeUp = new ToolboxStateMessage(ToolboxState.WAKE_UP);
            sendPackageToPlanner(wakeUp);
         }
      };
      
      BehaviorAction requestPlan = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
//            FootstepPlanningRequestPacket request = new FootstepPlanningRequestPacket(initialStanceFootPose, initialStanceSide, goalPose);
//            sendPackageToPlanner(request);
         }
      };

   }

   private void sendPackageToPlanner(Packet<?> packet)
   {
      packet.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE);
      communicationBridge.sendPacket(packet);
   }

   @Override
   public void doControl()
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void onBehaviorEntered()
   {
      setupPipeline();

   }

   @Override
   public void onBehaviorAborted()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorPaused()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorResumed()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public boolean isDone()
   {
      // TODO Auto-generated method stub
      return false;
   }

}