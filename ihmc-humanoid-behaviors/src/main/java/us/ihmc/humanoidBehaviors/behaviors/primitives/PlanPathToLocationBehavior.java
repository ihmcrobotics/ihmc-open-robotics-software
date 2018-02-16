package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.taskExecutor.PipeLine;

public class PlanPathToLocationBehavior extends AbstractBehavior
{
   //wakeup, request plan
   private final boolean DEBUG = false;
   private boolean planningSuccess = false;
   private PipeLine pipeLine = new PipeLine();
   private FramePose3D goalPose = null;
   private double timeout = 5.0;
   private final SleepBehavior sleepBehavior;
   private final FramePose3D initialStanceFootPose = new FramePose3D();
   private RobotSide initialStanceSide;
   private FootstepDataListMessage footstepDataListMessage;
   private FootstepPlanningToolboxOutputStatus footstepPlanningToolboxOutputStatus;

   protected final ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus> footPlanStatusQueue = new ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus>(
         2);

   public PlanPathToLocationBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, YoDouble yoTime)
   {
      super(outgoingCommunicationBridge);

      this.attachNetworkListeningQueue(footPlanStatusQueue, FootstepPlanningToolboxOutputStatus.class);

      sleepBehavior = new SleepBehavior(outgoingCommunicationBridge, yoTime);
   }

   public void setInputs(FramePose3D goalPose, FramePose3D initialStanceFootPose, RobotSide initialStanceSide)
   {
      this.goalPose = goalPose;
      this.initialStanceSide = initialStanceSide;
      this.initialStanceFootPose.setIncludingFrame(initialStanceFootPose);
      this.initialStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
   }

   public FootstepDataListMessage getFootStepList()
   {
      return footstepDataListMessage;
   }

   public FootstepPlanningToolboxOutputStatus geFootstepPlanningToolboxOutputStatus()
   {
      return footstepPlanningToolboxOutputStatus;
   }

   public void setPlanningTimeout(double timeout)
   {
      this.timeout = timeout;
   }

   private void setupPipeline()
   {

      pipeLine.clearAll();

      BehaviorAction wakeup = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            if (DEBUG)
            {
               TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Telling Planner To Wake Up");
               sendPacket(p1);
            }
            ToolboxStateMessage wakeUp = MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP);
            sendPackageToPlanner(wakeUp);

         }
      };

      BehaviorAction requestPlan = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            if (DEBUG)
            {
               TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Requesting Plan");
               sendPacket(p1);
            }
            FootstepPlanningRequestPacket request = HumanoidMessageTools.createFootstepPlanningRequestPacket(initialStanceFootPose, initialStanceSide, goalPose);
            sendPackageToPlanner(request);
         }
      };

      BehaviorAction waitForPlan = new BehaviorAction(sleepBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            
               TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Waiting For Plan");
               sendPacket(p1);
           
            sleepBehavior.setSleepTime(timeout);
         }

         @Override
         public boolean isDone()
         {
            return super.isDone() || footPlanStatusQueue.isNewPacketAvailable();
         }
      };

      BehaviorAction processPlan = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {

            if (footPlanStatusQueue.isNewPacketAvailable())
            {
               footstepPlanningToolboxOutputStatus = footPlanStatusQueue.getLatestPacket();
               if (footstepPlanningToolboxOutputStatus.planningResult == FootstepPlanningResult.OPTIMAL_SOLUTION.toByte()
                     || footstepPlanningToolboxOutputStatus.planningResult == FootstepPlanningResult.SUB_OPTIMAL_SOLUTION.toByte())
               {
                  planningSuccess = true;
                  footstepDataListMessage = footstepPlanningToolboxOutputStatus.footstepDataList;
               }
               else
                  planningSuccess = false;
            }
            else
            {
               planningSuccess = false;
            }

            if (planningSuccess)
            {
               if (DEBUG)
               {
                  TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Processing Plan");
                  sendPacket(p1);
               }
            }
            else if (DEBUG)
            {
               TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Plan Failed");
               sendPacket(p1);
            }

         }
      };

      pipeLine.requestNewStage();

      pipeLine.submitSingleTaskStage(wakeup);
      pipeLine.submitSingleTaskStage(requestPlan);
      pipeLine.submitSingleTaskStage(waitForPlan);
      pipeLine.submitSingleTaskStage(processPlan);

   }

   private void sendPackageToPlanner(Packet<?> packet)
   {
      packet.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE);
      communicationBridge.sendPacket(packet);
   }

   @Override
   public void doControl()
   {
      pipeLine.doControl();
   }

   public boolean planSuccess()
   {
      return planningSuccess;
   }

   @Override
   public void onBehaviorEntered()
   {
      setupPipeline();
      planningSuccess = false;
      footstepDataListMessage = null;
      footstepPlanningToolboxOutputStatus = null;
      footPlanStatusQueue.clear();
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

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

}