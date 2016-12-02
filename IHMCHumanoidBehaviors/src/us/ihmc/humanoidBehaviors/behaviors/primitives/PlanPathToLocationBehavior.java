package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.packets.ToolboxStateMessage.ToolboxState;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.taskExecutor.PipeLine;

public class PlanPathToLocationBehavior extends AbstractBehavior
{
   //wakeup, request plan
   private final boolean DEBUG = false;
   private boolean planningSuccess = false;
   private PipeLine pipeLine = new PipeLine();
   private FramePose goalPose = null;
   private double timeout = 5.0;
   private final SleepBehavior sleepBehavior;
   private final FramePose initialStanceFootPose = new FramePose();
   private RobotSide initialStanceSide;
   private FootstepDataListMessage footstepDataListMessage;
   private FootstepPlanningToolboxOutputStatus footstepPlanningToolboxOutputStatus;

   protected final ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus> footPlanStatusQueue = new ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus>(
         2);

   public PlanPathToLocationBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.attachNetworkListeningQueue(footPlanStatusQueue, FootstepPlanningToolboxOutputStatus.class);

      sleepBehavior = new SleepBehavior(outgoingCommunicationBridge, yoTime);
   }

   public void setInputs(FramePose goalPose, FramePose initialStanceFootPose, RobotSide initialStanceSide)
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
               TextToSpeechPacket p1 = new TextToSpeechPacket("Telling Planner To Wake Up");
               sendPacket(p1);
            }
            ToolboxStateMessage wakeUp = new ToolboxStateMessage(ToolboxState.WAKE_UP);
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
               TextToSpeechPacket p1 = new TextToSpeechPacket("Requesting Plan");
               sendPacket(p1);
            }
            FootstepPlanningRequestPacket request = new FootstepPlanningRequestPacket(initialStanceFootPose, initialStanceSide, goalPose);
            sendPackageToPlanner(request);
         }
      };

      BehaviorAction waitForPlan = new BehaviorAction(sleepBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            
               TextToSpeechPacket p1 = new TextToSpeechPacket("Waiting For Plan");
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
               if (footstepPlanningToolboxOutputStatus.planningResult == FootstepPlanningResult.OPTIMAL_SOLUTION
                     || footstepPlanningToolboxOutputStatus.planningResult == FootstepPlanningResult.SUB_OPTIMAL_SOLUTION)
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
                  TextToSpeechPacket p1 = new TextToSpeechPacket("Processing Plan");
                  sendPacket(p1);
               }
            }
            else if (DEBUG)
            {
               TextToSpeechPacket p1 = new TextToSpeechPacket("Plan Failed");
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
      return pipeLine.isDone();
   }

}