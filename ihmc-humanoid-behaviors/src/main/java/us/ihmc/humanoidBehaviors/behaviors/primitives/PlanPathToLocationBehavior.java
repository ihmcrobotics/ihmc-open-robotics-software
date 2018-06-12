package us.ihmc.humanoidBehaviors.behaviors.primitives;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningRequestPacketPubSubType;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatusPubSubType;
import controller_msgs.msg.dds.ToolboxStateMessage;
import controller_msgs.msg.dds.ToolboxStateMessagePubSubType;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.taskExecutor.PipeLine;
import us.ihmc.yoVariables.variable.YoDouble;

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

   protected final ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus> footPlanStatusQueue = new ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus>(2);
   private final IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final IHMCROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;

   public PlanPathToLocationBehavior(Ros2Node ros2Node, YoDouble yoTime)
   {
      super(ros2Node);

      createSubscriber(footPlanStatusQueue, new FootstepPlanningToolboxOutputStatusPubSubType(), "/ihmc/footstep_planning_toolbox_output_status");
      toolboxStatePublisher = createPublisher(new ToolboxStateMessagePubSubType(), "/ihmc/toolbox_state");
      footstepPlanningRequestPublisher = createPublisher(new FootstepPlanningRequestPacketPubSubType(), "/ihmc/footstep_planning_request");

      sleepBehavior = new SleepBehavior(ros2Node, yoTime);
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

      BehaviorAction wakeup = new BehaviorAction(new SimpleDoNothingBehavior(ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            if (DEBUG)
            {
               publishTextToSpeack("Telling Planner To Wake Up");
            }
            toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

         }
      };

      BehaviorAction requestPlan = new BehaviorAction(new SimpleDoNothingBehavior(ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            if (DEBUG)
            {
               publishTextToSpeack("Requesting Plan");
            }
            footstepPlanningRequestPublisher.publish(HumanoidMessageTools.createFootstepPlanningRequestPacket(initialStanceFootPose, initialStanceSide,
                                                                                                              goalPose));
         }
      };

      BehaviorAction waitForPlan = new BehaviorAction(sleepBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            publishTextToSpeack("Waiting For Plan");

            sleepBehavior.setSleepTime(timeout);
         }

         @Override
         public boolean isDone()
         {
            return super.isDone() || footPlanStatusQueue.isNewPacketAvailable();
         }
      };

      BehaviorAction processPlan = new BehaviorAction(new SimpleDoNothingBehavior(ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {

            if (footPlanStatusQueue.isNewPacketAvailable())
            {
               footstepPlanningToolboxOutputStatus = footPlanStatusQueue.getLatestPacket();
               if (footstepPlanningToolboxOutputStatus.getFootstepPlanningResult() == FootstepPlanningResult.OPTIMAL_SOLUTION.toByte()
                     || footstepPlanningToolboxOutputStatus.getFootstepPlanningResult() == FootstepPlanningResult.SUB_OPTIMAL_SOLUTION.toByte())
               {
                  planningSuccess = true;
                  footstepDataListMessage = footstepPlanningToolboxOutputStatus.getFootstepDataList();
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
                  publishTextToSpeack("Processing Plan");
               }
            }
            else if (DEBUG)
            {
               publishTextToSpeack("Plan Failed");
            }

         }
      };

      pipeLine.requestNewStage();

      pipeLine.submitSingleTaskStage(wakeup);
      pipeLine.submitSingleTaskStage(requestPlan);
      pipeLine.submitSingleTaskStage(waitForPlan);
      pipeLine.submitSingleTaskStage(processPlan);

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