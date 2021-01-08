package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.taskExecutor.PipeLine;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class PlanPathToLocationBehavior extends AbstractBehavior
{
   //wakeup, request plan
   private final boolean DEBUG = true;
   private boolean planningSuccess = false;

   private FootstepPlanningResult planningResult;

   private PipeLine<BehaviorAction> pipeLine;
   private final YoInteger planId = new YoInteger("planId", registry);

   private FramePose3D goalPose = null;
   private double timeout = 5.0;
   private final SleepBehavior sleepBehavior;
   private final FramePose3D startLeftFootPose = new FramePose3D();
   private final FramePose3D startRightFootPose = new FramePose3D();
   private RobotSide initialStanceSide;
   private double desiredHeading;
   private FootstepDataListMessage footstepDataListMessage;
   private FootstepPlanningToolboxOutputStatus footstepPlanningToolboxOutputStatus;
   private boolean squareUpEndSteps = true;

   protected final ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus> footPlanStatusQueue = new ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus>(2);
   private final IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final IHMCROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private final IHMCROS2Publisher<FootstepPlannerParametersPacket> footstepPlannerParametersPublisher;
   private final IHMCROS2Publisher<FootstepDataListMessage> goalFootstepToUIVisualization;

   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();

   private boolean planBodyPath = false;

   private boolean assumeFlatGround = false;
   private FootstepPlannerParametersBasics footstepPlannerParameters;

   public PlanPathToLocationBehavior(String robotName, ROS2Node ros2Node, FootstepPlannerParametersBasics footstepPlannerParameters, YoDouble yoTime)
   {
      super(robotName, ros2Node);
      pipeLine = new PipeLine<>(yoTime);
      this.footstepPlannerParameters = footstepPlannerParameters;
      createSubscriber(FootstepPlanningToolboxOutputStatus.class, footstepPlannerOutputTopic, footPlanStatusQueue::put);
      createSubscriber(PlanarRegionsListMessage.class, REACommunicationProperties.outputTopic, planarRegions::set);

      toolboxStatePublisher = createPublisher(ToolboxStateMessage.class, footstepPlannerInputTopic);
      footstepPlanningRequestPublisher = createPublisher(FootstepPlanningRequestPacket.class, footstepPlannerInputTopic);
      footstepPlannerParametersPublisher = createPublisher(FootstepPlannerParametersPacket.class, footstepPlannerInputTopic);
      goalFootstepToUIVisualization = createBehaviorOutputPublisher(FootstepDataListMessage.class);

      sleepBehavior = new SleepBehavior(robotName, ros2Node, yoTime);
   }

   public void setInputs(FramePose3D goalPose,
                         RobotSide initialStanceSide,
                         Pose3DReadOnly leftFootPose,
                         Pose3DReadOnly rightFootPose,
                         boolean planBodyPath,
                         boolean assumeFlatGround,
                         double desiredHeading, boolean squareUpEndSteps)
   {
      this.squareUpEndSteps = squareUpEndSteps;
      this.goalPose = goalPose;
      this.assumeFlatGround = assumeFlatGround;
      this.planBodyPath = planBodyPath;
      this.initialStanceSide = initialStanceSide;
      this.startLeftFootPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), leftFootPose);
      this.startRightFootPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), rightFootPose);
      this.desiredHeading = desiredHeading;
   }

   public FootstepDataListMessage getFootStepList()
   {
      return footstepDataListMessage;
   }

   public FootstepPlanningResult getPlanningResult()
   {
      return planningResult;
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

      BehaviorAction wakeup = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            if (DEBUG)
            {
               publishTextToSpeech("PlanPathToLocationBehavior: Telling Planner To Wake Up");
            }
            toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

         }
      };

      BehaviorAction requestPlan = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            if (DEBUG)
            {
               publishTextToSpeech("PlanPathToLocationBehavior: Requesting Plan");
            }

            planId.increment();
            FootstepPlanningRequestPacket request = FootstepPlannerMessageTools.createFootstepPlanningRequestPacket(initialStanceSide,
                                                                                                                    startLeftFootPose,
                                                                                                                    startRightFootPose,
                                                                                                                    goalPose,
                                                                                                                    footstepPlannerParameters.getIdealFootstepWidth(),
                                                                                                                    planBodyPath);

            FootstepPlan footstepPlan = new FootstepPlan();
            Pose3D goalLeftFootPose = new Pose3D(goalPose);
            Pose3D goalRightFootPose = new Pose3D(goalPose);
            goalLeftFootPose.appendTranslation(0.0, 0.5 * footstepPlannerParameters.getIdealFootstepWidth(), 0.0);
            goalRightFootPose.appendTranslation(0.0, -0.5 * footstepPlannerParameters.getIdealFootstepWidth(), 0.0);
            footstepPlan.addFootstep(RobotSide.LEFT, startLeftFootPose);
            footstepPlan.addFootstep(RobotSide.RIGHT, startLeftFootPose);
            request.setRequestedPathHeading(desiredHeading);
            if(squareUpEndSteps)
            {
               request.getGoalLeftFootPose().set(goalLeftFootPose);
               request.getGoalRightFootPose().set(goalRightFootPose);
            }


            FootstepDataListMessage footstepDataGoalStepForVisualization = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan,
                                                                                                                                       0.0,
                                                                                                                                       0.0);

            goalFootstepToUIVisualization.publish(footstepDataGoalStepForVisualization);

            request.setTimeout(timeout);
            request.setAssumeFlatGround(assumeFlatGround);
            if (planarRegions.get() != null)
            {
               request.getPlanarRegionsListMessage().set(planarRegions.get());
            }
            else
            {
               publishTextToSpeech("PlanPathToLocationBehavior: Planar regions are null, Requesting Plan without planar regions");
            }
            request.setPlannerRequestId(planId.getIntegerValue());
            request.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE.ordinal());
            request.setGenerateLog(true);
            FootstepPlannerLogger.deleteOldLogs(30);

            FootstepPlannerParametersPacket plannerParametersPacket = new FootstepPlannerParametersPacket();

            FootstepPlannerMessageTools.copyParametersToPacket(plannerParametersPacket, footstepPlannerParameters);
            plannerParametersPacket.setMaximumStepYaw(0.8);
            footstepPlannerParametersPublisher.publish(plannerParametersPacket);

            footstepPlanningRequestPublisher.publish(request);
         }
      };

      BehaviorAction waitForPlan = new BehaviorAction(sleepBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("PlanPathToLocationBehavior: Waiting until plan response for " + timeout);
            sleepBehavior.setSleepTime(timeout);
         }

         @Override
         public boolean isDone()
         {

        	 if(footPlanStatusQueue.isNewPacketAvailable())
        	 {
        		 footstepPlanningToolboxOutputStatus= footPlanStatusQueue.getLatestPacket();
        		 planningResult = FootstepPlanningResult.fromByte(footstepPlanningToolboxOutputStatus.getFootstepPlanningResult());
        		 publishTextToSpeech("Plan received: " + planningResult);
        	 }
        	 
//        	 System.out.println("***********^^^^^^^^^^^^^^^ "+ planningResult);
        	 
            return super.isDone() || (planningResult!=null&&planningResult != FootstepPlanningResult.PLANNING);

         }
      };

      BehaviorAction processPlan = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {

            if (footstepPlanningToolboxOutputStatus!=null)
            {

              // footstepPlanningToolboxOutputStatus = footPlanStatusQueue.getLatestPacket();
              // planningResult = FootstepPlanningResult.fromByte(footstepPlanningToolboxOutputStatus.getFootstepPlanningResult());

               if (planningResult == FootstepPlanningResult.FOUND_SOLUTION)
               {
                  planningSuccess = true;
                  footstepDataListMessage = footstepPlanningToolboxOutputStatus.getFootstepDataList();
               }
               else if (planningResult == FootstepPlanningResult.PLANNING)
               {
//            	   publishTextToSpeech("PlanPathToLocationBehavior: planner timed out after "+timeout+" seconds");
//
//                   planningSuccess = false;
               }
               else
               {
                  publishTextToSpeech("PlanPathToLocationBehavior: bad plan");

                  planningSuccess = false;
               }
            }
            else
            {
               publishTextToSpeech("PlanPathToLocationBehavior: never head back frm footstep planner");

               planningSuccess = false;
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
      if (planningResult != null)
         return planningResult == FootstepPlanningResult.FOUND_SOLUTION;
      return false;
   }

   @Override
   public void onBehaviorEntered()
   {
      setupPipeline();
      planningSuccess = false;
      planningResult = null;
      footstepDataListMessage = null;
      footstepPlanningToolboxOutputStatus = null;
      footPlanStatusQueue.clear();
      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);

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