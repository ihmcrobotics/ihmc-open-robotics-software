package us.ihmc.avatar.networkProcessor.footstepPlanAndProcessModule;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.FootstepPlanPostProcessingModule;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.FootstepPlanPostProcessingModuleLauncher;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.CloseableAndDisposable;

public class FootstepPlanAndProcessModule implements CloseableAndDisposable
{
   private final FootstepPlanningModule planningModule;
   private final FootstepPlanPostProcessingModule postProcessingModule;

   public static final String MODULE_NAME = "footstep_plan_and_process_module";

   public FootstepPlanAndProcessModule(DRCRobotModel robotModel, PubSubImplementation pubSubImplementation)
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, MODULE_NAME);

      this.planningModule = FootstepPlanningModuleLauncher.createModule(ros2Node, robotModel);
      this.postProcessingModule = FootstepPlanPostProcessingModuleLauncher.createModule(ros2Node, robotModel);
   }

   public FootstepPlanningModule getPlanningModule()
   {
      return planningModule;
   }

   public FootstepPlanPostProcessingModule getPostProcessingModule()
   {
      return postProcessingModule;
   }

   @Override
   public void closeAndDispose()
   {
      planningModule.closeAndDispose();
      postProcessingModule.closeAndDispose();
   }
}
