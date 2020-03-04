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

   private Ros2Node ros2Node;

   public static final String MODULE_NAME = "footstep_plan_and_process";

   public FootstepPlanAndProcessModule(DRCRobotModel robotModel)
   {
      this.planningModule = FootstepPlanningModuleLauncher.createModule(robotModel);
      this.postProcessingModule = FootstepPlanPostProcessingModuleLauncher.createModule(robotModel);
   }

   @Override
   public void closeAndDispose()
   {
      planningModule.closeAndDispose();
      postProcessingModule.closeAndDispose();
   }

   public void setupWithRos(PubSubImplementation pubSubImplementation)
   {
      if (ros2Node != null)
         return;

      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, MODULE_NAME);
      FootstepPlanningModuleLauncher.setupForRos(planningModule, ros2Node);
      FootstepPlanPostProcessingModuleLauncher.setupForRos(postProcessingModule, ros2Node);
   }
}
