package us.ihmc.gdx.ui;

import imgui.ImGui;
import us.ihmc.behaviors.BehaviorRegistry;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI;
import us.ihmc.behaviors.tools.MessagerHelper;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.utilities.ros.RosNodeInterface;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class ImGuiTargetPlacementPanel
{
   private static final String WINDOW_NAME = "Target Placement";

   private final MessagerHelper messagerHelper;

   private boolean targetEnabled = false;
   private ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();

   private IHMCROS2Publisher<Pose3D> goalInputPublisher;
   private String currentBehaviorState;
   private Pose3D goalPose;

   public ImGuiTargetPlacementPanel(RosNodeInterface ros1Node, ROS2NodeInterface ros2Node)
   {
      ImGuiGDXBehaviorUIRegistry defaultBehaviors = ImGuiGDXBehaviorUIRegistry.DEFAULT_BEHAVIORS;
      defaultBehaviors.activateRegistry();
      messagerHelper = new MessagerHelper(BehaviorRegistry.getActiveRegistry().getMessagerAPI());

      executor.scheduleAtFixedRate(this::run, 0, 2000, TimeUnit.MILLISECONDS);
      this.goalInputPublisher = IHMCROS2Publisher.newPose3DPublisher(ros2Node, LookAndStepBehaviorAPI.GOAL_INPUT);
      messagerHelper.subscribeViaCallback(LookAndStepBehaviorAPI.CurrentState, message -> {
         LogTools.info("Received State Message: {}", message);
         currentBehaviorState = message;
      });
   }

   public void run()
   {
      if (targetEnabled)
      {
         LogTools.info("Running Target Placer: CurrentState: {}", currentBehaviorState);

         if(currentBehaviorState.equals(LookAndStepBehavior.State.BODY_PATH_PLANNING.name()))
         {
            goalPose = new Pose3D(0, 10, 0, 0, 0, 0);
            this.goalInputPublisher.publish(goalPose);
         }
      }
   }

   public void render()
   {
      if (ImGui.button("Place Target"))
      {
         targetEnabled = true;
      }
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}
