package us.ihmc.gdx.ui;

import imgui.ImGui;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationBehaviorAPI;
import us.ihmc.behaviors.tools.ManagedMessager;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.utilities.ros.RosNodeInterface;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class ImGuiTargetPlacementPanel
{
   private static final String WINDOW_NAME = "Target Placement";

   private ManagedMessager managedMessager = new ManagedMessager();

   private boolean targetEnabled = false;
   private ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();

   public ImGuiTargetPlacementPanel(RosNodeInterface ros1Node, ROS2NodeInterface ros2Node)
   {
      executor.scheduleAtFixedRate(this::run, 0, 2000, TimeUnit.MILLISECONDS);
   }

   public void run()
   {
      if (targetEnabled)
      {
         LogTools.info("Running Target Placer");

         Pose3D goalPose = new Pose3D(0, 10, 0, 0, 0, 0);
         managedMessager.submitMessage(BuildingExplorationBehaviorAPI.Goal, goalPose);
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
