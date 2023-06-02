package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.HumanoidPerceptionModule;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2FramePlanarRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2PlanarRegionsVisualizer;
import us.ihmc.rdx.ui.visualizers.RDXGlobalVisualizersPanel;
import us.ihmc.ros2.ROS2Node;

public class RDXHumanoidPerceptionUI
{
   private HumanoidPerceptionModule humanoidPerception;
   private RDXRemotePerceptionUI remotePerceptionUI;
   private RDXRapidRegionsUI rapidRegionsUI;
   private RDXROS2PlanarRegionsVisualizer rapidRegionsMapVisualizer;
   private RDXROS2FramePlanarRegionsVisualizer rapidRegionsVisualizer;

   public RDXHumanoidPerceptionUI(HumanoidPerceptionModule humanoidPerception, ROS2Helper ros2Helper)
   {
      this.humanoidPerception = humanoidPerception;
      this.remotePerceptionUI = new RDXRemotePerceptionUI(ros2Helper);
      this.rapidRegionsUI = new RDXRapidRegionsUI();
      this.rapidRegionsUI.create(humanoidPerception.getRapidRegionsExtractor());
   }

   public void renderImGuiWidgets()
   {
      rapidRegionsUI.renderImGuiWidgets();
   }

   public void render()
   {
      rapidRegionsUI.render();
   }

   public void initializePerspectiveRegionsVisualizer(ROS2Node ros2Node, RDXGlobalVisualizersPanel globalVisualizersUI, boolean render) {
      rapidRegionsMapVisualizer = new RDXROS2PlanarRegionsVisualizer("SLAM Rapid Regions",
              ros2Node, PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS);
      rapidRegionsMapVisualizer.setActive(render);
      globalVisualizersUI.addVisualizer(rapidRegionsMapVisualizer);
   }

   public void initializeMapRegionsVisualizer(ROS2Node ros2Node, RDXGlobalVisualizersPanel globalVisualizersUI, boolean render)
   {
      rapidRegionsVisualizer = new RDXROS2FramePlanarRegionsVisualizer("Rapid Regions",
              ros2Node, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS);
      rapidRegionsVisualizer.setActive(render);
      globalVisualizersUI.addVisualizer(rapidRegionsVisualizer);
   }

   public RDXRapidRegionsUI getRapidRegionsUI()
   {
      return rapidRegionsUI;
   }

   public RDXRemotePerceptionUI getRemotePerceptionUI()
   {
      return remotePerceptionUI;
   }

   public void destroy()
   {
      rapidRegionsUI.destroy();
      rapidRegionsVisualizer.destroy();
      rapidRegionsMapVisualizer.destroy();
   }
}
