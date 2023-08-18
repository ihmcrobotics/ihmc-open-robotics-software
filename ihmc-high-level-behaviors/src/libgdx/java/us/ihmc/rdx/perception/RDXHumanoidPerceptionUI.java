package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.HumanoidPerceptionModule;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.rdx.RDXHeightMapRenderer;
import us.ihmc.rdx.ui.graphics.ros2.RDXHeightMapVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2FramePlanarRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2PlanarRegionsVisualizer;
import us.ihmc.rdx.ui.visualizers.RDXGlobalVisualizersPanel;
import us.ihmc.ros2.ROS2Node;

public class RDXHumanoidPerceptionUI
{
   private HumanoidPerceptionModule humanoidPerception;
   private RDXRemotePerceptionUI remotePerceptionUI;
   private RDXRapidRegionsUI rapidRegionsUI;
   private RDXActiveMappingUI activeMappingUI;
   private RDXROS2PlanarRegionsVisualizer rapidRegionsMapVisualizer;
   private RDXROS2FramePlanarRegionsVisualizer rapidRegionsVisualizer;
   private RDXHeightMapRenderer heightMapRenderer;
   private RDXRemoteHeightMapPanel heightMapUI;

   public RDXHumanoidPerceptionUI(HumanoidPerceptionModule humanoidPerception, ROS2Helper ros2Helper)
   {
      this.humanoidPerception = humanoidPerception;
      this.remotePerceptionUI = new RDXRemotePerceptionUI(ros2Helper);

      this.rapidRegionsUI = new RDXRapidRegionsUI();
      this.rapidRegionsUI.create(humanoidPerception.getRapidRegionsExtractor());

      this.activeMappingUI = new RDXActiveMappingUI("Active Mapping", ros2Helper);

      this.humanoidPerception.setPerceptionConfigurationParameters(remotePerceptionUI.getPerceptionConfigurationParameters());
   }

   public void initializeHeightMapRenderer(HumanoidPerceptionModule humanoidPerception)
   {
      this.heightMapRenderer = new RDXHeightMapRenderer();
      this.heightMapRenderer.create(
            humanoidPerception.getRapidHeightMapExtractor().getCellsPerAxis() * humanoidPerception.getRapidHeightMapExtractor().getCellsPerAxis());
   }

   public void initializeHeightMapUI(ROS2Helper ros2Helper)
   {
      heightMapUI = new RDXRemoteHeightMapPanel(ros2Helper);
   }


   public void renderImGuiWidgets()
   {
      rapidRegionsUI.renderImGuiWidgets();
   }

   public void render(RigidBodyTransform zUpFrameToWorld)
   {
      heightMapRenderer.update(zUpFrameToWorld,
            humanoidPerception.getRapidHeightMapExtractor().getOutputHeightMapImage().getPointerForAccessSpeed(),
            humanoidPerception.getRapidHeightMapExtractor().getGridCenter(),
            humanoidPerception.getRapidHeightMapExtractor().getCenterIndex(),
            humanoidPerception.getRapidHeightMapExtractor().getCellSizeXYInMeters());

      PerceptionDebugTools.displayHeightMap("Output Height Map",
                                humanoidPerception.getRapidHeightMapExtractor().getOutputHeightMapImage().getBytedecoOpenCVMat(),
                                1, 1 / (0.3f + 0.20f * humanoidPerception.getRapidHeightMapExtractor().getCellSizeXYInMeters()));
      rapidRegionsUI.render();
   }

   public void initializeMapRegionsVisualizer(ROS2Node ros2Node, RDXGlobalVisualizersPanel globalVisualizersUI, boolean render)
   {
      rapidRegionsMapVisualizer = new RDXROS2PlanarRegionsVisualizer("SLAM Rapid Regions", ros2Node, PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS);
      rapidRegionsMapVisualizer.setActive(render);
      globalVisualizersUI.addVisualizer(rapidRegionsMapVisualizer);
   }

   public void initializePerspectiveRegionsVisualizer(ROS2Node ros2Node, RDXGlobalVisualizersPanel globalVisualizersUI, boolean render)
   {
      rapidRegionsVisualizer = new RDXROS2FramePlanarRegionsVisualizer("Rapid Regions", ros2Node, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS);
      rapidRegionsVisualizer.setActive(render);
      globalVisualizersUI.addVisualizer(rapidRegionsVisualizer);
   }

   public void initializeHeightMapVisualizer(ROS2Helper ros2Helper, RDXGlobalVisualizersPanel globalVisualizersUI, boolean render)
   {
      RDXHeightMapVisualizer heightMapVisualizer = new RDXHeightMapVisualizer();
      heightMapVisualizer.setupForNetworking(ros2Helper);
      heightMapVisualizer.setActive(render);
      globalVisualizersUI.addVisualizer(heightMapVisualizer);
   }

   public RDXRapidRegionsUI getRapidRegionsUI()
   {
      return rapidRegionsUI;
   }

   public RDXRemotePerceptionUI getRemotePerceptionUI()
   {
      return remotePerceptionUI;
   }

   public RDXActiveMappingUI getActiveMappingUI()
   {
      return activeMappingUI;
   }

   public RDXHeightMapRenderer getHeightMapRenderer()
   {
      return heightMapRenderer;
   }

   public float getThresholdHeight()
   {
      return remotePerceptionUI.getThresholdHeight();
   }

   public void destroy()
   {
      rapidRegionsUI.destroy();
      rapidRegionsVisualizer.destroy();
      rapidRegionsMapVisualizer.destroy();
   }
}
