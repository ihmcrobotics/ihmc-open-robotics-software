package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.headless.HumanoidPerceptionModule;
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
   private RDXROS2FramePlanarRegionsVisualizer perspectiveRegionsVisualizer;
   private RDXROS2PlanarRegionsVisualizer sphericalRegionsVisualizer;
   private RDXHeightMapRenderer heightMapRenderer;
   private RDXRemoteHeightMapPanel heightMapUI;

   public RDXHumanoidPerceptionUI(HumanoidPerceptionModule humanoidPerception, ROS2Helper ros2Helper)
   {
      this.remotePerceptionUI = new RDXRemotePerceptionUI(ros2Helper);

      if (humanoidPerception != null)
      {
         this.humanoidPerception = humanoidPerception;
         this.humanoidPerception.setPerceptionConfigurationParameters(remotePerceptionUI.getPerceptionConfigurationParameters());
      }
   }

   public void initializeHeightMapRenderer(HumanoidPerceptionModule humanoidPerception)
   {
      this.heightMapRenderer = new RDXHeightMapRenderer();
      this.heightMapRenderer.create(
            humanoidPerception.getRapidHeightMapExtractor().getGlobalCellsPerAxis() * humanoidPerception.getRapidHeightMapExtractor().getGlobalCellsPerAxis());
   }

   public void initializeRapidRegionsUI()
   {
      this.rapidRegionsUI = new RDXRapidRegionsUI();
      this.rapidRegionsUI.create(humanoidPerception.getRapidRegionsExtractor());
   }

   public void render(RigidBodyTransform zUpFrameToWorld)
   {
      if (heightMapRenderer != null)
      {
         heightMapRenderer.update(zUpFrameToWorld,
                                  humanoidPerception.getRapidHeightMapExtractor().getGlobalHeightMapImage().getPointerForAccessSpeed(),
                                  humanoidPerception.getRapidHeightMapExtractor().getGlobalCenterIndex(),
                                  humanoidPerception.getRapidHeightMapExtractor().getGlobalCellSizeInMeters());

         PerceptionDebugTools.displayHeightMap("Output Height Map",
                                               humanoidPerception.getRapidHeightMapExtractor().getGlobalHeightMapImage().getBytedecoOpenCVMat(),
                                               1, 1 / (0.3f + 0.20f * humanoidPerception.getRapidHeightMapExtractor().getLocalCellSizeInMeters()));
      }

      if (rapidRegionsUI != null)
      {
         rapidRegionsUI.render();
      }
   }

   public void initializeMapRegionsVisualizer(ROS2Node ros2Node, RDXGlobalVisualizersPanel globalVisualizersUI, boolean render)
   {
      rapidRegionsMapVisualizer = new RDXROS2PlanarRegionsVisualizer("SLAM Rapid Regions", ros2Node, PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS);
      rapidRegionsMapVisualizer.setActive(render);
      globalVisualizersUI.addVisualizer(rapidRegionsMapVisualizer);
   }

   public void initializePerspectiveRegionsVisualizer(ROS2Node ros2Node, RDXGlobalVisualizersPanel globalVisualizersUI, boolean render)
   {
      perspectiveRegionsVisualizer = new RDXROS2FramePlanarRegionsVisualizer("Rapid Regions", ros2Node, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS);
      perspectiveRegionsVisualizer.setActive(render);
      globalVisualizersUI.addVisualizer(perspectiveRegionsVisualizer);
   }

   public void initializeSphericalRegionsVisualizer(ROS2Node ros2Node, RDXGlobalVisualizersPanel globalVisualizersUI, boolean render)
   {
      sphericalRegionsVisualizer = new RDXROS2PlanarRegionsVisualizer("Structural Rapid Regions", ros2Node, PerceptionAPI.SPHERICAL_RAPID_REGIONS);
      sphericalRegionsVisualizer.setActive(render);
      globalVisualizersUI.addVisualizer(sphericalRegionsVisualizer);
   }

   public void initializeHeightMapVisualizer(ROS2Helper ros2Helper, RDXGlobalVisualizersPanel globalVisualizersUI, boolean render)
   {
      RDXHeightMapVisualizer heightMapVisualizer = new RDXHeightMapVisualizer();
      heightMapVisualizer.setupForHeightMapMessage(ros2Helper);
      heightMapVisualizer.setupForImageMessage(ros2Helper);
      heightMapVisualizer.setActive(render);
      globalVisualizersUI.addVisualizer(heightMapVisualizer);
   }

   public void destroy()
   {
      if (remotePerceptionUI != null)
         remotePerceptionUI.destroy();

      if (rapidRegionsUI != null)
         rapidRegionsUI.destroy();

      if (activeMappingUI != null)
         activeMappingUI.destroy();

      if (heightMapRenderer != null)
         heightMapRenderer.dispose();

      if (heightMapUI != null)
         heightMapUI.destroy();

      if (perspectiveRegionsVisualizer != null)
         perspectiveRegionsVisualizer.destroy();

      if (sphericalRegionsVisualizer != null)
         sphericalRegionsVisualizer.destroy();

      if (rapidRegionsMapVisualizer != null)
         rapidRegionsMapVisualizer.destroy();
   }

   public void initializeHeightMapUI(ROS2Helper ros2Helper)
   {
      heightMapUI = new RDXRemoteHeightMapPanel(ros2Helper);
   }

   public void renderImGuiWidgets()
   {
      rapidRegionsUI.renderImGuiWidgets();
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

   public RDXRemoteHeightMapPanel getHeightMapUI()
   {
      return heightMapUI;
   }
}
