package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.headless.HumanoidPerceptionModule;
import us.ihmc.rdx.ui.graphics.ros2.RDXHeightMapVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2FramePlanarRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2PlanarRegionsVisualizer;
import us.ihmc.rdx.ui.visualizers.RDXGlobalVisualizersPanel;
import us.ihmc.ros2.ROS2Node;

public class RDXHumanoidPerceptionUI
{
   private final RDXRemotePerceptionUI remotePerceptionUI;

   private HumanoidPerceptionModule humanoidPerception;
   private RDXRapidRegionsUI rapidRegionsUI;
   private RDXActiveMappingUI activeMappingUI;
   private RDXROS2PlanarRegionsVisualizer rapidRegionsMapVisualizer;
   private RDXROS2FramePlanarRegionsVisualizer perspectiveRegionsVisualizer;
   private RDXROS2PlanarRegionsVisualizer sphericalRegionsVisualizer;
   private RDXRemoteHeightMapPanel heightMapUI;
   private RDXHeightMapVisualizer heightMapVisualizer;

   public RDXHumanoidPerceptionUI(HumanoidPerceptionModule humanoidPerception, ROS2Helper ros2Helper)
   {
      this.remotePerceptionUI = new RDXRemotePerceptionUI(ros2Helper);

      if (humanoidPerception != null)
      {
         this.humanoidPerception = humanoidPerception;
         this.humanoidPerception.setPerceptionConfigurationParameters(remotePerceptionUI.getPerceptionConfigurationParameters());
      }
   }

   public void initializeRapidRegionsUI()
   {
      this.rapidRegionsUI = new RDXRapidRegionsUI();
      this.rapidRegionsUI.create(humanoidPerception.getRapidRegionsExtractor());
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
      heightMapVisualizer = new RDXHeightMapVisualizer();
      heightMapVisualizer.setActive(render);

      if (globalVisualizersUI != null)
      {
         heightMapVisualizer.setupForHeightMapMessage(ros2Helper);
         heightMapVisualizer.setupForImageMessage(ros2Helper);
         globalVisualizersUI.addVisualizer(heightMapVisualizer);
      }
   }

   public void destroy()
   {
      if (remotePerceptionUI != null)
         remotePerceptionUI.destroy();

      if (rapidRegionsUI != null)
         rapidRegionsUI.destroy();

      if (activeMappingUI != null)
         activeMappingUI.destroy();

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

   public RDXHeightMapVisualizer getHeightMapVisualizer()
   {
      return heightMapVisualizer;
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
