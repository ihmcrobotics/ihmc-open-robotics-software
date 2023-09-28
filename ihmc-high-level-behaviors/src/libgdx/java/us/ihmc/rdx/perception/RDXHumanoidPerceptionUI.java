package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCond;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.headless.HumanoidPerceptionModule;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXHeightMapVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2FramePlanarRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2PlanarRegionsVisualizer;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.ros2.ROS2Node;

import java.util.HashMap;
import java.util.Set;

public class RDXHumanoidPerceptionUI extends RDXPanel implements RDXRenderableProvider
{
   private static final String WINDOW_NAME = "Perception Panel";
   private final RDXRemotePerceptionUI remotePerceptionUI;
   private final HashMap<String, RDXVisualizer> visualizers = new HashMap<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private HumanoidPerceptionModule humanoidPerception;
   private RDXRapidRegionsUI rapidRegionsUI;
   private RDXActiveMappingUI activeMappingUI;
   private RDXRemoteHeightMapPanel heightMapUI;

   private RDXBytedecoImagePanel localHeightMapPanel;
   private RDXBytedecoImagePanel internalHeightMapPanel;
   private RDXBytedecoImagePanel croppedHeightMapPanel;
   private RDXBytedecoImagePanel depthImagePanel;

   private final ImBoolean rapidRegionsCollapsedHeader = new ImBoolean(true);
   private final ImBoolean sphericalRegionsCollapsedHeader = new ImBoolean(true);
   private final ImBoolean heightMapCollapsedHeader = new ImBoolean(true);
   private final ImBoolean mapRegionsCollapsedHeader = new ImBoolean(true);

   private final ImBoolean enableRapidRegions = new ImBoolean(false);
   private final ImBoolean enableSphericalRapidRegions = new ImBoolean(false);
   private final ImBoolean enableGPUHeightMap = new ImBoolean(true);
   private final ImBoolean enableMapRegions = new ImBoolean(false);

   private ImFloat thresholdHeight = new ImFloat(1.0f);

   public RDXHumanoidPerceptionUI(HumanoidPerceptionModule humanoidPerception, ROS2Helper ros2Helper)
   {
      super(WINDOW_NAME);
      setRenderMethod(this::renderImGuiWidgets);
      this.remotePerceptionUI = new RDXRemotePerceptionUI(ros2Helper, this);

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

   public void intializeActiveMappingUI(ROS2Helper ros2Helper)
   {
      this.activeMappingUI = new RDXActiveMappingUI("Active Mapping", ros2Helper);
   }

   public void initializeMapRegionsVisualizer(ROS2Node ros2Node)
   {
      RDXROS2PlanarRegionsVisualizer mapRegionsVisualizer = new RDXROS2PlanarRegionsVisualizer("SLAM Rapid Regions",
                                                                                                    ros2Node,
                                                                                                    PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS);
      mapRegionsVisualizer.setActive(true);
      mapRegionsVisualizer.create();
      visualizers.put("MapRegions", mapRegionsVisualizer);
   }

   public void initializePerspectiveRegionsVisualizer(ROS2Node ros2Node)
   {
      RDXROS2FramePlanarRegionsVisualizer perspectiveRegionsVisualizer = new RDXROS2FramePlanarRegionsVisualizer("Rapid Regions",
                                                                                                                 ros2Node,
                                                                                                                 PerceptionAPI.PERSPECTIVE_RAPID_REGIONS);
      perspectiveRegionsVisualizer.setActive(true);
      perspectiveRegionsVisualizer.create();
      visualizers.put("PerspectiveRegions", perspectiveRegionsVisualizer);
   }

   public void initializeSphericalRegionsVisualizer(ROS2Node ros2Node)
   {
      RDXROS2PlanarRegionsVisualizer sphericalRegionsVisualizer = new RDXROS2PlanarRegionsVisualizer("Structural Rapid Regions",
                                                                                                     ros2Node,
                                                                                                     PerceptionAPI.SPHERICAL_RAPID_REGIONS);
      sphericalRegionsVisualizer.setActive(true);
      sphericalRegionsVisualizer.create();
      visualizers.put("SphericalRegions", sphericalRegionsVisualizer);
   }

   public void initializeHeightMapVisualizer(ROS2Helper ros2Helper)
   {
      RDXHeightMapVisualizer heightMapVisualizer = new RDXHeightMapVisualizer();
      heightMapVisualizer.setActive(true);
      if (ros2Helper != null)
      {
         heightMapVisualizer.setupForHeightMapMessage(ros2Helper);
         heightMapVisualizer.setupForImageMessage(ros2Helper);
      }
      heightMapVisualizer.create();
      visualizers.put("HeightMap", heightMapVisualizer);
   }

   public void update()
   {
      if (heightMapUI != null)
      {
         heightMapUI.update();

         depthImagePanel.drawDepthImage(humanoidPerception.getRealsenseDepthImage().getBytedecoOpenCVMat());
         localHeightMapPanel.drawDepthImage(humanoidPerception.getRapidHeightMapExtractor().getLocalHeightMapImage().getBytedecoOpenCVMat());
         croppedHeightMapPanel.drawDepthImage(humanoidPerception.getRapidHeightMapExtractor().getCroppedGlobalHeightMapImage());
         internalHeightMapPanel.drawDepthImage(humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().getBytedecoOpenCVMat());
      }

      if (rapidRegionsUI != null)
         rapidRegionsUI.render();

      if (humanoidPerception != null)
      {
         humanoidPerception.setMappingEnabled(enableMapRegions.get());
         humanoidPerception.setRapidRegionsEnabled(enableRapidRegions.get());
         humanoidPerception.setHeightMapEnabled(enableGPUHeightMap.get());
         //humanoidPerception.setSphericalRegionsEnabled(enableSphericalRapidRegions.get());
      }

      for (RDXVisualizer visualizer : visualizers.values())
      {
         if (visualizer.getPanel() != null)
            visualizer.getPanel().getIsShowing().set(visualizer.isActive());
         if (visualizer.isActive())
         {
            visualizer.update();
         }
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.collapsingHeader("Configuration"))
      {
         ImGui.indent();
         remotePerceptionUI.renderImGuiWidgets();
         ImGui.unindent();
      }

      if (ImGui.collapsingHeader("Rapid Regions", rapidRegionsCollapsedHeader))
      {
         ImGui.indent();
         ImGui.checkbox("Enable Rapid Regions", enableRapidRegions);
         RDXVisualizer visualizer = visualizers.get("PerspectiveRegions");
         if (visualizer != null)
         {
            visualizers.get("PerspectiveRegions").renderImGuiWidgets();
         }
         ImGui.unindent();
      }

      if (ImGui.collapsingHeader("Spherical Rapid Regions", sphericalRegionsCollapsedHeader))
      {
         ImGui.indent();
         ImGui.checkbox("Enable Spherical Rapid Regions", enableSphericalRapidRegions);
         RDXVisualizer visualizer = visualizers.get("SphericalRegions");
         if (visualizer != null)
         {
            visualizers.get("SphericalRegions").renderImGuiWidgets();
         }
         ImGui.unindent();
      }

      // Set the next header to remain open by default
      ImGui.setNextItemOpen(true, ImGuiCond.Once);
      if (ImGui.collapsingHeader("GPU Height Map", heightMapCollapsedHeader))
      {
         ImGui.indent();
         ImGui.checkbox("Enable GPU Height Map", enableGPUHeightMap);
         RDXHeightMapVisualizer heightMapVisualizer = (RDXHeightMapVisualizer) visualizers.get("HeightMap");
         if (heightMapVisualizer != null)
         {
            heightMapVisualizer.renderImGuiWidgets();
            heightMapUI.renderImGuiWidgets();
            ImGui.sliderFloat("Threshold Height", thresholdHeight.getData(), 0.0f, 2.0f);
         }
         ImGui.unindent();
      }

      if (ImGui.collapsingHeader("Map Regions", mapRegionsCollapsedHeader))
      {
         ImGui.indent();
         ImGui.checkbox("Enable Map Regions", enableMapRegions);
         RDXVisualizer visualizer = visualizers.get("MapRegions");
         if (visualizer != null)
         {
            visualizers.get("MapRegions").renderImGuiWidgets();
         }
         ImGui.unindent();
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

      for (RDXVisualizer visualizer : visualizers.values())
      {
         visualizer.destroy();
      }
   }

   public void initializeHeightMapUI(ROS2Helper ros2Helper)
   {
      heightMapUI = new RDXRemoteHeightMapPanel(ros2Helper);

      internalHeightMapPanel = new RDXBytedecoImagePanel("Internal Height Map",
                                                         humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().getImageWidth(),
                                                         humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().getImageHeight(),
                                                         RDXImagePanel.FLIP_Y);
      depthImagePanel = new RDXBytedecoImagePanel("Depth Image",
                                                  humanoidPerception.getRealsenseDepthImage().getBytedecoOpenCVMat().cols(),
                                                  humanoidPerception.getRealsenseDepthImage().getBytedecoOpenCVMat().rows(),
                                                  RDXImagePanel.DO_NOT_FLIP_Y);
      localHeightMapPanel = new RDXBytedecoImagePanel("Local Height Map",
                                                      humanoidPerception.getRapidHeightMapExtractor().getLocalHeightMapImage().getImageWidth(),
                                                      humanoidPerception.getRapidHeightMapExtractor().getLocalHeightMapImage().getImageHeight(),
                                                      RDXImagePanel.FLIP_Y);
      croppedHeightMapPanel = new RDXBytedecoImagePanel("Cropped Height Map",
                                                        humanoidPerception.getRapidHeightMapExtractor().getCroppedGlobalHeightMapImage().cols(),
                                                        humanoidPerception.getRapidHeightMapExtractor().getCroppedGlobalHeightMapImage().rows(),
                                                        RDXImagePanel.FLIP_Y);

      addChild(localHeightMapPanel.getImagePanel());
      addChild(croppedHeightMapPanel.getImagePanel());
      addChild(depthImagePanel.getImagePanel());
      addChild(internalHeightMapPanel.getImagePanel());
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
      return (RDXHeightMapVisualizer) visualizers.get("HeightMap");
   }

   public float getThresholdHeight()
   {
      return thresholdHeight.get();
   }

   public RDXRemoteHeightMapPanel getHeightMapUI()
   {
      return heightMapUI;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      for (RDXVisualizer visualizer : visualizers.values())
      {
         if (visualizer.isActive())
         {
            visualizer.getRenderables(renderables, pool, sceneLevels);
         }
      }

      if (activeMappingUI != null)
      {
         activeMappingUI.getRenderables(renderables, pool);
      }
   }
}
