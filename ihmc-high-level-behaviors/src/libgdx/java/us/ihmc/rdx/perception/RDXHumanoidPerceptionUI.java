package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCond;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.gpuHeightMap.HeatMapGenerator;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.headless.HumanoidPerceptionModule;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2HeightMapVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2FramePlanarRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2PlanarRegionsVisualizer;
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
   private RDXRemoteHeightMapPanel heightMapUI;

   private HeatMapGenerator contactHeatMapGenerator = new HeatMapGenerator();

   /* Image panel to display the local height map */
   private RDXBytedecoImagePanel localHeightMapPanel;

   /* Image panel to display the internal (full) height map */
   private RDXBytedecoImagePanel internalHeightMapPanel;

   /* Image panel to display the steppable height map */
   private RDXBytedecoImagePanel snappedHeightMapPanel;
   private RDXBytedecoImagePanel steppabilityPanel;
   private RDXBytedecoImagePanel snapNormalZMapPanel;

   /* Image panel to display the internal (cropped) height map */
   private RDXBytedecoImagePanel croppedHeightMapPanel;

   /* Image panel to display the depth image */
   private RDXBytedecoImagePanel depthImagePanel;

   /* Image panel to display the terrain cost map (16-bit scalar metric for steppability cost per cell,
    *  computed based on terrain inclination and continuity) */
   private RDXBytedecoImagePanel terrainCostImagePanel;
   private RDXBytedecoImagePanel steppableRegionAssignmentMat;
   private RDXBytedecoImagePanel steppableRegionBordersMat;
   private RDXBytedecoImagePanel steppabilityConnectionsImage;

   /* Image panel to display the feasible contact map (16-bit scalar for distance transform of the terrain cost map, represents safety score
    *  for distance away from boundaries and edges for each cell). For more information on Distance Transform visit:
    *  https://en.wikipedia.org/wiki/Distance_transform */
   private RDXBytedecoImagePanel contactMapImagePanel;

   private RDXTerrainGridGraphic terrainGridGraphic;

   private final ImBoolean rapidRegionsCollapsedHeader = new ImBoolean(true);
   private final ImBoolean sphericalRegionsCollapsedHeader = new ImBoolean(true);
   private final ImBoolean heightMapCollapsedHeader = new ImBoolean(true);
   private final ImBoolean mapRegionsCollapsedHeader = new ImBoolean(true);

   private final ImBoolean enableRapidRegions = new ImBoolean(false);
   private final ImBoolean enableSphericalRapidRegions = new ImBoolean(false);
   private final ImBoolean enableGPUHeightMap = new ImBoolean(false);
   private final ImBoolean enableMapRegions = new ImBoolean(false);

   private ImFloat thresholdHeight = new ImFloat(1.0f);

   public RDXHumanoidPerceptionUI(HumanoidPerceptionModule humanoidPerception, ROS2Helper ros2Helper)
   {
      super(WINDOW_NAME);
      setRenderMethod(this::renderImGuiWidgets);
      this.remotePerceptionUI = new RDXRemotePerceptionUI(ros2Helper, this);
      this.terrainGridGraphic = new RDXTerrainGridGraphic();

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

   public void initializeMapRegionsVisualizer(ROS2Node ros2Node)
   {
      RDXROS2PlanarRegionsVisualizer mapRegionsVisualizer = new RDXROS2PlanarRegionsVisualizer("SLAM Rapid Regions",
                                                                                               ros2Node,
                                                                                               PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS);
      mapRegionsVisualizer.create();
      visualizers.put("MapRegions", mapRegionsVisualizer);
   }

   public void initializePerspectiveRegionsVisualizer(ROS2Node ros2Node)
   {
      RDXROS2FramePlanarRegionsVisualizer perspectiveRegionsVisualizer = new RDXROS2FramePlanarRegionsVisualizer("Rapid Regions",
                                                                                                                 ros2Node,
                                                                                                                 PerceptionAPI.PERSPECTIVE_RAPID_REGIONS);
      perspectiveRegionsVisualizer.create();
      visualizers.put("PerspectiveRegions", perspectiveRegionsVisualizer);
   }

   public void initializeSphericalRegionsVisualizer(ROS2Node ros2Node)
   {
      RDXROS2FramePlanarRegionsVisualizer sphericalRegionsVisualizer = new RDXROS2FramePlanarRegionsVisualizer("Structural Rapid Regions",
                                                                                                     ros2Node,
                                                                                                     PerceptionAPI.SPHERICAL_RAPID_REGIONS_WITH_POSE);
      sphericalRegionsVisualizer.create();
      visualizers.put("SphericalRegions", sphericalRegionsVisualizer);
   }

   public void initializeHeightMapVisualizer(ROS2Helper ros2Helper)
   {
      terrainGridGraphic.create();
      RDXROS2HeightMapVisualizer heightMapVisualizer = new RDXROS2HeightMapVisualizer("Height Map");
      if (ros2Helper != null)
      {
         heightMapVisualizer.setupForHeightMapMessage(ros2Helper);
         heightMapVisualizer.setupForImageMessage(ros2Helper);
      }
      heightMapVisualizer.create();
      visualizers.put("HeightMap", heightMapVisualizer);
   }

   public void update(TerrainMapData terrainMapData)
   {
      Mat contactHeatMapImage = contactHeatMapGenerator.generateHeatMap(terrainMapData.getContactMap());
      croppedHeightMapPanel.drawDepthImage(terrainMapData.getHeightMap());
      contactMapImagePanel.drawColorImage(contactHeatMapImage);
      terrainGridGraphic.update(humanoidPerception.getRapidHeightMapExtractor().getCurrentGroundToWorldTransform());

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

   public void update()
   {
      if (heightMapUI != null)
      {
         heightMapUI.update();
      }

      if (humanoidPerception != null)
      {
         if (humanoidPerception.getRapidHeightMapExtractor().isInitialized())
         {
            TerrainMapData terrainMap = humanoidPerception.getRapidHeightMapExtractor().getTerrainMapData();
            Mat contactHeatMapImage = contactHeatMapGenerator.generateHeatMap(terrainMap.getContactMap());
            croppedHeightMapPanel.drawDepthImage(terrainMap.getHeightMap());
            terrainCostImagePanel.drawDepthImage(terrainMap.getTerrainCostMap());
            contactMapImagePanel.drawColorImage(contactHeatMapImage);

            //internalHeightMapPanel.drawDepthImage(humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().getBytedecoOpenCVMat());
            depthImagePanel.drawDepthImage(humanoidPerception.getRealsenseDepthImage().getBytedecoOpenCVMat());
            localHeightMapPanel.drawDepthImage(humanoidPerception.getRapidHeightMapExtractor().getLocalHeightMapImage().getBytedecoOpenCVMat());

            if (humanoidPerception.getRapidHeightMapExtractor().getComputeSnap())
            {
               snapNormalZMapPanel.drawDepthImage(humanoidPerception.getRapidHeightMapExtractor().getSnapNormalZImage().getBytedecoOpenCVMat());
               snappedHeightMapPanel.drawDepthImage(humanoidPerception.getRapidHeightMapExtractor().getSteppableHeightMapImage().getBytedecoOpenCVMat());
               steppabilityPanel.drawDepthImage(humanoidPerception.getRapidHeightMapExtractor().getSteppabilityImage().getBytedecoOpenCVMat());
               steppableRegionAssignmentMat.drawDepthImage(humanoidPerception.getRapidHeightMapExtractor().getSteppableRegionAssignmentMat());
               steppableRegionBordersMat.drawDepthImage(humanoidPerception.getRapidHeightMapExtractor().getSteppableRegionRingMat());
               steppabilityConnectionsImage.drawDepthImage(humanoidPerception.getRapidHeightMapExtractor().getSteppabilityConnectionsImage().getBytedecoOpenCVMat());
            }

            terrainGridGraphic.update(humanoidPerception.getRapidHeightMapExtractor().getCurrentGroundToWorldTransform());
         }
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
      remotePerceptionUI.setPropertyChanged();

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
         terrainGridGraphic.renderImGuiWidgets();
         RDXROS2HeightMapVisualizer heightMapVisualizer = (RDXROS2HeightMapVisualizer) visualizers.get("HeightMap");
         if (heightMapVisualizer != null)
         {
            heightMapVisualizer.renderImGuiWidgets();
            heightMapUI.renderImGuiWidgets();
            ImGui.sliderFloat("Threshold Height", thresholdHeight.getData(), 0.0f, 2.0f);
            if (ImGui.button("Print Local Height Map"))
            {
               PerceptionDebugTools.printMat("Local Height Map",
                                             humanoidPerception.getRapidHeightMapExtractor().getLocalHeightMapImage().getBytedecoOpenCVMat(),
                                             4);
            }
            if (ImGui.button("Print Cropped Height Map"))
            {
               PerceptionDebugTools.printMat("Cropped Height Map",
                                             humanoidPerception.getRapidHeightMapExtractor().getTerrainMapData().getHeightMap(),4);
            }
            if (ImGui.button("Print Internal Height Map"))
            {
               PerceptionDebugTools.printMat("Internal Height Map",
                                             humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().getBytedecoOpenCVMat(),
                                             4);
            }
            if (ImGui.button("Print Internal Contact Map"))
            {
               PerceptionDebugTools.printMat("Internal Contact Map",
                                             humanoidPerception.getRapidHeightMapExtractor().getGlobalContactImage(),
                                             4);
            }
            if (ImGui.button("Print Terrain Cost Image"))
            {
               PerceptionDebugTools.printMat("Terrain Cost Image", humanoidPerception.getRapidHeightMapExtractor().getTerrainMapData().getTerrainCostMap(), 4);
            }
            if (ImGui.button("Print Contact Map Image"))
            {
               PerceptionDebugTools.printMat("Contact Map Image", humanoidPerception.getRapidHeightMapExtractor().getTerrainMapData().getContactMap(), 4);
            }

//            if (humanoidPerception.getRapidHeightMapExtractor().getComputeSnap())
//            {
//               if (ImGui.button("Print Cropped Snapped Height Map"))
//               {
//                  PerceptionDebugTools.printMat("Cropped Snapped Height Map",
//                                                humanoidPerception.getRapidHeightMapExtractor().getSnapNormalZImage().getBytedecoOpenCVMat(),4);
//               }
//            }
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

      if (humanoidPerception != null)
      {
         //internalHeightMapPanel = new RDXBytedecoImagePanel("Internal Height Map",
         //                                                   humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().getImageWidth(),
         //                                                   humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().getImageHeight(),
         //                                                   RDXImagePanel.DO_NOT_FLIP_Y);

         depthImagePanel = new RDXBytedecoImagePanel("Depth Image",
                                                     RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize(),
                                                     RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize(),
                                                     RDXImagePanel.DO_NOT_FLIP_Y);
         localHeightMapPanel = new RDXBytedecoImagePanel("Local Height Map",
                                                         RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize(),
                                                         RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize(),
                                                         RDXImagePanel.FLIP_Y);
         croppedHeightMapPanel = new RDXBytedecoImagePanel("Cropped Height Map",
                                                           RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize(),
                                                           RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize(),
                                                           RDXImagePanel.FLIP_Y);
         snapNormalZMapPanel = new RDXBytedecoImagePanel("Normal Z",
                                                                  humanoidPerception.getRapidHeightMapExtractor().getSnapNormalZImage().getBytedecoOpenCVMat().cols(),
                                                                  humanoidPerception.getRapidHeightMapExtractor().getSnapNormalZImage().getBytedecoOpenCVMat().rows(),
                                                                  RDXImagePanel.FLIP_Y);
         steppableRegionAssignmentMat = new RDXBytedecoImagePanel("Region Assignment",
                                                         humanoidPerception.getRapidHeightMapExtractor().getSteppableRegionAssignmentMat().cols(),
                                                         humanoidPerception.getRapidHeightMapExtractor().getSteppableRegionAssignmentMat().rows(),
                                                         RDXImagePanel.FLIP_Y);
         steppableRegionBordersMat = new RDXBytedecoImagePanel("Region Borders",
                                                                  humanoidPerception.getRapidHeightMapExtractor().getSteppableRegionRingMat().cols(),
                                                                  humanoidPerception.getRapidHeightMapExtractor().getSteppableRegionRingMat().rows(),
                                                                  RDXImagePanel.FLIP_Y);
         steppabilityConnectionsImage = new RDXBytedecoImagePanel("Region Connections",
                                                                  humanoidPerception.getRapidHeightMapExtractor().getSteppabilityConnectionsImage().getBytedecoOpenCVMat().cols(),
                                                                  humanoidPerception.getRapidHeightMapExtractor().getSteppabilityConnectionsImage().getBytedecoOpenCVMat().rows(),
                                                                  RDXImagePanel.FLIP_Y);
         //sensorCroppedHeightMapPanel = new RDXBytedecoImagePanel("Sensor Cropped Height Map",
         //                                                        humanoidPerception.getRapidHeightMapExtractor().getSensorCroppedHeightMapImage().cols(),
         //                                                        humanoidPerception.getRapidHeightMapExtractor().getSensorCroppedHeightMapImage().rows(),
         //                                                        RDXImagePanel.DO_NOT_FLIP_Y);
         terrainCostImagePanel = new RDXBytedecoImagePanel("Terrain Cost Image",
                                                           RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize(),
                                                           RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize(),
                                                           RDXImagePanel.FLIP_Y);
         contactMapImagePanel = new RDXBytedecoImagePanel("Contact Map Image",
                                                          RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize(),
                                                          RapidHeightMapExtractor.getHeightMapParameters().getCropWindowSize(),
                                                          RDXImagePanel.FLIP_Y);

         addChild(localHeightMapPanel.getImagePanel());
         addChild(croppedHeightMapPanel.getImagePanel());
         addChild(snapNormalZMapPanel.getImagePanel());
         addChild(steppableRegionAssignmentMat.getImagePanel());
         addChild(steppableRegionBordersMat.getImagePanel());
         addChild(steppabilityConnectionsImage.getImagePanel());
         addChild(depthImagePanel.getImagePanel());
         //addChild(internalHeightMapPanel.getImagePanel());
         addChild(terrainCostImagePanel.getImagePanel());
         addChild(contactMapImagePanel.getImagePanel());

         if (humanoidPerception.getRapidHeightMapExtractor().getComputeSnap())
         {
            snappedHeightMapPanel = new RDXBytedecoImagePanel("Snapped Height Map",
                                                              humanoidPerception.getRapidHeightMapExtractor().getSteppableHeightMapImage().getImageWidth(),
                                                              humanoidPerception.getRapidHeightMapExtractor().getSteppableHeightMapImage().getImageHeight(),
                                                              RDXImagePanel.FLIP_Y);
            steppabilityPanel = new RDXBytedecoImagePanel("Steppability",
                                                          humanoidPerception.getRapidHeightMapExtractor().getSteppabilityImage().getImageWidth(),
                                                          humanoidPerception.getRapidHeightMapExtractor().getSteppabilityImage().getImageHeight(),
                                                          RDXImagePanel.FLIP_Y);
            snapNormalZMapPanel = new RDXBytedecoImagePanel("Normal Z",
                                                            humanoidPerception.getRapidHeightMapExtractor().getSnapNormalZImage().getBytedecoOpenCVMat().cols(),
                                                            humanoidPerception.getRapidHeightMapExtractor().getSnapNormalZImage().getBytedecoOpenCVMat().rows(),
                                                            RDXImagePanel.FLIP_Y);

            addChild(snapNormalZMapPanel.getImagePanel());
            addChild(snappedHeightMapPanel.getImagePanel());
            addChild(steppabilityPanel.getImagePanel());
         }
      }
   }

   public RDXRapidRegionsUI getRapidRegionsUI()
   {
      return rapidRegionsUI;
   }

   public RDXRemotePerceptionUI getRemotePerceptionUI()
   {
      return remotePerceptionUI;
   }

   public RDXROS2HeightMapVisualizer getHeightMapVisualizer()
   {
      return (RDXROS2HeightMapVisualizer) visualizers.get("HeightMap");
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
      terrainGridGraphic.getRenderables(renderables, pool);
      for (RDXVisualizer visualizer : visualizers.values())
      {
         if (visualizer.isActive())
         {
            visualizer.getRenderables(renderables, pool, sceneLevels);
         }
      }
   }
}
