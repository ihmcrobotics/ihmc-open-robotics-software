package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCond;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.gpuHeightMap.HeatMapGenerator;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.headless.HumanoidPerceptionModule;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXImagePanel;

import java.util.Set;

public class RDXHumanoidPerceptionUI extends RDXPanel implements RDXRenderableProvider
{
   private static final String WINDOW_NAME = "Perception Settings";
   private final RDXRemotePerceptionUI remotePerceptionUI;

   private HumanoidPerceptionModule humanoidPerception;

   private RDXRapidRegionsUI rapidRegionsUI;
   private RDXRemoteHeightMapPanel heightMapUI;

   private final HeatMapGenerator contactHeatMapGenerator = new HeatMapGenerator();

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

   private final RDXTerrainGridGraphic terrainGridGraphic;

   private final ImBoolean rapidRegionsCollapsedHeader = new ImBoolean(true);
   private final ImBoolean sphericalRegionsCollapsedHeader = new ImBoolean(true);
   private final ImBoolean heightMapCollapsedHeader = new ImBoolean(true);
   private final ImBoolean mapRegionsCollapsedHeader = new ImBoolean(true);

   private final ImBoolean enableRapidRegions = new ImBoolean(false);
   private final ImBoolean enableSphericalRapidRegions = new ImBoolean(false);
   private final ImBoolean enableGPUHeightMap = new ImBoolean(false);
   private final ImBoolean enableMapRegions = new ImBoolean(false);

   private final ImFloat thresholdHeight = new ImFloat(1.0f);

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

   public void update(TerrainMapData terrainMapData)
   {
      Mat contactHeatMapImage = contactHeatMapGenerator.generateHeatMap(terrainMapData.getContactMap());
      croppedHeightMapPanel.drawDepthImage(terrainMapData.getHeightMap());
      contactMapImagePanel.drawColorImage(contactHeatMapImage);
      terrainGridGraphic.update(humanoidPerception.getRapidHeightMapExtractor().getCurrentGroundToWorldTransform());
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
         ImGui.unindent();
      }

      if (ImGui.collapsingHeader("Spherical Rapid Regions", sphericalRegionsCollapsedHeader))
      {
         ImGui.indent();
         ImGui.checkbox("Enable Spherical Rapid Regions", enableSphericalRapidRegions);
         ImGui.unindent();
      }

      // Set the next header to remain open by default
      ImGui.setNextItemOpen(true, ImGuiCond.Once);
      if (ImGui.collapsingHeader("GPU Height Map", heightMapCollapsedHeader))
      {
         ImGui.indent();
         ImGui.checkbox("Enable GPU Height Map", enableGPUHeightMap);
         terrainGridGraphic.renderImGuiWidgets();

         ImGui.unindent();
      }

      if (ImGui.collapsingHeader("Map Regions", mapRegionsCollapsedHeader))
      {
         ImGui.indent();
         ImGui.checkbox("Enable Map Regions", enableMapRegions);
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
   }
}
