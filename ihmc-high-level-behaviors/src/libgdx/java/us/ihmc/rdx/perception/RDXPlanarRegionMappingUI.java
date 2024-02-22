package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import gnu.trove.map.TIntIntMap;
import gnu.trove.map.hash.TIntIntHashMap;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.perception.PlanarRegionMappingHandler;
import us.ihmc.perception.mapping.PlanarRegionMapStatistics;
import us.ihmc.perception.tools.PlaneRegistrationTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.visualizers.RDXLineGraphic;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarLandmarkList;

import java.util.ArrayList;

public class RDXPlanarRegionMappingUI implements RenderableProvider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXStoredPropertySetTuner mappingParametersTuner;
   private PlanarRegionMappingHandler mappingManager;
   private RDXPanel panel;
   private RDXFootstepPlanGraphic footstepPlanGraphic;

   private ImGuiPlot wholeAlgorithmDurationPlot;
   private ImGuiPlot quaternionAveragingDurationPlot;
   private ImGuiPlot factorGraphDurationPlot;
   private ImGuiPlot regionMergingDurationPlot;

   private final ImBoolean liveModeEnabled = new ImBoolean(true);
   private final ImBoolean renderEnabled = new ImBoolean(true);
   private final ImBoolean renderBoundingBoxEnabled = new ImBoolean(false);
   private final ImBoolean renderMatchLinesEnabled = new ImBoolean(true);
   private final ImBoolean renderPreviousEnabled = new ImBoolean(true);
   private final ImBoolean renderCurrentEnabled = new ImBoolean(true);
   private final ImBoolean renderMocapEnabled = new ImBoolean(false);
   private final ImBoolean renderSensorEnabled = new ImBoolean(false);

   private RDXLineGraphic matchLineMesh = new RDXLineGraphic(0.02f, Color.WHITE);
   private final RDXLineGraphic mocapGraphic = new RDXLineGraphic(0.02f, Color.YELLOW);
   private final RDXLineGraphic rootJointGraphic = new RDXLineGraphic(0.02f, Color.RED);

   private RDXPlanarRegionsGraphic previousRegionsGraphic = new RDXPlanarRegionsGraphic();
   private RDXPlanarRegionsGraphic currentRegionsGraphic = new RDXPlanarRegionsGraphic();
   private RDXPlanarRegionsGraphic mapRegionsGraphic;

   private ImBoolean renderPointCloud = new ImBoolean(false);
   private ImInt icpPreviousIndex = new ImInt(10);
   private ImInt icpCurrentIndex = new ImInt(14);

   public RDXPlanarRegionMappingUI(String name, PlanarRegionMappingHandler mappingManager, RDXPlanarRegionsGraphic mapGraphic)
   {
      this.mapRegionsGraphic = mapGraphic;
      this.mappingManager = mappingManager;
      panel = new RDXPanel(name, this::renderImGuiWidgets);
      mappingParametersTuner = new RDXStoredPropertySetTuner(mappingManager.getParameters().getTitle());
      mappingParametersTuner.create(mappingManager.getParameters());
      panel.addChild(mappingParametersTuner);

      wholeAlgorithmDurationPlot = new ImGuiPlot(labels.get("Whole algorithm duration"), 1000, 300, 50);
      quaternionAveragingDurationPlot = new ImGuiPlot(labels.get("ICP duration"), 1000, 300, 50);
      factorGraphDurationPlot = new ImGuiPlot(labels.get("ICP registration duration"), 1000, 300, 50);
      regionMergingDurationPlot = new ImGuiPlot(labels.get("Region merging duration"), 1000, 300, 50);

      footstepPlanGraphic = new RDXFootstepPlanGraphic(PlannerTools.createFootPolygons(0.2, 0.1, 0.08));
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.beginTabBar("Mapping"))
      {
         if (ImGui.beginTabItem("Map"))
         {
            PlanarRegionMapStatistics statistics = mappingManager.getPlanarRegionMap().getStatistics();
            quaternionAveragingDurationPlot.render(statistics.getRegistrationTime());
            factorGraphDurationPlot.render(statistics.getOptimizationTime());
            regionMergingDurationPlot.render(statistics.getMergingTime());
            wholeAlgorithmDurationPlot.render(statistics.getTotalProcessingTime());

            ImGui.checkbox("Render live mode", renderEnabled);
            if (ImGui.button("Load Next Set"))
            {
               mappingManager.nextButtonCallback();
            }
            ImGui.sameLine();
            if (ImGui.button("Perform Map Clean-up"))
               mappingManager.performMapCleanUp();
            if (ImGui.button("Auto Increment"))
               mappingManager.autoIncrementButtonCallback();
            ImGui.sameLine();
            if (ImGui.button("Pause"))
               mappingManager.pauseButtonCallback();
            if (ImGui.checkbox("Enable Live Mode", liveModeEnabled))
               mappingManager.setEnableLiveMode(liveModeEnabled.get());
            if (ImGui.button("Reset map"))
            {
               mapRegionsGraphic.clear();
               mapRegionsGraphic.update();
               mappingManager.resetMap();
            }
            if (ImGui.button("Hard reset map"))
            {
               mapRegionsGraphic.clear();
               mapRegionsGraphic.update();
               mappingManager.hardResetTheMap();
            }
            if (ImGui.button("Load and Evaluate"))
            {
               mappingManager.nextButtonCallback();

               FootstepDataListMessage message = mappingManager.getFootstepsMessage();

               if (message != null)
               {
                  footstepPlanGraphic.generateMeshesAsync(message, "Evaluation Footstep Plan");
                  footstepPlanGraphic.update();
               }
            }

            ImGui.checkbox("Render Bounding Box", renderBoundingBoxEnabled);

            ImGui.endTabItem();
         }

         if (ImGui.beginTabItem("Registration"))
         {
            if (ImGui.button("Load Previous"))
            {
               mappingManager.loadRegionsFromLogIntoPrevious(icpPreviousIndex.get());
               previousRegionsGraphic.generateMeshes(mappingManager.getPreviousRegions().getPlanarRegionsList());
               previousRegionsGraphic.update();
            }
            ImGui.sameLine();
            ImGui.sliderInt("Previous", icpPreviousIndex.getData(), 0, mappingManager.getTotalDepthCount() - 1);

            icpPreviousIndex.set(Math.min(icpPreviousIndex.get(), icpCurrentIndex.get() - 1));
            icpCurrentIndex.set(Math.max(1, icpCurrentIndex.get()));

            if (ImGui.button("Load Curernt"))
            {
               mappingManager.loadRegionsFromLogIntoCurrent(icpCurrentIndex.get());
               currentRegionsGraphic.generateMeshes(mappingManager.getCurrentRegions().getPlanarRegionsList());
               currentRegionsGraphic.update();
               drawMatches();
            }
            ImGui.sameLine();
            ImGui.sliderInt("Current", icpCurrentIndex.getData(), 1, mappingManager.getTotalDepthCount() - 1);
            if (ImGui.button("Optimize Transform"))
            {
               mappingManager.computeICP();
               currentRegionsGraphic.generateMeshes(mappingManager.getCurrentRegions().getPlanarRegionsList());
               currentRegionsGraphic.update();
               drawMatches();
            }

            if (ImGui.checkbox("Render Bounding Boxes [Previous]", renderBoundingBoxEnabled))
            {
               previousRegionsGraphic.setDrawBoundingBox(renderBoundingBoxEnabled.get());
               previousRegionsGraphic.generateMeshes(mappingManager.getPreviousRegions().getPlanarRegionsList());
               previousRegionsGraphic.update();
            }
            if (ImGui.checkbox("Render Bounding Boxes [Current]", renderBoundingBoxEnabled))
            {
               currentRegionsGraphic.setDrawBoundingBox(renderBoundingBoxEnabled.get());
               currentRegionsGraphic.generateMeshes(mappingManager.getCurrentRegions().getPlanarRegionsList());
               currentRegionsGraphic.update();
            }

            ImGui.checkbox("Render Match Lines", renderMatchLinesEnabled);
            ImGui.checkbox("Render Previous Regions", renderPreviousEnabled);
            ImGui.checkbox("Render Current Regions", renderCurrentEnabled);

            ImGui.endTabItem();
         }

         ImGui.endTabBar();
      }

      if (ImGui.button("Log Map Regions"))
      {
         mappingManager.logMapRegions();
      }

      if (ImGui.button("Log Aggregate Regions"))
      {
         mappingManager.logAggregateRegions();
      }

      ImGui.checkbox("Show Parameter Tuners", mappingParametersTuner.getIsShowing());
   }

   public void drawMatches()
   {
      matchLineMesh.clear();
      TIntIntMap matches = new TIntIntHashMap();
      PlaneRegistrationTools.findBestPlanarRegionMatches(new PlanarLandmarkList(mappingManager.getCurrentRegions().getPlanarRegionsList()),
                                                         new PlanarLandmarkList(mappingManager.getPreviousRegions().getPlanarRegionsList()),
                                                         matches,
                                                         (float) mappingManager.getParameters().getBestMinimumOverlapThreshold(),
                                                         (float) mappingManager.getParameters().getBestMatchAngularThreshold(),
                                                         (float) mappingManager.getParameters().getBestMatchDistanceThreshold(),
                                                         (float) mappingManager.getParameters().getMinimumBoundingBoxSize());

      ArrayList<Point3DReadOnly> matchEndPoints = new ArrayList<>();
      int[] keySet = matches.keySet().toArray();
      for (Integer match : keySet)
      {
         matchEndPoints.add(mappingManager.getPreviousRegions().getPlanarRegionsList().getPlanarRegion(match).getPoint());
         matchEndPoints.add(mappingManager.getCurrentRegions().getPlanarRegionsList().getPlanarRegion(matches.get(match)).getPoint());
      }
      matchLineMesh.generateMeshForMatchLines(matchEndPoints);
      matchLineMesh.update();
   }

   public void renderPoseGraphics()
   {
      rootJointGraphic.generateMeshes(mappingManager.getSensorPositionBuffer(), 5);
      rootJointGraphic.update();

      mocapGraphic.generateMeshes(mappingManager.getMocapPositionBuffer(), 10);
      mocapGraphic.update();
   }

   public RDXPanel getImGuiPanel()
   {
      return panel;
   }

   public boolean getPointCloudRenderEnabled()
   {
      return renderPointCloud.get();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderPreviousEnabled.get())
         previousRegionsGraphic.getRenderables(renderables, pool);

      if (renderCurrentEnabled.get())
         currentRegionsGraphic.getRenderables(renderables, pool);

      if (renderMatchLinesEnabled.get())
         matchLineMesh.getRenderables(renderables, pool);

      footstepPlanGraphic.getRenderables(renderables, pool);
   }
}
