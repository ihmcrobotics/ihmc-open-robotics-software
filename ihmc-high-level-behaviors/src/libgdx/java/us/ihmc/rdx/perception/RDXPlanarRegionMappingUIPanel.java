package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import gnu.trove.map.TIntIntMap;
import gnu.trove.map.hash.TIntIntHashMap;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.PlanarRegionMappingHandler;
import us.ihmc.perception.tools.PlaneRegistrationTools;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.visualizers.RDXLineMeshModel;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarLandmarkList;

import java.util.ArrayList;

public class RDXPlanarRegionMappingUIPanel implements RenderableProvider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImGuiStoredPropertySetTuner mappingParametersTuner;
   private PlanarRegionMappingHandler mappingManager;
   private ImGuiPanel imGuiPanel;

   private ImGuiPlot wholeAlgorithmDurationPlot;
   private ImGuiPlot quaternionAveragingDurationPlot;
   private ImGuiPlot factorGraphDurationPlot;
   private ImGuiPlot regionMergingDurationPlot;

   private final ImBoolean liveModeEnabled = new ImBoolean();
   private final ImBoolean renderEnabled = new ImBoolean(true);
   private final ImBoolean renderBoundingBoxEnabled = new ImBoolean(false);
   private boolean captured = false;

   private RDXLineMeshModel lineMeshModel = new RDXLineMeshModel(0.02f, Color.WHITE);
   private RDXPlanarRegionsGraphic previousRegionsGraphic = new RDXPlanarRegionsGraphic();
   private RDXPlanarRegionsGraphic currentRegionsGraphic = new RDXPlanarRegionsGraphic();

   private ImBoolean renderPointCloud = new ImBoolean(false);
   private ImInt icpPreviousIndex = new ImInt(10);
   private ImInt icpCurrentIndex = new ImInt(14);

   public RDXPlanarRegionMappingUIPanel(String name, PlanarRegionMappingHandler mappingManager)
   {
      this.mappingManager = mappingManager;
      imGuiPanel = new ImGuiPanel(name, this::renderImGuiWidgets);
      mappingParametersTuner = new ImGuiStoredPropertySetTuner(mappingManager.getParameters().getTitle());
      mappingParametersTuner.create(mappingManager.getParameters());
      imGuiPanel.addChild(mappingParametersTuner);

      wholeAlgorithmDurationPlot = new ImGuiPlot(labels.get("Whole algorithm duration"), 1000, 300, 50);
      quaternionAveragingDurationPlot = new ImGuiPlot(labels.get("ICP duration"), 1000, 300, 50);
      factorGraphDurationPlot = new ImGuiPlot(labels.get("ICP registration duration"), 1000, 300, 50);
      regionMergingDurationPlot = new ImGuiPlot(labels.get("Region merging duration"), 1000, 300, 50);
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.beginTabBar("Mapping"))
      {
         if (ImGui.beginTabItem("Map"))
         {
            wholeAlgorithmDurationPlot.render(mappingManager.getPlanarRegionMap().getWholeAlgorithmDurationStopwatch().totalElapsed());
            quaternionAveragingDurationPlot.render(mappingManager.getPlanarRegionMap().getQuaternionAveragingStopwatch().totalElapsed());
            factorGraphDurationPlot.render(mappingManager.getPlanarRegionMap().getFactorGraphStopwatch().totalElapsed());
            regionMergingDurationPlot.render(mappingManager.getPlanarRegionMap().getRegionMergingStopwatch().totalElapsed());

            if (ImGui.button("Load Next Set"))
               mappingManager.nextButtonCallback();
            ImGui.sameLine();
            if (ImGui.button("Perform Map Clean-up"))
               mappingManager.performMapCleanUp();
            if (ImGui.button("Auto Increment"))
               mappingManager.autoIncrementButtonCallback();
            if (ImGui.checkbox("Enable Live Mode", liveModeEnabled))
               mappingManager.setEnableLiveMode(liveModeEnabled.get());
            ImGui.checkbox("Render live mode", renderEnabled);
            if (ImGui.button("Reset map"))
               mappingManager.resetMap();
            if (ImGui.button("Hard reset map"))
               mappingManager.hardResetTheMap();

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

            ImGui.endTabItem();
         }

         ImGui.endTabBar();
      }

      if (ImGui.button("Capture"))
      {
         mappingManager.setCaptured(true);
      }

      ImGui.checkbox("Show Parameter Tuners", mappingParametersTuner.getIsShowing());
   }

   public void drawMatches()
   {
      lineMeshModel.clear();
      TIntIntMap matches = new TIntIntHashMap();
      PlaneRegistrationTools.findBestPlanarRegionMatches(new PlanarLandmarkList(mappingManager.getCurrentRegions().getPlanarRegionsList()),
                                                         new PlanarLandmarkList(mappingManager.getPreviousRegions().getPlanarRegionsList()),
                                                         matches,
                                                         (float) mappingManager.getParameters().getBestMinimumOverlapThreshold(),
                                                         (float) mappingManager.getParameters().getBestMatchAngularThreshold(),
                                                         (float) mappingManager.getParameters().getBestMatchDistanceThreshold(),
                                                         0.3f);

      ArrayList<Point3DReadOnly> matchEndPoints = new ArrayList<>();
      int[] keySet = matches.keySet().toArray();
      for (Integer match : keySet)
      {
         matchEndPoints.add(mappingManager.getPreviousRegions().getPlanarRegionsList().getPlanarRegion(match).getPoint());
         matchEndPoints.add(mappingManager.getCurrentRegions().getPlanarRegionsList().getPlanarRegion(matches.get(match)).getPoint());
      }
      lineMeshModel.generateMeshForMatchLines(matchEndPoints);
      lineMeshModel.update();
   }

   public void setCaptured(boolean captured)
   {
      this.captured = captured;
   }

   public boolean isCaptured()
   {
      return captured;
   }

   public ImGuiPanel getImGuiPanel()
   {
      return imGuiPanel;
   }

   public boolean getPointCloudRenderEnabled()
   {
      return renderPointCloud.get();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      previousRegionsGraphic.getRenderables(renderables, pool);
      currentRegionsGraphic.getRenderables(renderables, pool);
      lineMeshModel.getRenderables(renderables, pool);
   }
}
