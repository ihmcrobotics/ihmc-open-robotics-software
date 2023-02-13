package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.PlanarRegionMappingHandler;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.visualizers.RDXLineMeshModel;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMTools;

import java.util.ArrayList;
import java.util.HashMap;

public class RDXPlanarRegionMappingUIPanel implements RenderableProvider
{
   private ImGuiStoredPropertySetTuner mappingParametersTuner;
   private PlanarRegionMappingHandler mappingManager;
   private ImGuiPanel imGuiPanel;
   private final ImBoolean liveModeEnabled = new ImBoolean();
   private final ImBoolean renderEnabled = new ImBoolean(true);
   private final ImBoolean renderBoundingBoxEnabled = new ImBoolean(false);
   private boolean captured = false;

   private RDXLineMeshModel lineMeshModel = new RDXLineMeshModel(0.02f, Color.WHITE);
   private RDXPlanarRegionsGraphic previousRegionsGraphic = new RDXPlanarRegionsGraphic();
   private RDXPlanarRegionsGraphic currentRegionsGraphic = new RDXPlanarRegionsGraphic();

   private ImInt icpPreviousIndex = new ImInt(10);
   private ImInt icpCurrentIndex = new ImInt(14);
  
   public RDXPlanarRegionMappingUIPanel(String name, PlanarRegionMappingHandler mappingManager)
   {
      this.mappingManager = mappingManager;
      imGuiPanel = new ImGuiPanel(name, this::renderImGuiWidgets);
      mappingParametersTuner = new ImGuiStoredPropertySetTuner(mappingManager.getParameters().getTitle());
      mappingParametersTuner.create(mappingManager.getParameters());
      imGuiPanel.addChild(mappingParametersTuner);
   }

   public void renderImGuiWidgets()
   {
      if(ImGui.beginTabBar("Mapping"))
      {
         if(ImGui.beginTabItem("Map"))
         {
            if (ImGui.button("Load Next Set"))
               mappingManager.nextButtonCallback();
            if (ImGui.button("Auto Increment"))
               mappingManager.autoIncrementButtonCallback();
            if (ImGui.checkbox("Enable Live Mode", liveModeEnabled))
               mappingManager.setEnableLiveMode(liveModeEnabled.get());
            ImGui.checkbox("Render live mode", renderEnabled);
            if (ImGui.button("Reset map"))
               mappingManager.resetMap();
            if (ImGui.button("Hard reset map"))
               mappingManager.hardResetTheMap();
            ImGui.endTabItem();
         }

         if(ImGui.beginTabItem("Registration"))
         {
            if(ImGui.button("Load Previous"))
            {
               mappingManager.loadRegionsFromLogIntoPrevious(icpPreviousIndex.get());
               previousRegionsGraphic.generateMeshes(mappingManager.getPreviousRegions().getPlanarRegionsList());
               previousRegionsGraphic.update();
            }
            ImGui.sameLine();
            ImGui.sliderInt("Previous", icpPreviousIndex.getData(), 0, mappingManager.getSensorPositionBuffer().size() - 1);

            icpPreviousIndex.set(Math.min(icpPreviousIndex.get(), icpCurrentIndex.get() - 1));
            icpCurrentIndex.set(Math.max(1, icpCurrentIndex.get()));

            if(ImGui.button("Load Curernt"))
            {
               mappingManager.loadRegionsFromLogIntoCurrent(icpCurrentIndex.get());
               currentRegionsGraphic.generateMeshes(mappingManager.getCurrentRegions().getPlanarRegionsList());
               currentRegionsGraphic.update();
               drawMatches();
            }
            ImGui.sameLine();
            ImGui.sliderInt("Current", icpCurrentIndex.getData(), 0, mappingManager.getSensorPositionBuffer().size() - 1);
            if(ImGui.button("Optimize Transform"))
            {
               mappingManager.computeICP();
               currentRegionsGraphic.generateMeshes(mappingManager.getCurrentRegions().getPlanarRegionsList());
               currentRegionsGraphic.update();
               drawMatches();
            }

            if(ImGui.checkbox("Render Bounding Boxes [Previous]", renderBoundingBoxEnabled))
            {
               previousRegionsGraphic.setDrawBoundingBox(renderBoundingBoxEnabled.get());
               previousRegionsGraphic.generateMeshes(mappingManager.getPreviousRegions().getPlanarRegionsList());
               previousRegionsGraphic.update();

            }
            if(ImGui.checkbox("Render Bounding Boxes [Current]", renderBoundingBoxEnabled))
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
      HashMap<Integer, Integer> matches = new HashMap<>();
      PlanarRegionSLAMTools.findBestPlanarRegionMatches(mappingManager.getCurrentRegions().getPlanarRegionsList(),
                                                        mappingManager.getPreviousRegions().getPlanarRegionsList(), matches,
                                                        0.5f, 0.8f, 0.4f, 0.3f);

      ArrayList<Point3DReadOnly> matchEndPoints = new ArrayList<>();
      for(Integer match : matches.keySet())
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

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      previousRegionsGraphic.getRenderables(renderables, pool);
      currentRegionsGraphic.getRenderables(renderables, pool);
      lineMeshModel.getRenderables(renderables, pool);
   }
}
