package us.ihmc.rdx.perception;

import us.ihmc.perception.HumanoidPerceptionModule;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class RDXHumanoidPerceptionUI
{
   private HumanoidPerceptionModule humanoidPerception;
   private RDXPlanarRegionsGraphic mapRegionsGraphic;
   private RDXRapidRegionsUI rapidRegionsUI;
   private RDXPlanarRegionMappingUI terrainMappingUI;

   public RDXHumanoidPerceptionUI(HumanoidPerceptionModule humanoidPerception)
   {
      this.humanoidPerception = humanoidPerception;
      this.terrainMappingUI = new RDXPlanarRegionMappingUI("Filtered Map", humanoidPerception.getMapHandler());
      this.mapRegionsGraphic = new RDXPlanarRegionsGraphic();
      this.rapidRegionsUI = new RDXRapidRegionsUI();
      this.rapidRegionsUI.create(humanoidPerception.getRapidRegionsExtractor());
   }

   public void renderImGuiWidgets()
   {
      rapidRegionsUI.renderImGuiWidgets();
      terrainMappingUI.renderImGuiWidgets();
   }

   public void render()
   {
      rapidRegionsUI.render();

      FramePlanarRegionsList frameRegions = humanoidPerception.getFramePlanarRegionsResult();
      PlanarRegionsList planarRegionsList = frameRegions.getPlanarRegionsList();

      if (planarRegionsList != null && humanoidPerception.getRapidRegionsExtractor().isModified())
      {
         rapidRegionsUI.render3DGraphics(frameRegions);
         humanoidPerception.getRapidRegionsExtractor().setProcessing(false);
      }

      if (humanoidPerception.getMapHandler().pollIsModified() && humanoidPerception.getMapHandler().hasPlanarRegionsToRender())
      {
         mapRegionsGraphic.clear();
         mapRegionsGraphic.generateMeshes(humanoidPerception.getMapHandler().pollMapRegions());
         mapRegionsGraphic.update();
      }
   }

   public RDXPlanarRegionMappingUI getTerrainMappingUI()
   {
      return terrainMappingUI;
   }

   public RDXRapidRegionsUI getRapidRegionsUI()
   {
      return rapidRegionsUI;
   }

   public RDXPlanarRegionsGraphic getMapRegionsGraphic()
   {
      return mapRegionsGraphic;
   }
}
