package us.ihmc.jme;

import com.jme3.asset.AssetManager;
import com.jme3.math.ColorRGBA;
import com.jme3.scene.Node;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

public class PlanarRegionsListNode extends Node
{
   private final List<ColorRGBA> randomColors = new ArrayList<>();
   private final AssetManager assetManager;

   public PlanarRegionsListNode(AssetManager assetManager, PlanarRegionsList planarRegionsList)
   {
      this.assetManager = assetManager;

      updateRegions(planarRegionsList);
   }

   private List<PlanarRegionNode> createNewPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      List<PlanarRegionNode> planarRegionNodes = new ArrayList<>();

      randomColors.add(new ColorRGBA(0.7f, 0.7f, 0.7f, 0.3f));
      randomColors.add(new ColorRGBA(0.75f, 0.65f, 0.65f, 0.3f));
      randomColors.add(new ColorRGBA(0.65f, 0.75f, 0.65f, 0.3f));
      randomColors.add(new ColorRGBA(0.65f, 0.65f, 0.75f, 0.3f));

      for (int regionIndex = 0; regionIndex < planarRegionsList.getNumberOfPlanarRegions(); regionIndex++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionIndex);
         ColorRGBA color = randomColors.get(regionIndex % randomColors.size());
         PlanarRegionNode planarRegionNode = new PlanarRegionNode(regionIndex, planarRegion, color, assetManager);

         planarRegionNodes.add(planarRegionNode);
      }

      return planarRegionNodes;
   }

   public void updateRegions(PlanarRegionsList planarRegionsList)
   {
      getChildren().clear();
      List<PlanarRegionNode> regionNodes = createNewPlanarRegions(planarRegionsList);
      for (PlanarRegionNode regionNode : regionNodes)
      {
         attachChild(regionNode);
      }
   }
}
