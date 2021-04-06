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

      List<PlanarRegionNode> regionNodes = createNewPlanarRegions(planarRegionsList);
      for (PlanarRegionNode regionNode : regionNodes)
      {
         attachChild(regionNode);
      }
   }

   private List<PlanarRegionNode> createNewPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      List<PlanarRegionNode> planarRegionNodes = new ArrayList<>();

      while (randomColors.size() < planarRegionsList.getNumberOfPlanarRegions())
         randomColors.add(ColorRGBA.randomColor());

      for (int regionIndex = 0; regionIndex < planarRegionsList.getNumberOfPlanarRegions(); regionIndex++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionIndex);
         ColorRGBA color = randomColors.get(regionIndex);
         PlanarRegionNode planarRegionNode = new PlanarRegionNode(regionIndex, planarRegion, color, assetManager);

         planarRegionNodes.add(planarRegionNode);
      }

      return planarRegionNodes;
   }
}
