package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.utilities.dataStructures.quadTree.Box;
import us.ihmc.utilities.dataStructures.quadTree.SimplifiedQuadLeaf;
import us.ihmc.utilities.dataStructures.quadTree.SimplifiedQuadNode;

public class SimplifiedGroundOnlyQuadTreeVisualizer
{
// private static final AppearanceDefinition[] rainbow = YoAppearance.getStandardRoyGBivRainbow();

   public static Graphics3DNode drawNodeBoundingBoxes(SimplifiedGroundOnlyQuadTree heightMap, SimulationConstructionSet scs, double heightToDrawAt)
   {
      SimplifiedQuadNode rootNode = heightMap.getRootNode();
      Graphics3DObject nodeBoundsGraphic = new Graphics3DObject();
      drawNodeBoundingBoxesRecursively(rootNode, nodeBoundsGraphic, 0, heightToDrawAt);

      Graphics3DNode graphics3DNodeHandle = scs.addStaticLinkGraphics(nodeBoundsGraphic);
      return graphics3DNodeHandle;
   }

   private static void drawNodeBoundingBoxesRecursively(SimplifiedQuadNode node, Graphics3DObject nodeBoundsGraphic, int depth, double nodeZ)
   {
      AppearanceDefinition[] rainbow = YoAppearance.getStandardRoyGBivRainbow();

      Box bounds = node.getBounds();

      nodeBoundsGraphic.identity();

      if (node.hasChildren())
      {
         nodeBoundsGraphic.translate(bounds.centreX, bounds.centreY, nodeZ);
         nodeBoundsGraphic.addCube(0.9 * (bounds.maxX - bounds.minX), 0.9 * (bounds.maxY - bounds.minY), 0.002, rainbow[depth % rainbow.length]);
      }
      else
      {
         SimplifiedQuadLeaf leaf = node.getLeaf();
         if (leaf != null)
         {
            if (leaf.containsPoints())
            {
               Point3d averagePoint = leaf.getAveragePoint();
               nodeBoundsGraphic.translate(bounds.centreX, bounds.centreY, averagePoint.getZ());
               nodeBoundsGraphic.addCube(0.9 * (bounds.maxX - bounds.minX), 0.9 * (bounds.maxY - bounds.minY), 0.002, YoAppearance.Black());

               // nodeBoundsGraphic.addCube(0.9 * (bounds.maxX - bounds.minX), 0.9 * (bounds.maxY - bounds.minY), 0.002, rainbow[depth % rainbow.length]);
            }
            else
            {
               throw new RuntimeException("All leafs should have points in them!!!");

               // nodeBoundsGraphic.addCube(0.9 * (bounds.maxX - bounds.minX), 0.9 * (bounds.maxY - bounds.minY), 0.002, YoAppearance.Black());
            }
         }
         else
         {
            // throw new RuntimeException("Any node without children should have leafs!");
         }
      }


      ArrayList<SimplifiedQuadNode> children = new ArrayList<SimplifiedQuadNode>();
      node.getChildrenNodes(children);

      for (SimplifiedQuadNode child : children)
      {
         drawNodeBoundingBoxesRecursively(child, nodeBoundsGraphic, depth + 1, nodeZ + 0.01);
      }

   }

}
