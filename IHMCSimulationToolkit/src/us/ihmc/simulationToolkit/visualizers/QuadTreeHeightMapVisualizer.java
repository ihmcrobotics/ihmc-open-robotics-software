package us.ihmc.simulationToolkit.visualizers;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGroundLeaf;
import us.ihmc.robotics.quadTree.QuadTreeForGroundNode;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundHeightMap;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeHeightMapInterface;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class QuadTreeHeightMapVisualizer
{
   
   public static Graphics3DNode drawHeightMap(QuadTreeHeightMapInterface heightMap, SimulationConstructionSet scs, BoundingBox2d rangeOfPointsToDraw, double resolution)
   {
      double minX = rangeOfPointsToDraw.getMinPoint().getX();
      double minY = rangeOfPointsToDraw.getMinPoint().getY();
      double maxX = rangeOfPointsToDraw.getMaxPoint().getX();
      double maxY = rangeOfPointsToDraw.getMaxPoint().getY();
      
      return drawHeightMap(heightMap, scs, minX, minY, maxX, maxY, resolution);
   }
   
   public static Graphics3DNode drawHeightMap(QuadTreeHeightMapInterface heightMap, SimulationConstructionSet scs, double minX, double minY, double maxX, double maxY, double resolution)
   {
      AppearanceDefinition[] rainbow = YoAppearance.getStandardRoyGBivRainbow();

      Graphics3DObject heightMapGraphic = new Graphics3DObject();

      for (double x = minX; x<maxX; x = x + resolution)
      {
         for (double y = minY; y<maxY; y = y + resolution)
         {
            double z = heightMap.getHeightAtPoint(x, y);

            if (!Double.isNaN(z))
            {
               int index = (int) (z / resolution);
               index = index % rainbow.length;
               if (index < 0) index = index + rainbow.length;
               
               AppearanceDefinition appearance = rainbow[index];

               heightMapGraphic.identity();
               heightMapGraphic.translate(x, y, z - resolution/4.0);
               heightMapGraphic.addCube(resolution, resolution, resolution/4.0, appearance);
            }
         }
      }
      return scs.addStaticLinkGraphics(heightMapGraphic);
   }


   public static Graphics3DNode drawAllPointsInQuadTree(QuadTreeHeightMapInterface heightMap, double resolution, SimulationConstructionSet scs, AppearanceDefinition appearance)
   {
      if (heightMap instanceof QuadTreeForGroundHeightMap)
      {
         ArrayList<Point3D> points = new ArrayList<Point3D>();
         ((QuadTreeForGroundHeightMap) heightMap).getStoredPoints(points);

         return drawPoints(scs, points, resolution, appearance);
      }
      return null;
   }

   public static Graphics3DNode drawPoints(SimulationConstructionSet scs, ArrayList<Point3D> points, double resolution, AppearanceDefinition appearance)
   {
      Graphics3DObject pointsInQuadTreeGraphic = new Graphics3DObject();

      for (Point3D point : points)
      {
         pointsInQuadTreeGraphic.identity();
         pointsInQuadTreeGraphic.translate(point);
         pointsInQuadTreeGraphic.addCube(resolution, resolution, resolution/4.0, appearance);
      }

      Graphics3DNode graphics3DNodeHandle = scs.addStaticLinkGraphics(pointsInQuadTreeGraphic);
      return graphics3DNodeHandle;
   }


   public static Graphics3DNode drawNodeBoundingBoxes(QuadTreeForGroundHeightMap heightMap, SimulationConstructionSet scs, double heightToDrawAt)
   {
      QuadTreeForGroundNode rootNode = heightMap.getRootNode();
      Graphics3DObject nodeBoundsGraphic = new Graphics3DObject();
      drawNodeBoundingBoxesRecursively(rootNode, nodeBoundsGraphic, 0, heightToDrawAt);

      Graphics3DNode graphics3DNodeHandle = scs.addStaticLinkGraphics(nodeBoundsGraphic);
      return graphics3DNodeHandle;
   }

   private static void drawNodeBoundingBoxesRecursively(QuadTreeForGroundNode node, Graphics3DObject nodeBoundsGraphic, int depth, double nodeZ)
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
         QuadTreeForGroundLeaf leaf = node.getLeaf();
         if (leaf != null)
         {
            Point3D averagePoint = leaf.getAveragePoint();
            nodeBoundsGraphic.translate(bounds.centreX, bounds.centreY, averagePoint.getZ());
            nodeBoundsGraphic.addCube(0.9 * (bounds.maxX - bounds.minX), 0.9 * (bounds.maxY - bounds.minY), 0.002, YoAppearance.Black());
         }
      }

      ArrayList<QuadTreeForGroundNode> children = new ArrayList<QuadTreeForGroundNode>();
      node.getChildrenNodes(children);

      for (QuadTreeForGroundNode child : children)
      {
         drawNodeBoundingBoxesRecursively(child, nodeBoundsGraphic, depth + 1, nodeZ + 0.01);
      }
   }
}
