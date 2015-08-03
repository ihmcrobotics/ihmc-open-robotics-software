package us.ihmc.ihmcPerception.depthData;

import us.ihmc.communication.packets.sensing.DepthDataClearCommand.DepthDataTree;
import us.ihmc.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundHeightMap;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeHeightMapInterface;
import us.ihmc.userInterface.util.DecayingResolutionFilter;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGroundParameters;

public class DepthDataStore
{

   public static final double QUAD_TREE_EXTENT = 200;
   protected final QuadTreeHeightMapInterface quadTree;
   protected final DecayingResolutionFilter nearScan;
   protected DepthDataFilterParameters parameters;

   public DepthDataStore()
   {
      this(DepthDataFilterParameters.getDefaultParameters());
   }

   public DepthDataStore(DepthDataFilterParameters parameters)
   {
      this.parameters = parameters;
      nearScan = new DecayingResolutionFilter(parameters.nearScanResolution, parameters.nearScanDecayMillis, parameters.nearScanCapacity);
      quadTree = setupGroundOnlyQuadTree(parameters);

   }

   public static QuadTreeForGroundHeightMap setupGroundOnlyQuadTree(DepthDataFilterParameters parameters)
   {
      Box bounds = new Box(-QUAD_TREE_EXTENT, -QUAD_TREE_EXTENT, QUAD_TREE_EXTENT, QUAD_TREE_EXTENT);
      QuadTreeForGroundParameters quadTreeParameters = new QuadTreeForGroundParameters(DepthDataFilterParameters.GRID_RESOLUTION,
            parameters.quadtreeHeightThreshold, parameters.quadTreeMaxMultiLevelZChangeToFilterNoise, parameters.maxSameHeightPointsPerNode,
            parameters.maxAllowableXYDistanceForAPointToBeConsideredClose, parameters.maximumNumberOfPoints);

      return new QuadTreeForGroundHeightMap(bounds, quadTreeParameters);
   }

   public void clearLidarData(DepthDataTree lidarTree)
   {
      switch (lidarTree)
      {
      case DECAY_POINT_CLOUD:
         nearScan.clear();
         break;

      case QUADTREE:
         quadTree.clearTree(Double.NaN);

         break;

      default:
         throw new RuntimeException("Unknown tree");
      }
   }

   public QuadTreeHeightMapInterface getQuadTree()
   {
      return quadTree;
   }

   public DecayingResolutionFilter getNearScan()
   {
      return nearScan;
   }

   public void setParameters(DepthDataFilterParameters parameters)
   {
      this.parameters = parameters;

      nearScan.setResolution(parameters.nearScanResolution);
      nearScan.setCapacity(parameters.nearScanCapacity);
      nearScan.setDecay(parameters.nearScanDecayMillis);

      quadTree.setHeightThreshold(parameters.quadtreeHeightThreshold);

   }

   public DepthDataFilterParameters getParameters()
   {
      return parameters;
   }

}