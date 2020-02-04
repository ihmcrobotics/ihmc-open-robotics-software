package us.ihmc.pathPlanning.visibilityGraphs.graphSearch;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;

public class EdgeCostCalculator
{
   private final VisibilityGraphsParametersReadOnly parameters;

   public EdgeCostCalculator(VisibilityGraphsParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public double computeEdgeCost(VisibilityGraphEdge edge)
   {
      double edgeWeight = edge.getEdgeWeight();
      Point3DReadOnly originPointInWorld = edge.getSourceNode().getPointInWorld();
      Point3DReadOnly nextPointInWorld = edge.getTargetNode().getPointInWorld();

      double horizontalDistance = originPointInWorld.distanceXY(nextPointInWorld);
      if (horizontalDistance <= 0.0)
         return 0.0;

      double verticalDistance = Math.abs(nextPointInWorld.getZ() - originPointInWorld.getZ());


      double distanceCost = parameters.getDistanceWeight() * horizontalDistance;
      // 2/pi to scale error from {0:90} degrees or {0:pi/2} radians degrees to {0:1}
      double elevationCost;
      if (parameters.getElevationWeight() > 0.0)
      {
         double angle = Math.atan(verticalDistance / horizontalDistance);
         elevationCost = parameters.getElevationWeight() * angle * (2.0 / Math.PI);
      }
      else
      {
         elevationCost = 0.0;
      }

      return edgeWeight * distanceCost + elevationCost;
   }
}
