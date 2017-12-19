package us.ihmc.robotEnvironmentAwareness.updaters;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree.RayMissProbabilityUpdater;
import us.ihmc.jOctoMap.occupancy.OccupancyParameters;

public class AdaptiveRayMissProbabilityUpdater implements RayMissProbabilityUpdater
{
   private double distanceSquaredToEndThreshold = MathTools.square(0.06);
   private double rayEndMissProbability = 0.47;

   private int normalConsensusThreshold = 10;
   private double dotRayToNormalThreshold = Math.cos(Math.toRadians(150.0));
   private double shallowAngleMissProbability = 0.45;

   @Override
   public double computeRayMissProbability(Point3DReadOnly rayOrigin, Point3DReadOnly rayEnd, Vector3DReadOnly rayDirection, NormalOcTreeNode node, OccupancyParameters parameters)
   {
      Point3D hitLocation = new Point3D();
      node.getHitLocation(hitLocation);

      if (hitLocation.distanceSquared(rayEnd) < distanceSquaredToEndThreshold)
      {
         return rayEndMissProbability;
      }
      else if (node.getNormalConsensusSize() > normalConsensusThreshold && node.isNormalSet())
      {
         Point3D nodeHitLocation = new Point3D();
         Vector3D nodeNormal = new Vector3D();
         node.getHitLocation(nodeHitLocation);
         node.getNormal(nodeNormal);

         if (Math.abs(rayDirection.dot(nodeNormal)) < dotRayToNormalThreshold)
            return shallowAngleMissProbability;
         else
            return parameters.getMissProbability();
      }
      else
      {
         return parameters.getMissProbability();
      }
   }
}
