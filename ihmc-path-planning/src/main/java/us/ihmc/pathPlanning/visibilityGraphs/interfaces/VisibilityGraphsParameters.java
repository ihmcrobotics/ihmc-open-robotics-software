package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public interface VisibilityGraphsParameters
{
   public double getMaxInterRegionConnectionLength();

   public double getNormalZThresholdForAccessibleRegions();

   public double getExtrusionDistance();

   public double getExtrusionDistanceIfNotTooHighToStep();

   public double getTooHighToStepDistance();

   public double getClusterResolution();

   default double getExplorationDistanceFromStartGoal()
   {
      return Double.POSITIVE_INFINITY;
   }

   default double getPlanarRegionMinArea()
   {
      return 0.0;
   }

   default int getPlanarRegionMinSize()
   {
      return 0;
   }

   /**
    * Defines the angle from which two regions are considered orthogonal.
    * <p>
    * It is used to determine if a region should be projected onto another as a polygon or a line.
    * </p>
    * <p>
    * It should be close to 90 degrees.
    * </p>
    * 
    * @return the angle threshold to use to determine if a line or polygon projection method should
    *         be used.
    */
   default double getRegionOrthogonalAngle()
   {
      return Math.toRadians(75.0);
   }

   /**
    * This epsilon is is used when searching to which region the start/goal belongs to.
    * <p>
    * A positive value corresponds to growing all the regions before testing if the start/goal is
    * inside.
    * </p>
    * 
    * @return the value of the epsilon to use.
    */
   default double getSearchHostRegionEpsilon()
   {
      return 0.03;
   }

   /**
    * The constant extrusion distance to use when extruding the hull of a navigable region.
    * 
    * @return
    */
   default NavigableExtrusionDistanceCalculator getNavigableExtrusionDistanceCalculator()
   {
      return new NavigableExtrusionDistanceCalculator()
      {
         @Override
         public double computeExtrusionDistance(PlanarRegion navigableRegionToBeExtruded)
         {
            return getExtrusionDistanceIfNotTooHighToStep();
         }
      };
   }

   /**
    * This calculator is used when extruding the projection of an obstacle onto a navigable region.
    * 
    * @return the calculator use for obstacle extrusion.
    */
   default ObstacleExtrusionDistanceCalculator getObstacleExtrusionDistanceCalculator()
   {
      return (pointToExtrude, obstacleHeight) ->
      {
         if(obstacleHeight < 0.0)
         {
            return 0.0;
         }
         else if(obstacleHeight < getTooHighToStepDistance())
         {
            return getExtrusionDistanceIfNotTooHighToStep();
         }
         else
         {
            return getExtrusionDistance();
         }
      };
   }

   default NavigableRegionFilter getNavigableRegionFilter()
   {
      return new NavigableRegionFilter()
      {
         @Override
         public boolean isPlanarRegionNavigable(PlanarRegion query, List<PlanarRegion> allOtherRegions)
         {
            return query.getNormal().getZ() >= getNormalZThresholdForAccessibleRegions();
         }
      };
   }

   default InterRegionConnectionFilter getInterRegionConnectionFilter()
   {
      return new InterRegionConnectionFilter()
      {
         private final double maxLengthSquared = MathTools.square(getMaxInterRegionConnectionLength());
         private final double maxDeltaHeight = getTooHighToStepDistance();

         @Override
         public boolean isConnectionValid(ConnectionPoint3D source, ConnectionPoint3D target)
         {
            if (Math.abs(source.getZ() - target.getZ()) > maxDeltaHeight)
               return false;
            if (source.distanceSquared(target) > maxLengthSquared)
               return false;

            return true;
         }
      };
   }

   default PlanarRegionFilter getPlanarRegionFilter()
   {
      return new PlanarRegionFilter()
      {
         @Override
         public boolean isPlanarRegionRelevant(PlanarRegion region)
         {
            if (region.getConcaveHullSize() < getPlanarRegionMinSize())
               return false;
            if (!Double.isFinite(getPlanarRegionMinArea()) || getPlanarRegionMinArea() <= 0.0)
               return true;
            return PlanarRegionTools.computePlanarRegionArea(region) >= getPlanarRegionMinArea();
         }
      };
   }

   default ObstacleRegionFilter getObstacleRegionFilter()
   {
      return new ObstacleRegionFilter()
      {
         @Override
         public boolean isRegionValidObstacle(PlanarRegion query, PlanarRegion navigableRegion)
         {
            if (!PlanarRegionTools.isRegionAOverlapingWithRegionB(query, navigableRegion, 0.1))
               return false;

            if (PlanarRegionTools.computeMinHeightOfRegionAAboveRegionB(query, navigableRegion) > 3.0)
               return false;

            return true;
         }
      };
   }
}
