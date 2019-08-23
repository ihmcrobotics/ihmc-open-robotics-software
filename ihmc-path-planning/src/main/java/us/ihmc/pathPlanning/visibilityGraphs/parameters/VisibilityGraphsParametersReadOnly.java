package us.ihmc.pathPlanning.visibilityGraphs.parameters;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.*;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.tools.property.StoredPropertySetReadOnly;

import java.util.List;

import static us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphParametersKeys.*;

public interface VisibilityGraphsParametersReadOnly extends StoredPropertySetReadOnly
{
   //TODO: JEP: JavaDoc all the methods to make them more clear.

   default double getMaxInterRegionConnectionLength()
   {
      return get(maxInterRegionConnectionLength);
   }

   default double getNormalZThresholdForAccessibleRegions()
   {
      return get(normalZThresholdForAccessibleRegions);
   }

   default double getNavigableExtrusionDistance()
   {
      return get(navigableExtrusionDistance);
   }

   default double getObstacleExtrusionDistance()
   {
      return get(obstacleExtrusionDistance);
   }

   default double getPreferredObstacleExtrusionDistance()
   {
      return get(preferredObstacleExtrusionDistance);
   }

   default double getObstacleExtrusionDistanceIfNotTooHighToStep()
   {
      return get(obstacleExtrusionDistanceIfNotTooHighToStep);
   }

   default double getTooHighToStepDistance()
   {
      return get(tooHighToStepDistance);
   }

   default double getClusterResolution()
   {
      return get(clusterResolution);
   }

   default double getExplorationDistanceFromStartGoal()
   {
      return get(explorationDistanceFromStartGoal);
   }

   default double getPlanarRegionMinArea()
   {
      return get(planarRegionMinArea);
   }

   default int getPlanarRegionMinSize()
   {
      return get(planarRegionMinSize);
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
      return get(regionOrthogonalAngle);
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
      return get(searchHostRegionEpsilon);
   }


   /**
    * Returns the height at which the robot can duck under an obstacle. Any obstacles that are higher than that height above a navigable planar reigon will be ignored.
    * @return height at which the robot can duck under an obstacle.
    */
   default double getCanDuckUnderHeight()
   {
      return get(canDuckUnderHeight);
   }

   /**
    * Returns the height at which an obstacle can be easily stepped over. Any obstacles that are lower than that height above a navigable planar region will be ignored.
    * @return height at which the robot can easily step over an object.
    */
   default double getCanEasilyStepOverHeight()
   {
      return get(canEasilyStepOverHeight);
   }

   default double getHeuristicWeight()
   {
      return get(heuristicWeight);
   }

   default double getDistanceWeight()
   {
      return get(distanceWeight);
   }

   default double getElevationWeight()
   {
      return get(elevationWeight);
   }

   /**
    * This flag says whether or not to return a solution even when the goal is not reached.
    * The solution that is returned is the lowest cost path, including estimated cost to goal.
    */
   default boolean returnBestEffortSolution()
   {
      return true;
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
         public double computeNavigableExtrusionDistance(PlanarRegion navigableRegionToBeExtruded)
         {
            return getNavigableExtrusionDistance();
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
      return (pointToExtrude, obstacleHeight) -> {
         if (obstacleHeight < 0.0)
         {
            return 0.0;
         }
         else if (obstacleHeight < getTooHighToStepDistance())
         {
            return getObstacleExtrusionDistanceIfNotTooHighToStep();
         }
         else
         {
            return getObstacleExtrusionDistance();
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
         private final double maxLength = getMaxInterRegionConnectionLength();
         private final double maxLengthSquared = MathTools.square(maxLength);
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

         @Override
         public double getMaximumInterRegionConnetionDistance()
         {
            return maxLength;
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
      final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

      return new ObstacleRegionFilter()
      {
         @Override
         public boolean isRegionValidObstacle(PlanarRegion potentialObstacleRegion, PlanarRegion navigableRegion)
         {
            //TODO: Choose one and use it. Or if allow for either, make this boolean a parameter or static field.
            boolean usePolygonIntersection = false;

            if (usePolygonIntersection)
            {
               return isRegionValidObstacleUsingPolygonIntersection(potentialObstacleRegion, navigableRegion);
            }
            else
            {
               return isRegionValidObstacleUsingMinimumHeightAbove(potentialObstacleRegion, navigableRegion);
            }
         }

         boolean isRegionValidObstacleUsingMinimumHeightAbove(PlanarRegion potentialObstacleRegion, PlanarRegion navigableRegion)
         {
            //TODO: ++++++JerryPratt: Fix this and use this if you want it to be more accurate. However, if this is just for approximate tests and can have false positives, then bounding boxes are fine.
            //      return doPolygonsIntersect(convexHullOne, convexHullTwo, epsilon);

            //TOOD: ++++++JerryPratt: Lots of bugs here. Need to clean up ConvexPolygon stuff to find distances and if overlapping more nicely...
            //TODO: Get rid of these magic numbers and make them parameters somewhere. Make sure the overlapping region check is larger than getMaxInterRegionConnectionLength()
            //TODO: BodyPathPlannerEnvironment crash when the number is set to 1.0. But should work fine all the same...
            //TOOD: This check should just be an approximation and should be ok for false positives. In fact, just returning true should be ok. Check that.
            //TODO: But somehow that's not right, since if we change 0.25 to 1.0 below, we get a Runtime Exception: Tried to create a line from two coincidal points!?
            if (!PlanarRegionTools.isRegionAOverlapingWithRegionB(potentialObstacleRegion, navigableRegion, 0.25)) //1.0)) //0.25)) //1.0))
               return false;

            double minimumHeight = PlanarRegionTools.computeMinHeightOfRegionAAboveRegionB(potentialObstacleRegion, navigableRegion);
            if (minimumHeight > getCanDuckUnderHeight())
               return false;

            if (!PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(potentialObstacleRegion, navigableRegion, getCanEasilyStepOverHeight()))
               return false;

            return true;
         }

         private boolean isRegionValidObstacleUsingPolygonIntersection(PlanarRegion potentialObstacleRegion, PlanarRegion navigableRegion)
         {
            ConvexPolygon2D potentialObstacleConvexHull = PlanarRegionTools.getVerticallyProjectedConvexHull(potentialObstacleRegion);
            ConvexPolygon2D navigableRegionConvexHull = PlanarRegionTools.getVerticallyProjectedConvexHull(navigableRegion);

            //TODO: Extrude the obstacle first...

            ConvexPolygon2D intersectionPolygon = new ConvexPolygon2D();

            //TODO: Write more test cases for ConvexPolygonTools.computeIntersectionOfPolygons
            boolean polygonsIntersect = convexPolygonTools.computeIntersectionOfPolygons(potentialObstacleConvexHull, navigableRegionConvexHull,
                                                                                         intersectionPolygon);
            if (!polygonsIntersect)
               return false;

            List<? extends Point2DReadOnly> intersectionVerticesInWorld2D = intersectionPolygon.getPolygonVerticesView();

            Double minimumHeight = Double.POSITIVE_INFINITY;
            Double maximumHeight = Double.NEGATIVE_INFINITY;

            for (int i = 0; i < intersectionVerticesInWorld2D.size(); i++)
            {
               Point3D pointToProject = new Point3D(intersectionVerticesInWorld2D.get(i));
               Point3D pointProjectedToPotentialObstacle = PlanarRegionTools.projectInZToPlanarRegion(pointToProject, potentialObstacleRegion);
               Point3D pointProjectedToNavigableRegion = PlanarRegionTools.projectInZToPlanarRegion(pointToProject, navigableRegion);

               double height = pointProjectedToPotentialObstacle.getZ() - pointProjectedToNavigableRegion.getZ();
               if (height < minimumHeight)
                  minimumHeight = height;
               if (height > maximumHeight)
                  maximumHeight = height;
            }

            if (minimumHeight < getCanDuckUnderHeight())
               return false;

            return true;
         }
      };
   }
}
