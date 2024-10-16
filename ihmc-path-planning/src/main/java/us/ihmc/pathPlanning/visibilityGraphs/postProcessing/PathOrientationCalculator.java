package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapSolution;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.commons.AngleTools;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class PathOrientationCalculator
{
   private final VisibilityGraphsParametersReadOnly parameters;

   public PathOrientationCalculator(VisibilityGraphsParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public List<? extends Pose3DReadOnly> computePosesFromPath(List<Point3DReadOnly> path, VisibilityMapSolution visibilityMapSolution,
                                                              Orientation3DReadOnly startOrientation, Orientation3DReadOnly goalOrientation)
   {
      List<Pose3DBasics> pathPoses = computeNominalPosesForPath(path, startOrientation, goalOrientation);

      modifyPathOrientationsToAvoidObstacles(pathPoses, visibilityMapSolution);

      return pathPoses;
   }

   private List<Pose3DBasics> computeNominalPosesForPath(List<Point3DReadOnly> path, Orientation3DReadOnly startOrientation, Orientation3DReadOnly goalOrientation)
   {
      List<Pose3DBasics> nominalPathPoses = new ArrayList<>();
      if (path.size() < 2)
         return nominalPathPoses;

      List<Point3DReadOnly> pathCopy = path.stream().map(Point3D::new).collect(Collectors.toList());

      double startHeading = startOrientation.getYaw();
      nominalPathPoses.add(new Pose3D(pathCopy.get(0), startOrientation));

      int pathIndex = 1;

      while (pathIndex < pathCopy.size() - 1)
      {
         Point3DReadOnly previousPosition = pathCopy.get(pathIndex - 1);
         Point3DReadOnly currentPosition = pathCopy.get(pathIndex);
         Point3DReadOnly nextPosition = pathCopy.get(pathIndex + 1);

         double previousHeading = BodyPathPlannerTools.calculateHeading(previousPosition, currentPosition);
         double nextHeading = BodyPathPlannerTools.calculateHeading(currentPosition, nextPosition);
         double previousOrientation = previousHeading;

         // override these orientations if it's the start or goal.
         if (pathIndex == 1)
            previousOrientation = startHeading;

         double desiredOrientation = AngleTools.interpolateAngle(previousOrientation, nextHeading, 0.5);

         if (Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(previousOrientation, nextHeading)) > 1e-3)
         {
            double previousLength = currentPosition.distanceXY(previousPosition);
            double nextLength = currentPosition.distanceXY(nextPosition);

            // add a point before
            if (previousLength > 4.0 * parameters.getObstacleExtrusionDistance())
            {
               double alpha = 1.0 - 2.0 * parameters.getObstacleExtrusionDistance() / previousLength;
               Point3DBasics waypointPositionToAdd = new Point3D();
               waypointPositionToAdd.interpolate(previousPosition, currentPosition, alpha);
               pathCopy.add(pathIndex, waypointPositionToAdd);
               nominalPathPoses.add(pathIndex, new Pose3D(waypointPositionToAdd, new Quaternion(previousHeading, 0.0, 0.0)));

               // we had enough room to reach the previous heading, and we just started, let's re-do the desired orientation
               if (pathIndex == 1)
                  desiredOrientation = AngleTools.interpolateAngle(previousHeading, nextHeading, 0.5);

               pathIndex++;
            }
            else if (previousLength > parameters.getObstacleExtrusionDistance())
            {
               Point3DBasics waypointPositionToAdd = new Point3D();
               waypointPositionToAdd.interpolate(previousPosition, currentPosition, 0.5);
               pathCopy.add(pathIndex, waypointPositionToAdd);
               nominalPathPoses.add(pathIndex, new Pose3D(waypointPositionToAdd, new Quaternion(previousHeading, 0.0, 0.0)));

               // we had enough room to reach the previous heading, and we just started, let's re-do the desired orientation
               if (pathIndex == 1)
                  desiredOrientation = AngleTools.interpolateAngle(previousHeading, nextHeading, 0.5);

               pathIndex++;
            }

            nominalPathPoses.add(pathIndex, new Pose3D(currentPosition, new Quaternion(desiredOrientation, 0.0, 0.0)));
            pathIndex++;

            // add a point after
            if (nextLength > 4.0 * parameters.getObstacleExtrusionDistance())
            {
               double alpha = 2.0 * parameters.getObstacleExtrusionDistance() / nextLength;
               Point3DBasics waypointPositionToAdd = new Point3D();
               waypointPositionToAdd.interpolate(currentPosition, nextPosition, alpha);
               pathCopy.add(pathIndex, waypointPositionToAdd);
            }
            else if (nextLength > parameters.getObstacleExtrusionDistance())
            {
               Point3DBasics waypointPositionToAdd = new Point3D();
               waypointPositionToAdd.interpolate(currentPosition, nextPosition, 0.5);
               pathCopy.add(pathIndex, waypointPositionToAdd);
            }
         }
         else
         {
            nominalPathPoses.add(pathIndex, new Pose3D(pathCopy.get(pathIndex), new Quaternion(desiredOrientation, 0.0, 0.0)));
            pathIndex++;
         }
      }

      int endingSize = pathCopy.size();

      nominalPathPoses.add(new Pose3D(pathCopy.get(endingSize - 1), goalOrientation));

      return nominalPathPoses;
   }

   private void modifyPathOrientationsToAvoidObstacles(List<Pose3DBasics> pathPosesToPack, VisibilityMapSolution visibilityMapSolution)
   {
      List<Cluster> allObstacleClusters = new ArrayList<>();
      if (parameters.getComputeOrientationsToAvoidObstacles())
      {
         visibilityMapSolution.getNavigableRegions().getNavigableRegionsList().forEach(region -> allObstacleClusters.addAll(region.getObstacleClusters()));
      }

      for (int pathIndex = 1; pathIndex < pathPosesToPack.size() - 1; pathIndex++)
      {
         Point3DReadOnly currentPosition = pathPosesToPack.get(pathIndex).getPosition();

         Point2D currentPosition2D = new Point2D(currentPosition);

         double desiredOrientation = pathPosesToPack.get(pathIndex).getOrientation().getYaw();

         if (parameters.getComputeOrientationsToAvoidObstacles())
         {
            Point2D closestObstaclePoint = new Point2D();
            double distanceToClosestPoint = Double.POSITIVE_INFINITY;
            for (Cluster cluster : allObstacleClusters)
            {
               Point2D closestPointInCluster = new Point2D();
               double distance = VisibilityTools.distanceToCluster(currentPosition2D, cluster.getNonNavigableExtrusionsInWorld2D(), closestPointInCluster, null);
               if (distance < distanceToClosestPoint)
               {
                  distanceToClosestPoint = distance;
                  closestObstaclePoint = closestPointInCluster;
               }
            }

            Vector2D vectorToObstacle = new Vector2D();
            vectorToObstacle.sub(closestObstaclePoint, currentPosition2D);
         }

         pathPosesToPack.get(pathIndex).getOrientation().setYawPitchRoll(desiredOrientation, 0.0, 0.0);
      }
   }

}
