package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapSolution;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.AngleTools;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class PathOrientationCalculator
{
   private static final double epsilon = 5e-2;

   private final VisibilityGraphsParametersReadOnly parameters;

   public PathOrientationCalculator(VisibilityGraphsParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public List<Pose3DReadOnly> computePosesFromPath(List<Point3DReadOnly> path, VisibilityMapSolution visibilityMapSolution)
   {
      List<Pose3D> newPathPoses = new ArrayList<>();
      List<Cluster> allObstacleClusters = new ArrayList<>();
      visibilityMapSolution.getNavigableRegions().getNaviableRegionsList().forEach(region -> allObstacleClusters.addAll(region.getObstacleClusters()));

      List<Pose3D> nominalPathPoses = computeNominalPosesForPath(path);
      newPathPoses.add(nominalPathPoses.get(0));

      int pathIndex = 1;
      while (pathIndex < path.size() - 1)
      {
         Point3DReadOnly currentPosition = path.get(pathIndex);

         Point2D currentPosition2D = new Point2D(currentPosition);

         double previousOrientation = newPathPoses.get(pathIndex - 1).getOrientation().getYaw();
         double desiredOrientation = nominalPathPoses.get(pathIndex).getOrientation().getYaw();

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

            desiredOrientation = getHeadingToAvoidObstacles(desiredOrientation, previousOrientation, vectorToObstacle);
         }

         newPathPoses.add(new Pose3D(path.get(pathIndex), new Quaternion(desiredOrientation, 0.0, 0.0)));

         pathIndex++;
      }




      newPathPoses.add(nominalPathPoses.get(nominalPathPoses.size() - 1));

      return newPathPoses.parallelStream().map(Pose3D::new).collect(Collectors.toList());
   }

   private List<Pose3D> computeNominalPosesForPath(List<Point3DReadOnly> path)
   {
      List<Pose3D> nominalPathPoses = new ArrayList<>();

      double startHeading = BodyPathPlannerTools.calculateHeading(path.get(0), path.get(1));
      nominalPathPoses.add(new Pose3D(path.get(0), new Quaternion(startHeading, 0.0, 0.0)));

      int pathIndex = 1;

      while (pathIndex < path.size() - 1)
      {
         Point3DReadOnly previousPosition = path.get(pathIndex - 1);
         Point3DReadOnly currentPosition = path.get(pathIndex);
         Point3DReadOnly nextPosition = path.get(pathIndex + 1);

         double previousHeading = BodyPathPlannerTools.calculateHeading(previousPosition, currentPosition);
         double nextHeading = BodyPathPlannerTools.calculateHeading(currentPosition, nextPosition);
         double desiredOrientation = InterpolationTools.linearInterpolate(previousHeading, nextHeading, 0.5);

         if (!MathTools.epsilonEquals(previousHeading, nextHeading, 1e-3))
         {
            double previousLength = currentPosition.distanceXY(previousPosition);
            double nextLength = currentPosition.distanceXY(nextPosition);

            // add a point before
            if (previousLength > 2.0 * parameters.getPreferredObstacleExtrusionDistance())
            {
               double alpha = 1.0 - 2.0 * parameters.getPreferredObstacleExtrusionDistance() / previousLength;
               Point3DBasics waypointPositionToAdd = new Point3D();
               waypointPositionToAdd.interpolate(previousPosition, currentPosition, alpha);
               path.add(pathIndex, waypointPositionToAdd);
               nominalPathPoses.add(new Pose3D(waypointPositionToAdd, new Quaternion(previousHeading, 0.0, 0.0)));

               pathIndex++;
            }
            else if (previousLength > 0.25 * parameters.getPreferredObstacleExtrusionDistance())
            { // TODO extract magic number
               double alpha = 0.5;
               Point3DBasics waypointPositionToAdd = new Point3D();
               waypointPositionToAdd.interpolate(previousPosition, currentPosition, alpha);
               path.add(pathIndex, waypointPositionToAdd);
               nominalPathPoses.add(new Pose3D(waypointPositionToAdd, new Quaternion(previousHeading, 0.0, 0.0)));

               pathIndex++;
            }

            nominalPathPoses.add(new Pose3D(currentPosition, new Quaternion(desiredOrientation, 0.0, 0.0)));
            pathIndex++;

            // add a point after
            if (nextLength > 2.0 * parameters.getPreferredObstacleExtrusionDistance())
            {
               double alpha = 2.0 * parameters.getPreferredObstacleExtrusionDistance() / nextLength;
               Point3DBasics waypointPositionToAdd = new Point3D();
               waypointPositionToAdd.interpolate(currentPosition, nextPosition, alpha);
               path.add(pathIndex, waypointPositionToAdd);
            }
            else if (nextHeading > 0.25 * parameters.getPreferredObstacleExtrusionDistance())
            { // TODO extract magic number
               double alpha = 0.5;
               Point3DBasics waypointPositionToAdd = new Point3D();
               waypointPositionToAdd.interpolate(currentPosition, nextPosition, alpha);
               path.add(pathIndex, waypointPositionToAdd);
            }
         }
         else
         {
            nominalPathPoses.add(new Pose3D(path.get(pathIndex), new Quaternion(desiredOrientation, 0.0, 0.0)));
            pathIndex++;
         }

      }

      int endingSize = path.size();
      double endHeading = BodyPathPlannerTools.calculateHeading(path.get(endingSize - 2), path.get(endingSize - 1));

      nominalPathPoses.add(new Pose3D(path.get(endingSize - 1), new Quaternion(endHeading, 0.0, 0.0)));

      return nominalPathPoses;
   }

   private double getHeadingToAvoidObstacles(double nominalHeading, double previousHeading, Vector2DReadOnly headingToClosestObstacle)
   {
      double headingToObstacle = BodyPathPlannerTools.calculateHeading(headingToClosestObstacle);
      double distanceToObstacle = headingToClosestObstacle.length() + parameters.getObstacleExtrusionDistance();

      if (distanceToObstacle > parameters.getPreferredObstacleExtrusionDistance() - epsilon)
         return nominalHeading;

//      double rotationForAvoidance = computeRotationForAvoidanceUsingBox(distanceToObstacle, parameters.getObstacleExtrusionDistance());
      double rotationForAvoidance = computeRotationForAvoidanceUsingEllipse(distanceToObstacle, parameters.getObstacleExtrusionDistance(), parameters.getPreferredObstacleExtrusionDistance());

      TDoubleArrayList possibleHeadings = new TDoubleArrayList();
      possibleHeadings.add(AngleTools.trimAngleMinusPiToPi(headingToObstacle + rotationForAvoidance));
      possibleHeadings.add(AngleTools.trimAngleMinusPiToPi(headingToObstacle + rotationForAvoidance + Math.PI));
      possibleHeadings.add(AngleTools.trimAngleMinusPiToPi(headingToObstacle - rotationForAvoidance));
      possibleHeadings.add(AngleTools.trimAngleMinusPiToPi(headingToObstacle - rotationForAvoidance + Math.PI));

      double bestHeading = previousHeading;
      double smallestAngleDifference = Double.POSITIVE_INFINITY;
      for (int i = 0; i < possibleHeadings.size(); i++)
      {
         double heading = possibleHeadings.get(i);
         double angleDifference = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(heading, previousHeading));
         if (angleDifference < smallestAngleDifference)
         {
            bestHeading = heading;
            smallestAngleDifference = angleDifference;
         }
      }

      return bestHeading;
   }

   // assume a simple box
   private static double computeRotationForAvoidanceUsingBox(double distanceToObstacle, double minimumDistanceFromObstacle)
   {
      if (distanceToObstacle < minimumDistanceFromObstacle)
         return Math.PI / 2.0;
      else
         return Math.asin(minimumDistanceFromObstacle / distanceToObstacle);
   }

   private static double computeRotationForAvoidanceUsingEllipse(double distanceToObstacle, double minimumDistanceFromObstacle, double largerDistanceFromObstacle)
   {
      if (distanceToObstacle < minimumDistanceFromObstacle)
         return Math.PI / 2.0;
      else
      {
         double height = MathTools.square(minimumDistanceFromObstacle) * (MathTools.square(largerDistanceFromObstacle) - MathTools.square(distanceToObstacle));
         height /= (MathTools.square(largerDistanceFromObstacle) - MathTools.square(minimumDistanceFromObstacle));
         return Math.asin(height / distanceToObstacle);
      }
   }
}
