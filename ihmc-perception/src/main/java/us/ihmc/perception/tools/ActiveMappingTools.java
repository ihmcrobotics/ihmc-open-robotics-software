package us.ihmc.perception.tools;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;

public class ActiveMappingTools
{
   public static void setStraightGoalFootPoses(Pose3D leftSolePose, Pose3D rightSolePose, Pose3D leftGoalPose, Pose3D rightGoalPose, float distance)
   {
      LogTools.info("Start Pose: {}, Goal Pose: {}", leftSolePose, leftGoalPose);

      leftSolePose.setZ(0);
      rightSolePose.setZ(0);

      leftGoalPose.set(leftSolePose);
      leftGoalPose.appendTranslation(distance, 0.0, 0.0);

      rightGoalPose.set(rightSolePose);
      rightGoalPose.appendTranslation(distance, 0.0, 0.0);
   }

   public static void getFrontierPoints(ArrayList<Point2D> frontierPoints, PlanarRegionsList planarRegionMap, Point2D center)
   {
      float resolution = 0.2f;
      frontierPoints.clear();
      for (int i = 0; i < 25; i++)
      {
         for (int j = 0; j < 25; j++)
         {
            Point3D point = new Point3D(center.getX() + resolution * i, center.getY() + resolution * j, 0.0);

            for (PlanarRegion region : planarRegionMap.getPlanarRegionsAsList())
            {
               Point3D projectedPoint = PlanarRegionTools.projectInZToPlanarRegion(point, planarRegionMap.getPlanarRegionsAsList().get(i));

               if (PlanarRegionTools.isPointInWorldInsidePlanarRegion(region, projectedPoint))
               {
                  frontierPoints.add(new Point2D(point.getX(), point.getY()));
               }
            }
         }
      }
   }
}
