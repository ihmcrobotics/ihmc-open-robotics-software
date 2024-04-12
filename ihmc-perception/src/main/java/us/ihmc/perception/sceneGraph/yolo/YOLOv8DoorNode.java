package us.ihmc.perception.sceneGraph.yolo;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionClass;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

public class YOLOv8DoorNode extends YOLOv8Node
{
   private static final int SWITCH_SIDE_THRESHOLD = 10;
   private int switchSide = 0;
   private RobotSide lastSide;

   public YOLOv8DoorNode(long id, String name, YOLOv8DetectionClass detectionClass, List<Point3D32> objectPointCloud, Point3D32 objectCentroid)
   {
      this(id,
           name,
           2,
           2.0,
           0.2f,
           detectionClass,
           objectPointCloud,
           objectCentroid,
           new RigidBodyTransform(),
           new Pose3D(objectCentroid, new RotationMatrix()),
           new Pose3D(objectCentroid, new RotationMatrix()),
           new RigidBodyTransform());
   }

   public YOLOv8DoorNode(long id,
                         String name,
                         int maskErosionKernelRadius,
                         double outlierFilterThreshold,
                         float detectionAcceptanceThreshold,
                         YOLOv8DetectionClass detectionClass,
                         List<Point3D32> objectPointCloud,
                         Point3D32 objectCentroid,
                         RigidBodyTransformBasics centroidToObjectTransform,
                         Pose3D objectPose,
                         Pose3D filteredObjectPose,
                         RigidBodyTransformBasics visualTransformToObjectPose)
   {
      super(id,
            name,
            maskErosionKernelRadius,
            outlierFilterThreshold,
            detectionAcceptanceThreshold,
            detectionClass,
            objectPointCloud,
            objectCentroid,
            centroidToObjectTransform,
            objectPose,
            filteredObjectPose,
            visualTransformToObjectPose);
   }

   public void updatePlanarRegions(PlanarRegionsList planarRegionsList, ROS2Helper ros2Helper)
   {
      Point3D objectCentroidInWorld = new Point3D(getObjectPose().getTranslation());

      if (!planarRegionsList.isEmpty())
      {
         float epsilon = 0.75f;

         // TODO: fixme doesn't work
         //            PlanarRegion doorPlanarRegion = planarRegionsList.findClosestPlanarRegionToPointByProjectionOntoXYPlane(doorLeverPointInWorld.getX(),
         //                                                                                                                    doorLeverPointInWorld.getY());

         PlanarRegion doorPlanarRegion = null;
         Point3DReadOnly doorPlanarRegionCentroidInWorld = null;

         for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
         {
            Point3DReadOnly planarRegionCentroidInWorld = PlanarRegionTools.getCentroid3DInWorld(planarRegion);

            if (planarRegionCentroidInWorld.distance(objectCentroidInWorld) > epsilon)
            {
               continue;
            }

            if (planarRegion.getArea() < 0.2)
            {
               continue;
            }

            if (doorPlanarRegion == null)
            {
               doorPlanarRegion = planarRegion;
               doorPlanarRegionCentroidInWorld = planarRegionCentroidInWorld;
               continue;
            }

            if (planarRegion.getArea() > doorPlanarRegion.getArea())
            {
               doorPlanarRegion = planarRegion;
               doorPlanarRegionCentroidInWorld = planarRegionCentroidInWorld;
            }
         }

         if (doorPlanarRegion != null)
         {
            Line2D doorLineNormal = new Line2D(doorPlanarRegionCentroidInWorld.getX(),
                                               doorPlanarRegionCentroidInWorld.getY(),
                                               doorPlanarRegion.getNormalX(),
                                               doorPlanarRegion.getNormalY());
            Point2D doorLeverPointInWorld2D = new Point2D(objectCentroidInWorld);

            RobotSide doorSide = doorLineNormal.isPointOnLeftSideOfLine(doorLeverPointInWorld2D) ? RobotSide.RIGHT : RobotSide.LEFT;

            if (lastSide == null)
               lastSide = doorSide;

            // Glitch filter
            if (lastSide != doorSide)
            {
               if (++switchSide > SWITCH_SIDE_THRESHOLD)
               {
                  // Switch sides
                  switchSide = 0;
                  LogTools.info("Door lever switched sides");
               }
               else
               {
                  doorSide = lastSide;
               }
            }

            double yaw = TupleTools.angle(Axis2D.X, doorLineNormal.getDirection());
            //               double pitch = TupleTools.angle(Axis2D.Y, doorLineNormal.getDirection()) + Math.PI;
            if (getName().toLowerCase().contains("door lever"))
               getObjectPose().getRotation().setYawPitchRoll(yaw, 0.0, doorSide == RobotSide.LEFT ? Math.PI : 0.0);
            else
               getObjectPose().getRotation().setToYawOrientation(yaw);

            PlanarRegionsList doorPlanarRegionsList = new PlanarRegionsList();
            doorPlanarRegionsList.addPlanarRegion(doorPlanarRegion);

            FramePlanarRegionsList doorFramePlanarRegionsList = new FramePlanarRegionsList();
            doorFramePlanarRegionsList.setPlanarRegionsList(doorPlanarRegionsList);

            PerceptionMessageTools.publishFramePlanarRegionsList(doorFramePlanarRegionsList, PerceptionAPI.PERSPECTIVE_DOOR_RAPID_REGION, ros2Helper);
         }
      }
   }
}
