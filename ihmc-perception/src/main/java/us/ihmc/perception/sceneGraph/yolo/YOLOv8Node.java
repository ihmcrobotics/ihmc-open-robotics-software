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
import us.ihmc.perception.YOLOv8.YOLOv8DetectionClass;
import us.ihmc.perception.filters.BreakFrequencyAlphaCalculator;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;
import java.util.Locale;

public class YOLOv8Node extends DetectableSceneNode
{
   // Read from RDX node, write to YOLO Manager
   private int maskErosionKernelRadius;
   private double outlierFilterThreshold;
   private float detectionAcceptanceThreshold;

   // Read from YOLO manager, write to RDX
   private YOLOv8DetectionClass detectionClass;
   private List<Point3D32> objectPointCloud;
   private Point3D32 objectCentroid;

   // Set this somewhere
   private final RigidBodyTransform centroidToObjectTransform = new RigidBodyTransform();
   private Pose3D objectPose;
   private Pose3D filteredObjectPose;
   private final RigidBodyTransform visualTransformToObjectPose = new RigidBodyTransform();

   private final BreakFrequencyAlphaCalculator breakFrequencyAlphaCalculator = new BreakFrequencyAlphaCalculator();
   private double breakFrequency = 0.5;

   public YOLOv8Node(long id, String name, YOLOv8DetectionClass detectionClass, List<Point3D32> objectPointCloud, Point3D32 objectCentroid)
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

   public YOLOv8Node(long id,
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
      super(id, name);

      this.maskErosionKernelRadius = maskErosionKernelRadius;
      this.outlierFilterThreshold = outlierFilterThreshold;
      this.detectionAcceptanceThreshold = detectionAcceptanceThreshold;
      this.detectionClass = detectionClass;
      this.objectPointCloud = objectPointCloud;
      this.objectCentroid = objectCentroid;
      this.centroidToObjectTransform.set(centroidToObjectTransform);
      this.objectPose = objectPose;
      this.filteredObjectPose = filteredObjectPose;
      this.visualTransformToObjectPose.set(visualTransformToObjectPose);
   }

   public void update()
   {
      objectPose.getTranslation().set(objectCentroid);
      objectPose.appendTransform(centroidToObjectTransform);

      if (!filteredObjectPose.hasRotation())
         filteredObjectPose.getRotation().set(objectPose.getRotation());

      filteredObjectPose.interpolate(objectPose, breakFrequencyAlphaCalculator.calculateAlpha(breakFrequency));
      getNodeToParentFrameTransform().set(filteredObjectPose);
      getNodeFrame().update();
   }

   public void updatePlanarRegions(PlanarRegionsList planarRegionsList, ROS2Helper ros2Helper)
   {
      if (getName().toLowerCase(Locale.ROOT).contains("door lever") || getName().toLowerCase(Locale.ROOT).contains("door handle"))
      {
         Point3D objectCentroidInWorld = new Point3D(objectPose.getTranslation());

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

               double yaw = TupleTools.angle(Axis2D.X, doorLineNormal.getDirection());
               double pitch = TupleTools.angle(Axis2D.Y, doorLineNormal.getDirection()) + Math.PI;
               getObjectPose().getRotation().setYawPitchRoll(yaw, pitch, doorSide == RobotSide.LEFT ? Math.PI : 0.0);

               PlanarRegionsList doorPlanarRegionsList = new PlanarRegionsList();
               doorPlanarRegionsList.addPlanarRegion(doorPlanarRegion);

               FramePlanarRegionsList doorFramePlanarRegionsList = new FramePlanarRegionsList();
               doorFramePlanarRegionsList.setPlanarRegionsList(doorPlanarRegionsList);

               PerceptionMessageTools.publishFramePlanarRegionsList(doorFramePlanarRegionsList, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS, ros2Helper);
            }
         }
      }
   }

   public int getMaskErosionKernelRadius()
   {
      return maskErosionKernelRadius;
   }

   public void setMaskErosionKernelRadius(int maskErosionKernelRadius)
   {
      this.maskErosionKernelRadius = maskErosionKernelRadius;
   }

   public double getOutlierFilterThreshold()
   {
      return outlierFilterThreshold;
   }

   public void setOutlierFilterThreshold(double outlierFilterThreshold)
   {
      this.outlierFilterThreshold = outlierFilterThreshold;
   }

   public float getDetectionAcceptanceThreshold()
   {
      return detectionAcceptanceThreshold;
   }

   public void setDetectionAcceptanceThreshold(float detectionAcceptanceThreshold)
   {
      this.detectionAcceptanceThreshold = detectionAcceptanceThreshold;
   }

   public YOLOv8DetectionClass getDetectionClass()
   {
      return detectionClass;
   }

   public void setDetectionClass(YOLOv8DetectionClass detectionClass)
   {
      this.detectionClass = detectionClass;
   }

   public List<Point3D32> getObjectPointCloud()
   {
      return objectPointCloud;
   }

   public void setObjectPointCloud(List<Point3D32> objectPointCloud)
   {
      this.objectPointCloud = objectPointCloud;
   }

   public Point3D32 getObjectCentroid()
   {
      return objectCentroid;
   }

   public void setObjectCentroid(Point3D32 objectCentroid)
   {
      this.objectCentroid = objectCentroid;
   }

   public RigidBodyTransform getCentroidToObjectTransform()
   {
      return centroidToObjectTransform;
   }

   public void setCentroidToObjectTransform(RigidBodyTransformBasics centroidToObjectTransform)
   {
      this.centroidToObjectTransform.set(centroidToObjectTransform);
   }

   public Pose3D getObjectPose()
   {
      return objectPose;
   }

   public void setObjectPose(Pose3D objectPose)
   {
      this.objectPose = objectPose;
   }

   public Pose3D getFilteredObjectPose()
   {
      return filteredObjectPose;
   }

   public void setFilteredObjectPose(Pose3D filteredObjectPose)
   {
      this.filteredObjectPose = filteredObjectPose;
   }

   public RigidBodyTransform getVisualTransformToObjectPose()
   {
      return visualTransformToObjectPose;
   }

   public void setVisualTransformToObjectPose(RigidBodyTransformBasics visualTransformToObjectPose)
   {
      this.visualTransformToObjectPose.set(visualTransformToObjectPose);
   }

   public double getBreakFrequency()
   {
      return breakFrequency;
   }

   public void setBreakFrequency(double breakFrequency)
   {
      this.breakFrequency = breakFrequency;
   }
}
