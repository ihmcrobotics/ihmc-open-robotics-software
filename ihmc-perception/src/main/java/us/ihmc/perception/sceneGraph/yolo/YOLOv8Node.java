package us.ihmc.perception.sceneGraph.yolo;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.YOLOv8.YOLOv8Detection;
import us.ihmc.perception.YOLOv8.YOLOv8Tools;
import us.ihmc.perception.filters.DetectionFilter;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;

import java.util.List;

public class YOLOv8Node extends DetectableSceneNode
{
   // Read from RDX node, write to YOLO Manager
   private int maskErosionKernelRadius = 2;
   private double outlierFilterThreshold = 2.0;
   private float detectionAcceptanceThreshold = 0.2f;

   // Read from YOLO Manager, write to RDX node
   private YOLOv8Detection yoloDetection;
   private DetectionFilter detectionFilter;
   private List<Point3DReadOnly> objectPointCloud;
   private Pose3D objectCentroid;

   private RigidBodyTransform centroidToPoseTransform;

   public YOLOv8Node(long id, String name, YOLOv8Detection yoloDetection, DetectionFilter detectionFilter)
   {
      super(id, name);

      this.yoloDetection = yoloDetection;
      this.detectionFilter = detectionFilter;
   }

   public boolean update()
   {
      detectionFilter.setAcceptanceThreshold(detectionAcceptanceThreshold);
      if (detectionFilter.isStableDetectionResult())
      {
         objectPointCloud = YOLOv8Tools.filterOutliers(objectPointCloud, outlierFilterThreshold, 200);
         Point3D32 centroidPoint = YOLOv8Tools.computeCentroidOfPointCloud(objectPointCloud, 200);
         objectCentroid.set(centroidPoint, new RotationMatrix());

         return true;
      }

      return false;
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

   public YOLOv8Detection getYoloDetection()
   {
      return yoloDetection;
   }

   public void setYoloDetection(YOLOv8Detection yoloDetection)
   {
      this.yoloDetection = yoloDetection;
   }

   public DetectionFilter getDetectionFilter()
   {
      return detectionFilter;
   }

   public List<Point3DReadOnly> getObjectPointCloud()
   {
      return objectPointCloud;
   }

   public void setObjectPointCloud(List<Point3DReadOnly> objectPointCloud)
   {
      this.objectPointCloud = objectPointCloud;
   }

   public Pose3D getObjectCentroid()
   {
      return objectCentroid;
   }

   public void setCentroidToPoseTransform(RigidBodyTransform centroidToPoseTransform)
   {
      this.centroidToPoseTransform = centroidToPoseTransform;
   }
}
