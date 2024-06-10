package us.ihmc.perception.YOLOv8;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;

import java.util.List;

public class YOLOv8Node extends DetectableSceneNode
{
   // Read from RDX node, write to YOLO Manager
   private float detectionAcceptanceThreshold;

   // Read from YOLO manager, write to RDX
   private YOLOv8Detection detection;
   private List<Point3D32> objectPointCloud;
   private Point3D32 objectCentroid;

   // Set this somewhere
   private final RigidBodyTransform centroidToObjectTransform = new RigidBodyTransform();
   private Pose3D objectPose;

   public YOLOv8Node(long id, String name, YOLOv8Detection detection, List<Point3D32> objectPointCloud, Point3D32 objectCentroid)
   {
      this(id,
           name,
           0.2f,
           detection,
           objectPointCloud,
           objectCentroid,
           new Pose3D(objectCentroid, new RotationMatrix()),
           new Pose3D(objectCentroid, new RotationMatrix()));
   }

   public YOLOv8Node(long id,
                     String name,
                     float detectionAcceptanceThreshold,
                     YOLOv8Detection detection,
                     List<Point3D32> objectPointCloud,
                     Point3D32 objectCentroid,
                     RigidBodyTransformBasics centroidToObjectTransform,
                     Pose3D objectPose)
   {
      super(id, name);

      this.detectionAcceptanceThreshold = detectionAcceptanceThreshold;
      this.detection = detection;
      this.objectPointCloud = objectPointCloud;
      this.objectCentroid = objectCentroid;
      this.centroidToObjectTransform.set(centroidToObjectTransform);
      this.objectPose = objectPose;
   }

   public void update()
   {
      objectPose.getTranslation().set(objectCentroid);
//      objectPose.appendTransform(centroidToObjectTransform);

      getNodeToParentFrameTransform().set(objectPose);
      getNodeFrame().update();
   }

   public float getDetectionAcceptanceThreshold()
   {
      return detectionAcceptanceThreshold;
   }

   public void setDetectionAcceptanceThreshold(float detectionAcceptanceThreshold)
   {
      this.detectionAcceptanceThreshold = detectionAcceptanceThreshold;
   }

   public YOLOv8Detection getDetection()
   {
      return detection;
   }

   public void setDetection(YOLOv8Detection detection)
   {
      this.detection = detection;
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
}
