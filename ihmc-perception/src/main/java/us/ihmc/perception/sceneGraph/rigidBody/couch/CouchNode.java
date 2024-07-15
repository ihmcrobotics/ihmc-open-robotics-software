package us.ihmc.perception.sceneGraph.rigidBody.couch;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import perception_msgs.msg.dds.CouchNodeMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8Tools;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

import java.util.List;

public class CouchNode extends DetectableSceneNode
{
   private static final Scalar HSV_LOWER_BOUND = new Scalar(0.0, 180.0, 200.0, 0.0);
   private static final Scalar HSV_UPPER_BOUND = new Scalar(30.0, 220.0, 255.0, 255.0);

   private final RigidBodyTransform couchCentroidToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform pillowToWorldTransform = new RigidBodyTransform();

   private final PersistentDetection couchDetection;
   private final MutableReferenceFrame couchCentroidFrame = new MutableReferenceFrame();
   private final MutableReferenceFrame pillowFrame = new MutableReferenceFrame();

   private final OpenCLPointCloudExtractor pointCloudExtractor = new OpenCLPointCloudExtractor();

   public CouchNode(long id, String name, CRDTInfo crdtInfo)
   {
      this(id, name, null, crdtInfo);
   }

   public CouchNode(long id, String name, PersistentDetection couchDetection, CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);

      this.couchDetection = couchDetection;

      // Initially set everything to NaN
      couchCentroidToWorldTransform.setToNaN();
      pillowToWorldTransform.setToNaN();
      couchCentroidFrame.update(RigidBodyTransformBasics::setToNaN);
      pillowFrame.update(RigidBodyTransform::setToNaN);
   }

   @Override
   public void update(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue)
   {
      super.update(sceneGraph, modificationQueue);

      Point3DReadOnly couchCentroid = couchDetection.getMostRecentPosition();
      Point3DReadOnly pillowCentroid = computePillowCentroid((YOLOv8InstantDetection) couchDetection.getMostRecentDetection());

      if (!pillowCentroid.containsNaN() && !couchCentroid.containsNaN())
      {
         Vector3D couchToPillowDirection = new Vector3D(pillowCentroid);
         couchToPillowDirection.sub(couchCentroid);
         Line2D couchToPillowLine = new Line2D(couchCentroid.getX(), couchCentroid.getY(),
                                               couchToPillowDirection.getX(), couchToPillowDirection.getY());
         double couchYaw = TupleTools.angle(Axis2D.X, couchToPillowLine.getDirection());

         // Update transforms to world
         couchCentroidToWorldTransform.setTranslationAndIdentityRotation(couchCentroid);
         couchCentroidToWorldTransform.getRotation().setYawPitchRoll(couchYaw, 0.0, 0.0);
         pillowToWorldTransform.setTranslationAndIdentityRotation(pillowCentroid);
         pillowToWorldTransform.getRotation().setYawPitchRoll(couchYaw, 0.0, 0.0);

         // Update frames
         couchCentroidFrame.update(transformToWorld -> transformToWorld.set(couchCentroidToWorldTransform));
         pillowFrame.update(transformToWorld -> transformToWorld.set(pillowToWorldTransform));

         // Update node frame
         getNodeToParentFrameTransform().set(couchCentroidFrame.getTransformToParent());
         getNodeFrame().update();
      }
   }

   public void updateFromMessage(CouchNodeMessage message)
   {
      couchCentroidToWorldTransform.set(message.getCouchCentroidToWorldTransform());
      pillowToWorldTransform.set(message.getPillowToWorldTransform());
      couchCentroidFrame.update(transformToWorld -> transformToWorld.set(couchCentroidToWorldTransform));
      pillowFrame.update(transformToWorld -> transformToWorld.set(pillowToWorldTransform));
   }

   public RigidBodyTransformReadOnly getPillowToWorldTransform()
   {
      return pillowToWorldTransform;
   }

   public RigidBodyTransformReadOnly getCouchCentroidToWorldTransform()
   {
      return couchCentroidToWorldTransform;
   }

   public ReferenceFrame getCouchCentroidFrame()
   {
      return couchCentroidFrame.getReferenceFrame();
   }

   public ReferenceFrame getPillowFrame()
   {
      return pillowFrame.getReferenceFrame();
   }

   private Point3D32 computePillowCentroid(YOLOv8InstantDetection yoloDetection)
   {
      Point3D32 pillowCentroid = new Point3D32();
      pillowCentroid.setToNaN();

      RawImage colorImage = yoloDetection.getColorImage().get();
      RawImage depthImage = yoloDetection.getDepthImage().get();
      RawImage couchMask = yoloDetection.getObjectMask().get();

      try (Mat colorImageMat = colorImage.getCpuImageMat().clone();
           Mat depthImageMat = depthImage.getCpuImageMat().clone();
           Mat couchMaskMat = new Mat();
           Mat couchOnlyImage = new Mat();
           Mat pillowMask = new Mat();
           Mat pillowDepth = new Mat())
      {
         // Convert 32F mask to 8U
         couchMask.getCpuImageMat().convertTo(couchMaskMat, opencv_core.CV_8U, 255.0, 0.0);

         // Resize color image (larger) to match the mask (smaller)
         opencv_imgproc.resize(colorImageMat, colorImageMat, couchMaskMat.size(), 0.0, 0.0, opencv_imgproc.INTER_NEAREST);

         // Apply the mask to the color image
         opencv_core.bitwise_and(colorImageMat, colorImageMat, couchOnlyImage, couchMaskMat);

         // Convert the couch image to HSV color space
         opencv_imgproc.cvtColor(couchOnlyImage, couchOnlyImage, opencv_imgproc.COLOR_BGR2HSV);

         // Use HSV segmentation to get mask of the handle
         OpenCVTools.inRange(couchOnlyImage, HSV_LOWER_BOUND, HSV_UPPER_BOUND, pillowMask);

         // Resize the mask (smaller) to match the depth image (larger)
         opencv_imgproc.resize(pillowMask, pillowMask, depthImageMat.size(), 0.0, 0.0, opencv_imgproc.INTER_NEAREST);

         // Apply the mask to the depth image
         opencv_core.bitwise_and(depthImageMat, depthImageMat, pillowDepth, pillowMask);
         PerceptionDebugTools.display("handle mask", pillowMask, 1);

         // Get the point cloud of the handle
         RawImage trashCanHandleDepthImage = depthImage.replaceImage(pillowDepth);
         List<Point3D32> handlePoints = pointCloudExtractor.extractPointCloud(trashCanHandleDepthImage);
         trashCanHandleDepthImage.release();

         // Make sure we have points
         if (handlePoints.isEmpty() || handlePoints.size() < 32)
            return pillowCentroid;

         pillowCentroid = YOLOv8Tools.computeCentroidOfPointCloud(handlePoints, 128);
      }

      colorImage.release();
      depthImage.release();
      couchMask.release();

      return pillowCentroid;
   }

   @Override
   public void destroy(SceneGraph sceneGraph)
   {
      super.destroy(sceneGraph);

      if (couchDetection != null)
         couchDetection.markForDeletion();
   }
}
