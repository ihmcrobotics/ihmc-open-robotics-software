package us.ihmc.perception.sceneGraph.rigidBody.trashcan;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.YOLOv8.YOLOv8InstantDetection;
import us.ihmc.perception.detections.YOLOv8.YOLOv8Tools;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

import java.util.List;

public class TrashCanNode extends DetectableSceneNode
{
   private final static Scalar HSV_LOWER_BOUND = new Scalar(0.0, 0.0, 0.0, 0.0);
   private final static Scalar HSV_UPPER_BOUND = new Scalar(179.0, 255.0, 255.0, 255.0);

   private final OpenCLPointCloudExtractor pointCloudExtractor = new OpenCLPointCloudExtractor();

   private final MutableReferenceFrame trashCanFrame = new MutableReferenceFrame();

   // Not sent over ROS2
   private final PersistentDetection trashCanDetection;

   public TrashCanNode(long id, String name, CRDTInfo crdtInfo)
   {
      this(id, name, null, crdtInfo);
   }

   public TrashCanNode(long id, String name, PersistentDetection trashCanDetection, CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);
      this.trashCanDetection = trashCanDetection;
   }

   // on robot only
   @Override
   public void update(SceneGraph sceneGraph)
   {
      super.update(sceneGraph);

      Point3D32 handleCentroid = computeHandleCentroid((YOLOv8InstantDetection) trashCanDetection.getMostRecentDetection());
      Point3DReadOnly trashCanCentroid = trashCanDetection.getMostRecentPosition();

      Line2D trashCanToHandleLine = new Line2D(trashCanCentroid.getX(),
                                               trashCanCentroid.getY(),
                                               handleCentroid.getX(),
                                               handleCentroid.getY());
      double trashCanYaw = TupleTools.angle(Axis2D.X, trashCanToHandleLine.getDirection());

      RigidBodyTransform trashCanToWorldTransform = new RigidBodyTransform(new Quaternion(), trashCanCentroid);
      trashCanToWorldTransform.getRotation().setYawPitchRoll(trashCanYaw, 0.0, 0.0);
      trashCanFrame.update(transformToWorld -> transformToWorld.set(trashCanToWorldTransform));
      getNodeToParentFrameTransform().set(trashCanFrame.getTransformToParent());
      getNodeFrame().update();
   }

   private Point3D32 computeHandleCentroid(YOLOv8InstantDetection yoloDetection)
   {
      Point3D32 handleCentroid;

      RawImage colorImage = yoloDetection.getColorImage().get();
      RawImage depthImage = yoloDetection.getDepthImage().get();
      RawImage trashCanMask = yoloDetection.getObjectMask().get();

      try (Mat colorImageMat = colorImage.getCpuImageMat().clone();
           Mat depthImageMat = depthImage.getCpuImageMat().clone();
           Mat trashCanMaskMat = new Mat();
           Mat trashCanImage = new Mat();
           Mat trashCanHandleMask = new Mat();
           Mat trashCanHandleDepth = new Mat())
      {
         // Convert 32F mask to 8U
         trashCanMask.getCpuImageMat().convertTo(trashCanMaskMat, opencv_core.CV_8U, 255.0, 0.0);
         opencv_core.bitwise_and(colorImageMat, colorImageMat, trashCanImage, trashCanMaskMat);

         // Resize color image (larger) to match the mask (smaller)
         opencv_imgproc.resize(colorImageMat, colorImageMat, trashCanMaskMat.size(), 0.0, 0.0, opencv_imgproc.INTER_NEAREST);

         // Apply the mask to the color image
         opencv_core.bitwise_and(colorImageMat, colorImageMat, trashCanImage, trashCanMaskMat);

         // Convert the trashcan image to HSV color space
         opencv_imgproc.cvtColor(trashCanImage, trashCanImage, opencv_imgproc.COLOR_BGR2HSV);

         // Use HSV segmentation to get mask of the handle
         OpenCVTools.inRange(trashCanImage, HSV_LOWER_BOUND, HSV_UPPER_BOUND, trashCanHandleMask);

         // Resize the mask (smaller) to match the depth image (larger)
         opencv_imgproc.resize(trashCanHandleMask, trashCanHandleMask, depthImageMat.size(), 0.0, 0.0, opencv_imgproc.INTER_NEAREST);

         // Apply the mask to the depth image
         opencv_core.bitwise_and(depthImageMat, depthImageMat, trashCanHandleDepth, trashCanHandleMask);

         // Get the point cloud of the handle
         RawImage trashCanHandleDepthImage = depthImage.replaceImage(trashCanHandleDepth);
         List<Point3D32> handlePoints = pointCloudExtractor.extractPointCloud(trashCanHandleDepthImage);
         trashCanHandleDepthImage.release();

         handleCentroid = YOLOv8Tools.computeCentroidOfPointCloud(handlePoints, 128);
      }

      colorImage.release();
      depthImage.release();

      return handleCentroid;
   }

   @Override
   public void destroy(SceneGraph sceneGraph)
   {
      super.destroy(sceneGraph);

      if (trashCanDetection != null)
         trashCanDetection.markForDeletion();
   }
}
