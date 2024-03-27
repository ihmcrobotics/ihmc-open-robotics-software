package us.ihmc.perception.sceneGraph.yolo;

import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.YOLOv8.YOLOv8Detection;
import us.ihmc.perception.YOLOv8.YOLOv8Tools;
import us.ihmc.perception.filters.DetectionFilter;

import java.util.Collections;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;

public class YOLOv8SegmentedDetection
{
   private final YOLOv8Detection detection;
   private final DetectionFilter detectionFilter;
   private final List<Point3D32> objectPointCloud;
   private final Point3D32 centroid;

   public YOLOv8SegmentedDetection(YOLOv8Detection detection,
                                   DetectionFilter detectionFilter,
                                   RawImage mask,
                                   RawImage depthImage,
                                   int erosionKernelRadius,
                                   BiFunction<RawImage, RawImage, RawImage> segmentImageFunction,
                                   Function<RawImage, List<Point3D32>> extractDepthFunction,
                                   double zScoreThreshold)
   {
      this.detection = detection;
      this.detectionFilter = detectionFilter;

      // Erode mask to get better segmentation
      Mat erodedMat = new Mat(mask.getImageHeight(), mask.getImageWidth(), mask.getOpenCVType());
      opencv_imgproc.erode(mask.getCpuImageMat(),
                           erodedMat,
                           opencv_imgproc.getStructuringElement(opencv_imgproc.CV_SHAPE_RECT,
                                                                new Size(2 * erosionKernelRadius + 1, 2 * erosionKernelRadius + 1),
                                                                new Point(erosionKernelRadius, erosionKernelRadius)));
      erodedMat.copyTo(mask.getCpuImageMat()); // RawImage isn't designed for this, but it works... 10/10 wouldn't recommend.
      mask.getGpuImageMat().upload(erodedMat);

      // Get segmented depth image
      RawImage segmentedDepth = segmentImageFunction.apply(depthImage, mask);
      // Get point cloud with outliers filtered ot
      objectPointCloud = YOLOv8Tools.filterOutliers(extractDepthFunction.apply(segmentedDepth), zScoreThreshold, 128);
      // Shuffle the point cloud
      Collections.shuffle(objectPointCloud);
      // Find point cloud centroid
      centroid = YOLOv8Tools.computeCentroidOfPointCloud(objectPointCloud, 128);
   }

   public YOLOv8Detection getDetection()
   {
      return detection;
   }

   public DetectionFilter getDetectionFilter()
   {
      return detectionFilter;
   }

   public List<Point3D32> getObjectPointCloud()
   {
      return objectPointCloud;
   }

   public Point3D32 getCentroid()
   {
      return centroid;
   }
}
