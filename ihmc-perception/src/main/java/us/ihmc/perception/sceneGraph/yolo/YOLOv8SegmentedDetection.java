package us.ihmc.perception.sceneGraph.yolo;

import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.YOLOv8.YOLOv8Detection;
import us.ihmc.perception.YOLOv8.YOLOv8Tools;
import us.ihmc.perception.filters.DetectionFilter;

import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;

public class YOLOv8SegmentedDetection
{
   private final YOLOv8Detection detection;
   private final DetectionFilter detectionFilter;
   private final List<Point3DReadOnly> objectPointCloud;
   private final Point3D32 centroid;

   public YOLOv8SegmentedDetection(YOLOv8Detection detection,
                                   DetectionFilter detectionFilter,
                                   RawImage mask,
                                   RawImage depthImage,
                                   BiFunction<RawImage, RawImage, RawImage> segmentImageFunction,
                                   Function<RawImage, List<Point3DReadOnly>> extractDepthFunction)
   {
      this.detection = detection;
      this.detectionFilter = detectionFilter;

      RawImage segmentedDepth = segmentImageFunction.apply(depthImage, mask);
      objectPointCloud = extractDepthFunction.apply(segmentedDepth);

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

   public List<Point3DReadOnly> getObjectPointCloud()
   {
      return objectPointCloud;
   }

   public Point3D32 getCentroid()
   {
      return centroid;
   }
}
