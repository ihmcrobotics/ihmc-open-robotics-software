package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.perception.RawImage;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class YOLOv8DetectionResults
{
   private final Set<YOLOv8DetectionOutput> detections;
   private final MatVector outputBlobs;
   private final RawImage detectionImage;
   private final FloatIndexer outputMasksIndexer;
   private final Map<YOLOv8DetectionOutput, Mat> objectMasks = new HashMap<>();

   private final float maskThreshold;

   private final int maskOpenCVType;
   private final int numberOfMasks;
   private final int maskWidth;
   private final int maskHeight;
   private final float maskFocalLengthX;
   private final float maskFocalLengthY;
   private final float maskPrincipalPointX;
   private final float maskPrincipalPointY;

   public YOLOv8DetectionResults(Set<YOLOv8DetectionOutput> detections, MatVector outputBlobs, RawImage detectionImage, float maskThreshold)
   {
      this.detections = detections;
      this.outputBlobs = outputBlobs;
      this.detectionImage = detectionImage.get();
      this.outputMasksIndexer = outputBlobs.get(1).createIndexer();
      this.maskThreshold = maskThreshold;

      maskOpenCVType = outputBlobs.get(1).col(0).type();
      numberOfMasks = (int) outputMasksIndexer.size(1);
      maskHeight = (int) outputMasksIndexer.size(2);
      maskWidth = (int) outputMasksIndexer.size(3);

      float xScaleFactor = (float) maskWidth / detectionImage.getImageWidth();
      float yScaleFactor = (float) maskHeight / detectionImage.getImageHeight();
      maskFocalLengthX = xScaleFactor * detectionImage.getFocalLengthX();
      maskFocalLengthY = yScaleFactor * detectionImage.getFocalLengthY();
      maskPrincipalPointX = xScaleFactor * detectionImage.getPrincipalPointX();
      maskPrincipalPointY = yScaleFactor * detectionImage.getPrincipalPointY();
   }

   public Map<YOLOv8DetectionOutput, RawImage> getSegmentationImages()
   {
      Map<YOLOv8DetectionOutput, RawImage> segmentationImages = new HashMap<>();

      for (YOLOv8DetectionOutput detection : detections)
      {
         Mat detectionMask = getDetectionMask(detection);
         segmentationImages.put(detection, createRawImageWithMat(detectionMask));
      }

      return segmentationImages;
   }

   public Map<YOLOv8DetectionOutput, RawImage> getTargetSegmentationImages(Set<String> targetClasses)
   {
      Map<YOLOv8DetectionOutput, RawImage> segmentationImages = new HashMap<>();

      for (YOLOv8DetectionOutput detection : detections)
      {
         if (targetClasses.contains(detection.objectClass()))
         {
            Mat detectionMask = getDetectionMask(detection);
            segmentationImages.put(detection, createRawImageWithMat(detectionMask));
         }
      }

      return segmentationImages;
   }

   public RawImage getSegmentationMatrixForObject(String objectClass)
   {
      for (YOLOv8DetectionOutput detection : detections)
      {
         if (detection.objectClass().equals(objectClass))
            return createRawImageWithMat(getDetectionMask(detection));
      }

      return null;
   }

   public Set<YOLOv8DetectionOutput> getDetections()
   {
      return detections;
   }

   private Mat getDetectionMask(YOLOv8DetectionOutput detection)
   {
      if (objectMasks.containsKey(detection))
         return objectMasks.get(detection);

      Mat floatMaskMat = getFloatMaskMat(detection);
      Mat booleanMaskMat = getBooleanMaskMat(detection, floatMaskMat);
      objectMasks.put(detection, booleanMaskMat);
      floatMaskMat.close();
      return booleanMaskMat;
   }

   private Mat getFloatMaskMat(YOLOv8DetectionOutput detection)
   {
      Mat floatMaskMat = new Mat(maskHeight, maskWidth, maskOpenCVType, new Scalar(0.0));
      Mat multipliedMask = new Mat(maskHeight, maskWidth, maskOpenCVType);
      for (int i = 0; i < numberOfMasks; ++i)
      {
         Mat mask = outputBlobs.get(1).col(i).reshape(1, maskHeight);
         mask.convertTo(multipliedMask, maskOpenCVType, detection.maskWeights()[i], 0.0);
         opencv_core.add(floatMaskMat, multipliedMask, floatMaskMat);
         mask.close();
      }

      multipliedMask.close();

      return floatMaskMat;
   }

   private Mat getBooleanMaskMat(YOLOv8DetectionOutput detection, Mat maskFloatMat)
   {
      // Use the float mat to threshold
      Mat booleanMask = new Mat(maskHeight, maskWidth, opencv_core.CV_32FC1);
      opencv_imgproc.threshold(maskFloatMat, booleanMask, maskThreshold, 1.0, opencv_imgproc.CV_THRESH_BINARY);

      Mat boundingBoxMask = new Mat(maskHeight, maskWidth, opencv_core.CV_32FC1, new Scalar(0.0));
      opencv_imgproc.rectangle(boundingBoxMask,
                               new Rect(detection.x() / 4, detection.y() / 4, detection.width() / 4, detection.height() / 4),
                               new Scalar(1.0), opencv_imgproc.FILLED, opencv_imgproc.LINE_8, 0);

      opencv_core.bitwise_and(booleanMask, boundingBoxMask, booleanMask);
      boundingBoxMask.close();

      return booleanMask;
   }

   private RawImage createRawImageWithMat(Mat mat)
   {
      return new RawImage(detectionImage.getSequenceNumber(),
                          detectionImage.getAcquisitionTime(),
                          -1.0f,
                          mat.clone(),
                          null,
                          maskFocalLengthX,
                          maskFocalLengthY,
                          maskPrincipalPointX,
                          maskPrincipalPointY,
                          detectionImage.getPosition(),
                          detectionImage.getOrientation());
   }

   public void destroy()
   {
      for (Mat mat : objectMasks.values())
         if (mat != null && !mat.isNull())
            mat.close();

      detectionImage.release();
      outputMasksIndexer.close();
      outputBlobs.close();
   }
}