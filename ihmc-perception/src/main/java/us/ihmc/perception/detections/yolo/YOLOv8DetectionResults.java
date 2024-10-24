package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatExpr;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.imageMessage.PixelFormat;

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

   private final int numberOfMasks;
   private final int maskWidth;
   private final int maskHeight;
   private final CameraIntrinsics maskIntrinsics;

   private final Mat zeroMat;

   public YOLOv8DetectionResults(Set<YOLOv8DetectionOutput> detections, MatVector outputBlobs, RawImage detectionImage, float maskThreshold)
   {
      this.detections = detections;
      this.outputBlobs = outputBlobs;
      this.detectionImage = detectionImage.get();
      this.outputMasksIndexer = outputBlobs.get(1).createIndexer();
      this.maskThreshold = maskThreshold;

      int maskOpenCVType = outputBlobs.get(1).col(0).type();
      numberOfMasks = (int) outputMasksIndexer.size(1);
      maskHeight = (int) outputMasksIndexer.size(2);
      maskWidth = (int) outputMasksIndexer.size(3);

      float xScaleFactor = (float) maskWidth / detectionImage.getWidth();
      float yScaleFactor = (float) maskHeight / detectionImage.getHeight();
      float maskFocalLengthX = xScaleFactor * detectionImage.getFocalLengthX();
      float maskFocalLengthY = yScaleFactor * detectionImage.getFocalLengthY();
      float maskPrincipalPointX = xScaleFactor * detectionImage.getPrincipalPointX();
      float maskPrincipalPointY = yScaleFactor * detectionImage.getPrincipalPointY();
      maskIntrinsics = new CameraIntrinsics(maskHeight, maskWidth, maskFocalLengthX, maskFocalLengthY, maskPrincipalPointX, maskPrincipalPointY);

      zeroMat = new Mat(maskHeight, maskWidth, maskOpenCVType, new Scalar(0.0));
   }

   public Map<YOLOv8DetectionOutput, RawImage> getSegmentationImages()
   {
      Map<YOLOv8DetectionOutput, RawImage> segmentationImages = new HashMap<>();

      synchronized (this)
      {
         for (YOLOv8DetectionOutput detection : detections)
         {
            Mat detectionMask = getDetectionMask(detection);

            if (detectionMask != null && !detectionMask.isNull()) // FIXME: This can be NULL
               segmentationImages.put(detection, createMaskRawImage(detectionMask));
         }
      }

      return segmentationImages;
   }

   public Map<YOLOv8DetectionOutput, RawImage> getTargetSegmentationImages(Set<String> targetClasses)
   {
      Map<YOLOv8DetectionOutput, RawImage> segmentationImages = new HashMap<>();

      synchronized (this)
      {
         for (YOLOv8DetectionOutput detection : detections)
         {
            if (targetClasses.contains(detection.objectClass()))
            {
               Mat detectionMask = getDetectionMask(detection);
               if (detectionMask != null && !detectionMask.isNull()) // FIXME: This can be NULL
                  segmentationImages.put(detection, createMaskRawImage(detectionMask));
            }
         }
      }

      return segmentationImages;
   }

   public RawImage getSegmentationMatrixForObject(String objectClass)
   {
      for (YOLOv8DetectionOutput detection : detections)
      {
         if (detection.objectClass().equals(objectClass))
            return createMaskRawImage(getDetectionMask(detection));
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
      MatExpr floatMask = new MatExpr(zeroMat);
      for (int i = 0; i < numberOfMasks; ++i)
      {
         Mat mask = outputBlobs.get(1).col(i).reshape(1, maskHeight);
         MatExpr weightMultipliedMask = opencv_core.multiply(mask, detection.maskWeights()[i]);
         floatMask = opencv_core.add(weightMultipliedMask, floatMask);

         mask.close();
         weightMultipliedMask.close();
      }

      return floatMask.asMat();
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

   private RawImage createMaskRawImage(Mat mask)
   {
      return new RawImage(mask,
                          null,
                          PixelFormat.UNKNOWN,
                          maskIntrinsics,
                          CameraModel.PINHOLE,
                          detectionImage.getPose(),
                          detectionImage.getAcquisitionTime(),
                          detectionImage.getSequenceNumber(),
                          -1.0f);
   }

   public synchronized void destroy()
   {
      for (Mat mat : objectMasks.values())
         if (mat != null && !mat.isNull())
            mat.close();

      detectionImage.release();
      outputMasksIndexer.close();
      outputBlobs.close();
      zeroMat.close();
   }
}