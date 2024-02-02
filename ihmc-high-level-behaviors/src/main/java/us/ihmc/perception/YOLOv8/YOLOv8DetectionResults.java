package us.ihmc.perception.YOLOv8;

import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;

import java.util.EnumMap;
import java.util.List;

public class YOLOv8DetectionResults
{
   private final static Stopwatch totalWatch = new Stopwatch();
   private final static Stopwatch floatWatch = new Stopwatch();
   private double floatTime = 0.0;
   private final static Stopwatch booleanWatch = new Stopwatch();
   private double booleanTime = 0.0;

   private final List<YOLOv8Detection> detections;
   private final MatVector outputBlobs;
   private final RawImage detectionImage;
   private final FloatIndexer outputMasksIndexer;
   private final EnumMap<YOLOv8DetectableObject, Mat> objectMasks = new EnumMap<>(YOLOv8DetectableObject.class);

   private final int maskOpenCVType;
   private final int numberOfMasks;
   private final int maskWidth;
   private final int maskHeight;
   private final float maskFocalLengthX;
   private final float maskFocalLengthY;
   private final float maskPrincipalPointX;
   private final float maskPrincipalPointY;

   public YOLOv8DetectionResults(List<YOLOv8Detection> detections, MatVector outputBlobs, RawImage detectionImage)
   {
      this.detections = detections;
      this.outputBlobs = outputBlobs;
      this.detectionImage = detectionImage.get();
      this.outputMasksIndexer = outputBlobs.get(1).createIndexer();

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

   public RawImage getSegmentationMatrixForObject(YOLOv8DetectableObject objectType, float maskThreshold)
   {
      if (objectMasks.containsKey(objectType))
      {
         Mat mask = objectMasks.get(objectType);
         return new RawImage(detectionImage.getSequenceNumber(),
                             detectionImage.getAcquisitionTime(),
                             maskWidth,
                             maskHeight,
                             -1.0f,
                             mask,
                             null,
                             mask.type(),
                             maskFocalLengthX,
                             maskFocalLengthY,
                             maskPrincipalPointX,
                             maskPrincipalPointY,
                             detectionImage.getPosition(),
                             detectionImage.getOrientation());
      }

      totalWatch.start();
      Mat maskBooleanMat = null;
      if (!detections.isEmpty())
      {
         for (YOLOv8Detection detection : detections)
         {
            // Find the detection that matches the query object type
            if (detection.objectClass() == objectType)
            {
               Mat floatMaskMat = getFloatMaskMatrix(detection);
               maskBooleanMat = getBooleanMaskMat(detection, floatMaskMat, maskThreshold);
               floatMaskMat.close();
            }
         }
      }
      double totalElapsed = totalWatch.totalElapsed();
      LogTools.info("Total ellapsed: " + totalElapsed + " | float time: " + floatTime + " | bool time: " + booleanTime);

      if (maskBooleanMat != null)
      {
         objectMasks.put(objectType, maskBooleanMat);
         return new RawImage(detectionImage.getSequenceNumber(),
                             detectionImage.getAcquisitionTime(),
                             maskWidth,
                             maskHeight,
                             -1.0f,
                             maskBooleanMat,
                             null,
                             maskBooleanMat.type(),
                             maskFocalLengthX,
                             maskFocalLengthY,
                             maskPrincipalPointX,
                             maskPrincipalPointY,
                             detectionImage.getPosition(),
                             detectionImage.getOrientation());
      }

      // Did not find object we're looking for
      Mat previousValue = objectMasks.putIfAbsent(objectType, null);
      if (previousValue != null)
         previousValue.close();

      return null;
   }

   private Mat getFloatMaskMatrix(YOLOv8Detection detection)
   {
      floatWatch.start();
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

      floatTime = floatWatch.totalElapsed();
      return floatMaskMat;
   }

   private Mat getBooleanMaskMat(YOLOv8Detection detection, Mat maskFloatMat, double maskThreshold)
   {
      booleanWatch.start();
      // Use the float mat to threshold
      Mat booleanMask = new Mat(maskHeight, maskWidth, opencv_core.CV_32FC1);
      opencv_imgproc.threshold(maskFloatMat, booleanMask, maskThreshold, 1.0, opencv_imgproc.CV_THRESH_BINARY);

      Mat boundingBoxMask = new Mat(maskHeight, maskWidth, opencv_core.CV_32FC1, new Scalar(0.0));
      opencv_imgproc.rectangle(boundingBoxMask,
                               new Rect(detection.x() / 4, detection.y() / 4, detection.width() / 4, detection.height() / 4),
                               new Scalar(1.0), opencv_imgproc.FILLED, opencv_imgproc.LINE_8, 0);

      opencv_core.bitwise_and(booleanMask, boundingBoxMask, booleanMask);
      boundingBoxMask.close();

      booleanTime = booleanWatch.totalElapsed();
      return booleanMask;
   }

   public void destroy()
   {
      for (Mat mat : objectMasks.values())
         if (mat != null && !mat.isNull())
            mat.release();

      detectionImage.release();
      outputMasksIndexer.close();
      outputBlobs.close();
   }
}
