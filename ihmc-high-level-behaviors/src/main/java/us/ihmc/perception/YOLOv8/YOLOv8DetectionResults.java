package us.ihmc.perception.YOLOv8;

import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.perception.RawImage;

import java.util.EnumMap;
import java.util.List;


// FIXME: Getting core dump somewhere here
public class YOLOv8DetectionResults
{
   private final List<YOLOv8Detection> detections;
   private final MatVector outputBlobs;
   private final RawImage detectionImage;
   private final FloatIndexer outputMasksIndexer;
   private final EnumMap<YOLOv8DetectableObject, Mat> objectMasks = new EnumMap<>(YOLOv8DetectableObject.class);

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

      boolean foundObject = false;
      int rowSize = (int) outputMasksIndexer.size(2);
      int columnSize = (int) outputMasksIndexer.size(3);
      Mat maskBooleanMat = new Mat(rowSize, columnSize, opencv_core.CV_8UC1, new Scalar(0.0));

      if (!detections.isEmpty())
      {
         for (YOLOv8Detection detection : detections)
         {
            // Find the detection that matches the query object type
            if (detection.objectClass() == objectType)
            {
               float[][] maskFloatMatrix = getFloatMaskMatrix(detection, numberOfMasks, rowSize, columnSize);

               // TODO: Use CV threshold here instead
               foundObject = getBooleanMaskMat(maskBooleanMat, detection, maskFloatMatrix, maskThreshold);
            }
         }
      }

      if (foundObject)
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
      objectMasks.put(objectType, null);
      maskBooleanMat.release();
      return null;
   }

   public float[][] getFloatMaskMatrix(YOLOv8Detection detection, int numberOfMasks, int rowSize, int columnSize)
   {
      float[][] maskFloatMatrix = new float[rowSize][columnSize];

      for (int i = 0; i < numberOfMasks; i++)
         for (int j = 0; j < rowSize; j++)
            for (int k = 0; k < columnSize; k++)
               maskFloatMatrix[j][k] += detection.maskWeights()[i] * outputMasksIndexer.get(0, i, j, k);

      return maskFloatMatrix;
   }

   private boolean getBooleanMaskMat(Mat booleanMaskToPack, YOLOv8Detection detection, float[][] maskFloatMatrix, float maskThreshold)
   {
      int rowSize = booleanMaskToPack.rows();
      int columnSize = booleanMaskToPack.cols();

      boolean foundObject = false;
      // TODO: Use CV threshold here instead
      for (int j = 0; j < rowSize; j++)
      {
         for (int k = 0; k < columnSize; k++)
         {
            if (maskFloatMatrix[j][k] >= maskThreshold
                && j >= detection.y() / 4 && j <= (detection.y() + detection.height()) / 4
                && k >= detection.x() / 4 && k <= (detection.x() + detection.width()) / 4)
            {
               booleanMaskToPack.data().put((long) j * columnSize + k, (byte) 255);
               foundObject = true;
            }
            else
               booleanMaskToPack.data().put((long) j * columnSize + k, (byte) 0);
         }
      }

      return foundObject;
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
