package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_dnn;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatExpr;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.RectVector;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_core.StringVector;
import org.bytedeco.opencv.opencv_dnn.Net;
import org.bytedeco.opencv.opencv_text.FloatVector;
import org.bytedeco.opencv.opencv_text.IntVector;
import org.yaml.snakeyaml.Yaml;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.imageMessage.PixelFormat;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class YOLOv8Model
{
   private static final double SCALE_FACTOR = 1.0 / 255.0;
   private static final Size DETECTION_SIZE = new Size(1280, 736);

   // Meta data
   private final String modelName;
   private final List<String> detectionClassNames = new ArrayList<>();
   private final Path onnxFile;

   // OpenCV DNN stuff
   private final Net yoloNet;
   private final StringVector outputNames; // literally list of "output0", "output1", "output2"...

   public YOLOv8Model(Path modelBaseDirectory)
   {
      // Ensure the passed in directory is a valid YOLO model directory
      if (!YOLOv8Tools.isValidYOLOModelDirectory(modelBaseDirectory))
         throw new IllegalArgumentException("Provided directory is not a YOLO model directory");

      // Get name & onnx file path
      modelName = modelBaseDirectory.getFileName().toString();
      onnxFile = YOLOv8Tools.getONNXFile(modelBaseDirectory);

      // Parse class_names.yaml
      try (InputStream inputStream = new FileInputStream(YOLOv8Tools.getClassNamesFile(modelBaseDirectory).toFile()))
      {
         Yaml yaml = new Yaml();
         Map<String, Object> classNamesData = yaml.load(inputStream);
         List<String> names = (List<String>) classNamesData.get("names");
         detectionClassNames.addAll(names);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      // Create OpenCV DNN objects
      yoloNet = opencv_dnn.readNetFromONNX(readONNXFile());

      // Use CUDA if available
      if (opencv_core.getCudaEnabledDeviceCount() > 0)
      {
         yoloNet.setPreferableBackend(opencv_dnn.DNN_BACKEND_CUDA);
         yoloNet.setPreferableTarget(opencv_dnn.DNN_TARGET_CUDA);
      }
      else
      {
         yoloNet.setPreferableBackend(opencv_dnn.DNN_BACKEND_OPENCV);
         yoloNet.setPreferableTarget(opencv_dnn.DNN_TARGET_CPU);
      }

      outputNames = yoloNet.getUnconnectedOutLayersNames();
   }

   public String getName()
   {
      return modelName;
   }

   public List<String> getDetectionClassNames()
   {
      return detectionClassNames;
   }

   public String getObjectClassFromIndex(int i)
   {
      return detectionClassNames.get(i);
   }

   public List<YOLOv8Detection> run(RawImage image, float confidenceThreshold, float nmsThreshold, float maskThreshold)
   {
      RawImage bgrImage;
      if (image.getPixelFormat() == PixelFormat.BGR8)
      {
         bgrImage = image.get();
         if (bgrImage == null)
            return new ArrayList<>();
      }
      else
      {
         Mat bgrMat = new Mat();
         image.getPixelFormat().convertToPixelFormat(image.getCpuImageMat(), bgrMat, PixelFormat.BGR8);
         bgrImage = image.replaceImage(bgrMat, PixelFormat.BGR8);
      }

      Mat blob = opencv_dnn.blobFromImage(bgrImage.getCpuImageMat(), SCALE_FACTOR, DETECTION_SIZE, new Scalar(), true, true, opencv_core.CV_32F);
      MatVector outputBlobs = new MatVector(outputNames.size());

      yoloNet.setInput(blob);
      yoloNet.forward(outputBlobs, outputNames);

      processOutput(bgrImage, outputBlobs, confidenceThreshold, nmsThreshold);

      bgrImage.release();

      return null; // TODO: Finish
   }

   private void processOutput(RawImage detectionImage, MatVector outputBlobs, float confidenceThreshold, float nonMaximumSuppressionThreshold)
   {
      int imageWidth = detectionImage.getWidth();
      int imageHeight = detectionImage.getHeight();
      int shiftWidth = (imageWidth - DETECTION_SIZE.width()) / 2;
      int shiftHeight = (imageHeight - DETECTION_SIZE.height()) / 2;
      int numberOfMasks = outputBlobs.get(1).size(1);

      try (FloatIndexer output0Indexer = outputBlobs.get(0).createIndexer();
           IntVector detectedClassIds = new IntVector();
           FloatVector detectedConfidences = new FloatVector();
           RectVector detectedBoxes = new RectVector();
           FloatVector detectedMaskWeights = new FloatVector())
      {
         for (long i = 0; i < output0Indexer.size(2); i++)
         {
            // Find most confident class detection
            float maxConfidence = 0;
            long maxConfidenceClass = 0;
            for (long j = 0; j < detectionClassNames.size(); j++)
            {
               float confidence = output0Indexer.get(0, 4 + j, i);
               if (confidence > maxConfidence)
               {
                  maxConfidence = confidence;
                  maxConfidenceClass = j;
               }
            }
            // Ensure confidence is above threshold
            if (maxConfidence >= confidenceThreshold)
            {
               int centerX = (int) (output0Indexer.get(0, 0, i));
               int centerY = (int) (output0Indexer.get(0, 1, i));
               int width = (int) (output0Indexer.get(0, 2, i));
               int height = (int) (output0Indexer.get(0, 3, i));
               int left = centerX - width / 2;
               int top = centerY - height / 2;

               detectedClassIds.push_back((int) maxConfidenceClass);
               detectedConfidences.push_back(maxConfidence);
               detectedBoxes.push_back(new Rect(left, top, width, height));
               for (long k = 0; k < numberOfMasks; k++)
               {
                  detectedMaskWeights.push_back(output0Indexer.get(0, detectionClassNames.size() + 4 + k, i));
               }
            }
         }
         IntPointer reducedIndices = new IntPointer(detectedConfidences.size());
         FloatPointer confidencesPointer = new FloatPointer(detectedConfidences.size());

         if (detectedBoxes.size() > 0)
         {
            // remove overlapping bounding boxes with NMS
            confidencesPointer.put(detectedConfidences.get());
            opencv_dnn.NMSBoxes(detectedBoxes, confidencesPointer, confidenceThreshold, nonMaximumSuppressionThreshold, reducedIndices, 1.0f, 0);
         }

         for (int i = 0; i < reducedIndices.limit(); i++)
         {
            int index = reducedIndices.get(i);
            float[] maskWeights = new float[numberOfMasks];
            for (int j = 0; j < numberOfMasks; j++)
            {
               maskWeights[j] = detectedMaskWeights.get(((long) numberOfMasks * index) + j);
            }
            new YOLOv8DetectionOutput(detectionClassNames.get(detectedClassIds.get(index)),
                                                     detectedConfidences.get(index),
                                                     detectedBoxes.get(index).x() + shiftWidth,
                                                     detectedBoxes.get(index).y() + shiftHeight,
                                                     detectedBoxes.get(index).width(),
                                                     detectedBoxes.get(index).height(),
                                                     maskWeights);
         }

         confidencesPointer.close();
         reducedIndices.close();
      }
   }

   private RawImage computeDetectionMask(RawImage detectionImage,
                                         MatVector outputBlobs,
                                         float[] maskWeights,
                                         int boundingBoxX,
                                         int boundingBoxY,
                                         int boundingBoxWidth,
                                         int boundingBoxHeight,
                                         float maskThreshold)
   {
      FloatIndexer output1Indexer = outputBlobs.get(1).createIndexer();
      int numberOfMasks = (int) output1Indexer.size(1);
      int maskHeight = (int) output1Indexer.size(2);
      int maskWidth = (int) output1Indexer.size(3);
      output1Indexer.close();

      float xScaleFactor = (float) maskWidth / detectionImage.getWidth();
      float yScaleFactor = (float) maskHeight / detectionImage.getHeight();
      float maskFocalLengthX = xScaleFactor * detectionImage.getFocalLengthX();
      float maskFocalLengthY = yScaleFactor * detectionImage.getFocalLengthY();
      float maskPrincipalPointX = xScaleFactor * detectionImage.getPrincipalPointX();
      float maskPrincipalPointY = yScaleFactor * detectionImage.getPrincipalPointY();

      CameraIntrinsics maskIntrinsics = new CameraIntrinsics(maskHeight,
                                                             maskWidth,
                                                             maskFocalLengthX,
                                                             maskFocalLengthY,
                                                             maskPrincipalPointX,
                                                             maskPrincipalPointY);

      // Get float value mask
      Mat zeros = new Mat(maskHeight, maskWidth, opencv_core.CV_32F, new Scalar(0.0));
      MatExpr floatMask = new MatExpr(zeros);
      for (int i = 0; i < numberOfMasks; ++i)
      {
         Mat mask = outputBlobs.get(1).col(i).reshape(1, maskHeight);
         MatExpr weightMultipliedMask = opencv_core.multiply(mask, maskWeights[i]);
         floatMask = opencv_core.add(weightMultipliedMask, floatMask);

         mask.close();
         weightMultipliedMask.close();
      }

      // Apply threshold to get binary mask
      Mat binaryMask = new Mat(floatMask.size(), opencv_core.CV_8UC1);
      opencv_imgproc.threshold(floatMask.asMat(), binaryMask, maskThreshold, 255.0, opencv_imgproc.THRESH_BINARY);

      Mat boundingBoxMask = new Mat(maskHeight, maskWidth, opencv_core.CV_8UC1, new Scalar(0.0));
      opencv_imgproc.rectangle(boundingBoxMask,
                               new Rect(boundingBoxX / 4, boundingBoxY / 4, boundingBoxWidth / 4 + 2, boundingBoxHeight / 4 + 2),
                               new Scalar(255.0), opencv_imgproc.FILLED, opencv_imgproc.LINE_8, 0);

      opencv_core.bitwise_and(binaryMask, boundingBoxMask, binaryMask);
      boundingBoxMask.close();

      return new RawImage(binaryMask,
                          null,
                          PixelFormat.GRAY8,
                          maskIntrinsics,
                          CameraModel.PINHOLE,
                          detectionImage.getPose(),
                          detectionImage.getAcquisitionTime(),
                          detectionImage.getSequenceNumber(),
                          detectionImage.getDepthDiscretization());
   }

   public byte[] readONNXFile()
   {
      try
      {
         return Files.readAllBytes(onnxFile);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}
