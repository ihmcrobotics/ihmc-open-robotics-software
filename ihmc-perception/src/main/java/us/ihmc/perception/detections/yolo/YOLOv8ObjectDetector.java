package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_dnn;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.RectVector;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_core.StringVector;
import org.bytedeco.opencv.opencv_dnn.Net;
import org.bytedeco.opencv.opencv_text.FloatVector;
import org.bytedeco.opencv.opencv_text.IntVector;
import us.ihmc.perception.RawImage;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.io.IOException;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

public class YOLOv8ObjectDetector
{
   private static final double SCALE_FACTOR = 1.0 / 255.0;
   private static final Size DETECTION_SIZE = new Size(1280, 736);

   private YOLOv8Model yoloModel;

   private final Net yoloNet;
   private final StringVector outputNames; // literally list of "output0", "output1", "output2"...

   private final AtomicBoolean isReady = new AtomicBoolean(true);

   // TODO: Remove
//   @Deprecated
//   public YOLOv8ObjectDetector(String onnxFileName)
//   {
//      WorkspaceResourceDirectory directory = new WorkspaceResourceDirectory(getClass(), "/yolo/");
//      WorkspaceResourceFile onnxFile = new WorkspaceResourceFile(directory, onnxFileName);
//
//      if (onnxFile.getClasspathResource() == null)
//         throw new NullPointerException("YOLOv8 ONNX file could not be found");
//
//      try
//      {
//         yoloNet = opencv_dnn.readNetFromONNX(onnxFile.getClasspathResourceAsStream().readAllBytes());
//      }
//      catch (IOException e)
//      {
//         throw new RuntimeException(e);
//      }
//      if (opencv_core.getCudaEnabledDeviceCount() > 0)
//      {
//         yoloNet.setPreferableBackend(opencv_dnn.DNN_BACKEND_CUDA);
//         yoloNet.setPreferableTarget(opencv_dnn.DNN_TARGET_CUDA);
//      }
//      else
//      {
//         yoloNet.setPreferableBackend(opencv_dnn.DNN_BACKEND_OPENCV);
//         yoloNet.setPreferableTarget(opencv_dnn.DNN_TARGET_CPU);
//      }
//
//      outputNames = yoloNet.getUnconnectedOutLayersNames();
//   }

   public YOLOv8ObjectDetector(YOLOv8Model yoloModel)
   {
      yoloNet = opencv_dnn.readNetFromONNX(yoloModel.readONNXFile());

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

      this.yoloModel = yoloModel;
   }

   public YOLOv8DetectionResults runOnImage(RawImage bgrImage, float confidenceThreshold, float nonMaximumSuppressionThreshold, float maskThreshold)
   {
      YOLOv8DetectionResults results = null;

      try
      {
         bgrImage.get();
         isReady.set(false);
         Mat blob = opencv_dnn.blobFromImage(bgrImage.getCpuImageMat(), SCALE_FACTOR, DETECTION_SIZE, new Scalar(), true, true, opencv_core.CV_32F);
         MatVector outputBlobs = new MatVector(outputNames.size());

         yoloNet.setInput(blob);
         yoloNet.forward(outputBlobs, outputNames);
         isReady.set(true);

         Set<YOLOv8DetectionOutput> detections = processOutput(outputBlobs,
                                                               confidenceThreshold,
                                                               nonMaximumSuppressionThreshold,
                                                               bgrImage.getImageWidth(),
                                                               bgrImage.getImageHeight());
         results = new YOLOv8DetectionResults(detections, outputBlobs, bgrImage, maskThreshold);

         blob.release();
         bgrImage.release();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      return results;
   }

   public boolean isReady()
   {
      return isReady.get();
   }

   public void destroy()
   {
      System.out.println("Destroying " + getClass().getSimpleName());
      DETECTION_SIZE.close();
      yoloNet.close();
      System.out.println("Destroyed " + getClass().getSimpleName());
   }

   private Set<YOLOv8DetectionOutput> processOutput(MatVector outputBlobs, float confidenceThreshold, float nonMaximumSuppressionThreshold, int imageWidth, int imageHeight)
   {
      int shiftWidth = (imageWidth - DETECTION_SIZE.width()) / 2;
      int shiftHeight = (imageHeight - DETECTION_SIZE.height()) / 2;
      int numberOfMasks = outputBlobs.get(1).size(1);

      Set<YOLOv8DetectionOutput> detections = new HashSet<>();

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
            for (long j = 0; j < yoloModel.getDetectionClassNames().size(); j++)
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
                  detectedMaskWeights.push_back(output0Indexer.get(0, yoloModel.getDetectionClassNames().size() + 4 + k, i));
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
            String className = yoloModel.getObjectClassFromIndex(detectedClassIds.get(index));
            if (className.toLowerCase().contains("door_push_bar"))
            {
               if (detectedConfidences.get(index) > 0.9)
               {
                  detections.add(new YOLOv8DetectionOutput(yoloModel.getObjectClassFromIndex(detectedClassIds.get(index)),
                                                           detectedConfidences.get(index),
                                                           detectedBoxes.get(index).x() + shiftWidth,
                                                           detectedBoxes.get(index).y() + shiftHeight,
                                                           detectedBoxes.get(index).width(),
                                                           detectedBoxes.get(index).height(),
                                                           maskWeights));
               }
            }
            else
            {
               detections.add(new YOLOv8DetectionOutput(yoloModel.getObjectClassFromIndex(detectedClassIds.get(index)),
                                                        detectedConfidences.get(index),
                                                        detectedBoxes.get(index).x() + shiftWidth,
                                                        detectedBoxes.get(index).y() + shiftHeight,
                                                        detectedBoxes.get(index).width(),
                                                        detectedBoxes.get(index).height(),
                                                        maskWeights));
            }
         }

         confidencesPointer.close();
         reducedIndices.close();
      }

      return detections;
   }
}