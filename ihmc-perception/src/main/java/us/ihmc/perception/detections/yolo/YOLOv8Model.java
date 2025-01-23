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
import us.ihmc.tools.Destroyable;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class YOLOv8Model implements Destroyable
{
   private static final double SCALE_FACTOR = 1.0 / 255.0;
   private static final Size DETECTION_SIZE = new Size(1280, 736);

   // Meta data
   private final String modelName;
   private final List<String> detectionClassNames = new ArrayList<>();

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

      try (InputStream inputStream = new FileInputStream(YOLOv8Tools.getClassNamesFile(modelBaseDirectory).toFile()))
      {
         // Parse class_names.yaml
         Yaml yaml = new Yaml();
         Map<String, List<String>> classNamesData = yaml.load(inputStream);
         List<String> names = classNamesData.get("names");
         detectionClassNames.addAll(names);

         // Read the YOLO net
         Path onnxFile = YOLOv8Tools.getONNXFile(modelBaseDirectory);
         yoloNet = opencv_dnn.readNetFromONNX(Files.readAllBytes(onnxFile));
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

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

   @Override
   public void destroy()
   {
      yoloNet.close();
      outputNames.close();
   }

   public String getName()
   {
      return modelName;
   }

   public List<String> getDetectionClassNames()
   {
      return detectionClassNames;
   }

   public YOLOv8DetectionList run(RawImage image, float confidenceThreshold, float nmsThreshold, float maskThreshold)
   {
      YOLOv8DetectionList result = new YOLOv8DetectionList();

      // Ensure image is in BGR format
      RawImage bgrInputImage;
      if (image.getPixelFormat() == PixelFormat.BGR8)
      {
         bgrInputImage = image.get();
         if (bgrInputImage == null)
            return result;
      }
      else
      {
         Mat bgrMat = new Mat();
         image.getPixelFormat().convertToPixelFormat(image.getCpuImageMat(), bgrMat, PixelFormat.BGR8);
         bgrInputImage = image.replaceImage(bgrMat, PixelFormat.BGR8);
      }

      try (MatVector outputBlobs = new MatVector();
           IntVector classIDs = new IntVector();
           FloatVector confidences = new FloatVector();
           FloatVector maskWeights = new FloatVector();
           RectVector boundingBoxes = new RectVector();
           IntPointer reducedIndices = new IntPointer())
      {
         synchronized (yoloNet)
         {
            // Run the net
            Mat blob = opencv_dnn.blobFromImage(bgrInputImage.getCpuImageMat(), SCALE_FACTOR, DETECTION_SIZE, new Scalar(), true, true, opencv_core.CV_32F);
            yoloNet.setInput(blob);
            yoloNet.forward(outputBlobs, outputNames);
            blob.close();
         }

         // Get some useful stuff
         CameraIntrinsics maskIntrinsics = computeMaskIntrinsics(outputBlobs, bgrInputImage);
         int shiftWidth = (bgrInputImage.getWidth() - DETECTION_SIZE.width()) / 2;
         int shiftHeight = (bgrInputImage.getHeight() - DETECTION_SIZE.height()) / 2;

         // Get class ids, confidences, mask weights, bounding boxes from YOLO output
         processOutput(outputBlobs, confidenceThreshold, classIDs, confidences, maskWeights, boundingBoxes);

         // Ensure we have detections
         if (boundingBoxes.empty())
            return result;

         // Apply non-maximum suppression
         FloatPointer confidenceArray = new FloatPointer(confidences.get());
         opencv_dnn.NMSBoxes(boundingBoxes, confidenceArray, confidenceThreshold, nmsThreshold, reducedIndices, 1.0f, 0);
         confidenceArray.close();

         // Create YOLOv8Detections from the data
         int numberOfMasks = outputBlobs.get(1).size(1);
         for (int i = 0; i < reducedIndices.limit(); i++)
         {
            int index = reducedIndices.get(i);
            float[] weights = new float[numberOfMasks];
            for (int j = 0; j < numberOfMasks; j++)
            {
               weights[j] = maskWeights.get(((long) numberOfMasks * index) + j);
            }

            Rect boundingBox = boundingBoxes.get(index);
            Rect shiftedBox = new Rect(boundingBox.x() + shiftWidth, boundingBox.y() + shiftHeight, boundingBox.width(), boundingBox.height());
            RawImage mask = computeDetectionMask(outputBlobs, weights, shiftedBox, maskThreshold, maskIntrinsics, bgrInputImage);
            YOLOv8Detection detection = new YOLOv8Detection(detectionClassNames.get(classIDs.get(index)), confidences.get(index), shiftedBox, mask);
            result.add(detection);
         }
      }

      bgrInputImage.release();

      return result;
   }

   private void processOutput(MatVector outputBlobs,
                              float confidenceThreshold,
                              IntVector classIDs,
                              FloatVector confidences,
                              FloatVector maskWeights,
                              RectVector boundingBoxes)
   {
      int numberOfMasks = outputBlobs.get(1).size(1);

      FloatIndexer output0Indexer = outputBlobs.get(0).createIndexer();
      long detectionCount = output0Indexer.size(2);
      for (long i = 0; i < detectionCount; i++)
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
         if (maxConfidence < confidenceThreshold)
            continue;

         // Get the detection data
         int centerX = (int) (output0Indexer.get(0, 0, i));
         int centerY = (int) (output0Indexer.get(0, 1, i));
         int width = (int) (output0Indexer.get(0, 2, i));
         int height = (int) (output0Indexer.get(0, 3, i));
         int left = centerX - width / 2;
         int top = centerY - height / 2;

         classIDs.push_back((int) maxConfidenceClass);
         confidences.push_back(maxConfidence);
         boundingBoxes.push_back(new Rect(left, top, width, height));
         for (long k = 0; k < numberOfMasks; k++)
         {
            maskWeights.push_back(output0Indexer.get(0, detectionClassNames.size() + 4 + k, i));
         }
      }
      output0Indexer.close();
   }

   private CameraIntrinsics computeMaskIntrinsics(MatVector outputBlobs, RawImage bgrInputImage)
   {
      int maskHeight = outputBlobs.get(1).size(2);
      int maskWidth = outputBlobs.get(1).size(3);

      float xScaleFactor = (float) maskWidth / bgrInputImage.getWidth();
      float yScaleFactor = (float) maskHeight / bgrInputImage.getHeight();
      float maskFocalLengthX = xScaleFactor * bgrInputImage.getFocalLengthX();
      float maskFocalLengthY = yScaleFactor * bgrInputImage.getFocalLengthY();
      float maskPrincipalPointX = xScaleFactor * bgrInputImage.getPrincipalPointX();
      float maskPrincipalPointY = yScaleFactor * bgrInputImage.getPrincipalPointY();

      return new CameraIntrinsics(maskHeight, maskWidth, maskFocalLengthX, maskFocalLengthY, maskPrincipalPointX, maskPrincipalPointY);
   }

   private RawImage computeDetectionMask(MatVector outputBlobs, float[] maskWeights, Rect boundingBox, float maskThreshold, CameraIntrinsics maskIntrinsics, RawImage bgrInputImage)
   {
      // Get float value mask
      Mat zeros = new Mat(maskIntrinsics.getHeight(), maskIntrinsics.getWidth(), opencv_core.CV_32F, new Scalar(0.0));
      MatExpr floatMask = new MatExpr(zeros);
      for (int i = 0; i < maskWeights.length; ++i)
      {
         Mat mask = outputBlobs.get(1).col(i).reshape(1, maskIntrinsics.getHeight());
         MatExpr weightMultipliedMask = opencv_core.multiply(mask, maskWeights[i]);
         floatMask = opencv_core.add(weightMultipliedMask, floatMask);

         mask.close();
         weightMultipliedMask.close();
      }
      zeros.close();

      // Apply threshold to get binary mask
      Mat binaryMask = new Mat();
      opencv_imgproc.threshold(floatMask.asMat(), binaryMask, maskThreshold, 255.0, opencv_imgproc.THRESH_BINARY);
      binaryMask.convertTo(binaryMask, opencv_core.CV_8UC1);

      // Remove other objects from image using bounding box
      Mat boundingBoxMask = new Mat(maskIntrinsics.getHeight(), maskIntrinsics.getWidth(), opencv_core.CV_8UC1, new Scalar(0.0));
      opencv_imgproc.rectangle(boundingBoxMask,
                               new Rect(boundingBox.x() / 4, boundingBox.y() / 4, boundingBox.width() / 4 + 2, boundingBox.height() / 4 + 2),
                               new Scalar(255.0), opencv_imgproc.FILLED, opencv_imgproc.LINE_8, 0);

      opencv_core.bitwise_and(binaryMask, boundingBoxMask, binaryMask);

      boundingBoxMask.close();
      floatMask.close();

      return new RawImage(binaryMask,
                          null,
                          PixelFormat.GRAY8,
                          new CameraIntrinsics(maskIntrinsics),
                          CameraModel.PINHOLE,
                          bgrInputImage.getPose(),
                          bgrInputImage.getAcquisitionTime(),
                          bgrInputImage.getSequenceNumber(),
                          bgrInputImage.getDepthDiscretization());
   }
}
