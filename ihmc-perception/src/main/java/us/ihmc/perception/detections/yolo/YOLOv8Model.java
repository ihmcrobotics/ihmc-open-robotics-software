package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_dnn;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_core.StringVector;
import org.bytedeco.opencv.opencv_dnn.Net;
import org.yaml.snakeyaml.Yaml;
import us.ihmc.perception.RawImage;
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

   public void run(RawImage image, float confidenceThreshold, float nmsThreshold, float maskThreshold)
   {
      RawImage bgrImage;
      if (image.getPixelFormat() == PixelFormat.BGR8)
      {
         bgrImage = image.get();
         if (bgrImage == null)
            return;
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

      processOutput(outputBlobs, confidenceThreshold, nmsThreshold, bgrImage.getWidth(), bgrImage.getHeight());

      bgrImage.release();
   }

   private void processOutput(MatVector outputBlobs, float confidenceThreshold, float nonMaximumSuppressionThreshold, int imageWidth, int imageHeight)
   {
      // TODO
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
