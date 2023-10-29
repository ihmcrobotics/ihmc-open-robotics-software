package us.ihmc.perception.neural;

import ai.onnxruntime.*;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.ejml.data.FMatrixRMaj;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.tools.IHMCCommonPaths;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;

public class HeightMapAutoencoder
{
   private static final int IMAGE_HEIGHT = 201;
   private static final int IMAGE_WIDTH = 201;

   private String modelFilePath = "/home/quantum/Workspace/Code/IHMC/repository-group/ihmc-open-robotics-software/ihmc-perception/python/plogs/";
   private String onnxFileName = "height_map_autoencoder.onnx";

   private OrtEnvironment environment = OrtEnvironment.getEnvironment();
   private OrtSession.SessionOptions sessionOptions = new OrtSession.SessionOptions();
   private OrtSession session;

   public HeightMapAutoencoder()
   {
      try
      {
         session = environment.createSession(modelFilePath + onnxFileName, sessionOptions);

         for (Map.Entry<String, NodeInfo> stringNodeInfoEntry : session.getInputInfo().entrySet())
         {
            LogTools.info("{}: {}", stringNodeInfoEntry.getKey(), stringNodeInfoEntry.getValue());
         }
         for (Map.Entry<String, NodeInfo> stringNodeInfoEntry : session.getOutputInfo().entrySet())
         {
            LogTools.info("{}: {}", stringNodeInfoEntry.getKey(), stringNodeInfoEntry.getValue());
         }

         //print all input names and sizes
         for (Map.Entry<String, NodeInfo> entry : session.getInputInfo().entrySet())
         {
            String inputName = entry.getKey();
            NodeInfo nodeInfo = entry.getValue();
            LogTools.info("Input Name: {}", inputName);
            LogTools.info("Input Info: {}", nodeInfo.toString());
         }
      }
      catch (OrtException e)
      {
         throw new RuntimeException(e);
      }
   }

   public Mat denoiseHeightMap(Mat heightMap)
   {
      ArrayList<Point3D> footstepPositions = new ArrayList<>();
      Mat denoisedHeightMapImage = null;

      try
      {
         denoisedHeightMapImage = predict(heightMap);
      }
      catch (OrtException e)
      {
         throw new RuntimeException(e);
      }

      return denoisedHeightMapImage;
   }

   public Mat predict(Mat imageInput) throws OrtException
   {
      if (imageInput.rows() != IMAGE_HEIGHT || imageInput.cols() != IMAGE_WIDTH)
         throw new RuntimeException("Image height and width must be " + IMAGE_HEIGHT + " and " + IMAGE_WIDTH);

      LogTools.info("Image Input Size: {}x{}", imageInput.rows(), imageInput.cols());

      Mat heightMapImage = imageInput.clone();
      heightMapImage.convertTo(heightMapImage, opencv_core.CV_32FC1, 1.0 / 10000.0, 0);

      // set the image to be in the first input
      String inputName = (String) session.getInputNames().toArray()[0];
      long[] tensorInputShape = {1, 1, heightMapImage.rows(), heightMapImage.cols()};
      FloatBuffer data = heightMapImage.getByteBuffer().asFloatBuffer();

      // create a map to store the input tensors
      Map<String, OnnxTensor> inputs = new HashMap<>();
      inputs.put(inputName, OnnxTensor.createTensor(environment, data, tensorInputShape));

      // run the inference
      OrtSession.Result output = session.run(inputs);

      Mat outputImage = new Mat(IMAGE_HEIGHT, IMAGE_WIDTH, opencv_core.CV_32FC1);
      for (Map.Entry<String, OnnxValue> stringOnnxValueEntry : output)
      {
         LogTools.debug("{}: {}", stringOnnxValueEntry.getKey(), stringOnnxValueEntry.getValue());

         OnnxTensor outputTensor = (OnnxTensor) stringOnnxValueEntry.getValue();
         float[][][][] outputArray = (float[][][][]) outputTensor.getValue();

         LogTools.info("Output: {}", outputTensor.getInfo());

         if (outputArray[0][0].length != IMAGE_HEIGHT && outputArray[0][0][0].length != IMAGE_WIDTH)
         {
            throw new RuntimeException("Output size must be " + IMAGE_HEIGHT * IMAGE_WIDTH);
         }

         FloatBuffer floatBuffer = FloatBuffer.wrap(outputArray[0][0][0]);

         outputImage.getByteBuffer().asFloatBuffer().put(floatBuffer);

      }

      return outputImage;
   }

   public static void main(String[] args) throws OrtException
   {
      String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20231018_135001_PerceptionLog.hdf5").toString();
      ArrayList<Point3D> startPositions = new ArrayList<>();
      ArrayList<Point3D> goalPositions = new ArrayList<>();
      ArrayList<Point3D> sensorPositions = new ArrayList<>();

      HeightMapAutoencoder heightMapAutoencoder = new HeightMapAutoencoder();

      PerceptionDataLoader perceptionDataLoader = new PerceptionDataLoader();

      perceptionDataLoader.openLogFile(perceptionLogFile);

      perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.START_FOOTSTEP_POSITION, startPositions);
      perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.GOAL_FOOTSTEP_POSITION, goalPositions);
      perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.L515_SENSOR_POSITION, sensorPositions);

      BytePointer depthBytePointer = new BytePointer(1000000);
      Mat heightMapImage = new Mat(201, 201, opencv_core.CV_16UC1);
      Mat inputImage = new Mat(201, 201, opencv_core.CV_32FC1);
      Mat output = null;

      for (int i = 0; i < 100; i++)
      {
         perceptionDataLoader.loadCompressedDepth(PerceptionLoggerConstants.CROPPED_HEIGHT_MAP_NAME, i, depthBytePointer, heightMapImage);

         Mat heightMapForDisplay = heightMapImage.clone();
         opencv_core.convertScaleAbs(heightMapForDisplay, heightMapForDisplay, 255.0 / 65535.0, 0);
         Mat display = new Mat(heightMapForDisplay.rows(), heightMapForDisplay.cols(), opencv_core.CV_8UC3);
         opencv_imgproc.cvtColor(heightMapForDisplay, display, opencv_imgproc.COLOR_GRAY2RGB);

         heightMapImage.convertTo(inputImage, opencv_core.CV_32FC1, 1.0 / 10000.0, 0);

         startPositions.get(i).sub(sensorPositions.get(i));
         goalPositions.get(i).sub(sensorPositions.get(i));

         // Measure start time
         long startTime = System.nanoTime();

         output = heightMapAutoencoder.predict(inputImage);

         // Measure end time
         long endTime = System.nanoTime();

         // Print time difference in ms
         LogTools.info("Inference time: {} ms", (endTime - startTime) / 1000000.0);

         Mat outputImage = output.clone();
         opencv_core.convertScaleAbs(outputImage, outputImage, 1000000, 0);
         opencv_core.normalize(outputImage, outputImage, 0, 65535, opencv_core.NORM_MINMAX, opencv_core.CV_16UC1, new Mat());

         // plot and display the image using imshow, brighten the image
         PerceptionDebugTools.printMat("Denoised Height Map", output, 4);
         opencv_imgproc.resize(outputImage, outputImage, new org.bytedeco.opencv.opencv_core.Size(1000, 1000));
         imshow("Display", outputImage);
         int code = waitKeyEx(0);

         if (code == 'q')
         {
            break;
         }
      }
   }
}
