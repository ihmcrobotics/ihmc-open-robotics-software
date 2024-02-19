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

public class FootstepPredictor
{
   private static final int LINEAR_INPUT_SIZE = 6;
   private static final int LINEAR_OUTPUT_SIZE = 12;
   private static final int NUMBER_OF_FOOTSTEPS = 10;
   private static final int FOOTSTEP_VECTOR_SIZE = 3;

   private static final int IMAGE_INPUT_HEIGHT = 201;
   private static final int IMAGE_INPUT_WIDTH = 201;

   private String onnxFilePath = IHMCCommonPaths.USER_HOME_DIRECTORY.resolve("Downloads/Model_Weights/footstep_predictor_4.onnx").toString();

   private OrtEnvironment environment = OrtEnvironment.getEnvironment();
   private OrtSession.SessionOptions sessionOptions = new OrtSession.SessionOptions();
   private OrtSession session;

   public FootstepPredictor()
   {
      try
      {
         session = environment.createSession(onnxFilePath, sessionOptions);

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

   public ArrayList<Point3D> generateFootsteps(Mat heightMap, Point2D startPosition, Point2D goalPosition, Point2D sensorPosition)
   {
      ArrayList<Point3D> footstepPositions = new ArrayList<>();
      FMatrixRMaj linearInput = new FMatrixRMaj(LINEAR_INPUT_SIZE, 1);
      linearInput.set(0, 0, startPosition.getX32());
      linearInput.set(1, 0, startPosition.getY32());
      linearInput.set(2, 0, 0);
      linearInput.set(3, 0, goalPosition.getX32());
      linearInput.set(4, 0, goalPosition.getY32());
      linearInput.set(5, 0, 0);

      FMatrixRMaj linearOutput = null;

      try
      {
         linearOutput = predict(heightMap, linearInput);
      }
      catch (OrtException e)
      {
         throw new RuntimeException(e);
      }

      if (linearOutput != null)
      {
         for (int i = 0; i < LINEAR_OUTPUT_SIZE / FOOTSTEP_VECTOR_SIZE; i++)
         {
            Point3D point = new Point3D(linearOutput.get(FOOTSTEP_VECTOR_SIZE * i, 0) + sensorPosition.getX32(),
                                        linearOutput.get(FOOTSTEP_VECTOR_SIZE * i + 1, 0) + sensorPosition.getY32(),
                                        1.0);
            footstepPositions.add(point);
         }
      }

      return footstepPositions;
   }

   public FMatrixRMaj predict(Mat imageInput, FMatrixRMaj linearInput) throws OrtException
   {
      if (imageInput.rows() != IMAGE_INPUT_HEIGHT || imageInput.cols() != IMAGE_INPUT_WIDTH)
         throw new RuntimeException("Image height and width must be " + IMAGE_INPUT_HEIGHT + " and " + IMAGE_INPUT_WIDTH);

      if (linearInput.getNumRows() != LINEAR_INPUT_SIZE || linearInput.getNumCols() != 1)
         throw new RuntimeException("Linear input size must be " + LINEAR_INPUT_SIZE + "x1");

      LogTools.info("Image Input Size: {}x{}", imageInput.rows(), imageInput.cols());
      LogTools.info("Linear Input Size: {}x{}", linearInput.getNumRows(), linearInput.getNumCols());

      Mat heightMapImage = imageInput.clone();
      heightMapImage.convertTo(heightMapImage, opencv_core.CV_32FC1, 1.0 / 10000.0, 0);

      // set the image to be in the first input
      String inputName = (String) session.getInputNames().toArray()[0];
      long[] tensorInputShape = {1, 1, heightMapImage.rows(), heightMapImage.cols()};
      FloatBuffer data = heightMapImage.getByteBuffer().asFloatBuffer();

      // set the linear input to be in the second input (extract from linearInput matrix)
      String inputName2 = (String) session.getInputNames().toArray()[1];
      long[] tensorInputShape2 = {1, LINEAR_INPUT_SIZE};
      float[] data2 = linearInput.getData();
      FloatBuffer data2Buffer = FloatBuffer.wrap(data2);

      // create a map to store the input tensors
      Map<String, OnnxTensor> inputs = new HashMap<>();
      inputs.put(inputName, OnnxTensor.createTensor(environment, data, tensorInputShape));
      inputs.put(inputName2, OnnxTensor.createTensor(environment, data2Buffer, tensorInputShape2));

      // run the inference
      OrtSession.Result output = session.run(inputs);

      FMatrixRMaj outputMatrix = new FMatrixRMaj(LINEAR_OUTPUT_SIZE, 1);
      for (Map.Entry<String, OnnxValue> stringOnnxValueEntry : output)
      {
         LogTools.debug("{}: {}", stringOnnxValueEntry.getKey(), stringOnnxValueEntry.getValue());

         OnnxTensor outputTensor = (OnnxTensor) stringOnnxValueEntry.getValue();
         float[][] outputArray = (float[][]) outputTensor.getValue();

         if (outputArray[0].length != LINEAR_OUTPUT_SIZE)
            throw new RuntimeException("Linear output size must be " + LINEAR_OUTPUT_SIZE + "x1" + " but was " + outputArray[0].length);

         outputMatrix.setData(outputArray[0]);

         LogTools.info("Output: {}", outputTensor.getInfo());
      }

      return outputMatrix;
   }

   public static void main(String[] args) throws OrtException
   {
      String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20231018_135001_PerceptionLog.hdf5").toString();
      ArrayList<Point3D> startPositions = new ArrayList<>();
      ArrayList<Point3D> goalPositions = new ArrayList<>();
      ArrayList<Point3D> sensorPositions = new ArrayList<>();

      FootstepPredictor footstepPredictor = new FootstepPredictor();

      PerceptionDataLoader perceptionDataLoader = new PerceptionDataLoader();

      perceptionDataLoader.openLogFile(perceptionLogFile);

      perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.START_FOOTSTEP_POSITION, startPositions);
      perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.GOAL_FOOTSTEP_POSITION, goalPositions);
      perceptionDataLoader.loadPoint3DList(PerceptionLoggerConstants.L515_SENSOR_POSITION, sensorPositions);

      BytePointer depthBytePointer = new BytePointer(1000000);
      Mat heightMapImage = new Mat(201, 201, opencv_core.CV_16UC1);
      Mat inputImage = new Mat(201, 201, opencv_core.CV_32FC1);
      FMatrixRMaj linearInput = new FMatrixRMaj(LINEAR_INPUT_SIZE, 1);
      FMatrixRMaj output = null;

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

         linearInput.set(0, 0, startPositions.get(i).getX32());
         linearInput.set(1, 0, startPositions.get(i).getY32());
         linearInput.set(2, 0, goalPositions.get(i).getX32());
         linearInput.set(3, 0, goalPositions.get(i).getY32());

         // Measure start time
         long startTime = System.nanoTime();

         output = footstepPredictor.predict(inputImage, linearInput);

         // Measure end time
         long endTime = System.nanoTime();

         // Print time difference in ms
         LogTools.info("Inference time: {} ms", (endTime - startTime) / 1000000.0);

         // plot and display the image using imshow, brighten the image
         PerceptionDebugTools.plotRectangle(display, new Point2D(startPositions.get(i)), 4, new Scalar(0, 0, 0, 255));
         PerceptionDebugTools.plotRectangle(display, new Point2D(goalPositions.get(i)), 4, new Scalar(255, 255, 255, 255));
         PerceptionDebugTools.plotFootsteps(display, output, 2);
         opencv_imgproc.resize(display, display, new org.bytedeco.opencv.opencv_core.Size(1000, 1000));
         imshow("Display", display);
         int code = waitKeyEx(0);

         if (code == 'q')
         {
            break;
         }
      }
   }
}

