package us.ihmc.perception.neural;

import ai.onnxruntime.*;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.log.LogTools;

import java.nio.FloatBuffer;
import java.util.HashMap;
import java.util.Map;

public class FootstepPredictor
{
   private String modelFilePath = "/home/quantum/Workspace/Code/IHMC/repository-group/ihmc-open-robotics-software/ihmc-perception/python/plogs/";
   private String onnxFileName = "footstep_predictor.onnx";

   public FootstepPredictor()
   {
      create();
   }

   public void create()
   {
      int LINEAR_INPUT_SIZE = 14;
      int imageHeight = 201;
      int imageWidth = 201;

      Mat inputImage = new Mat(imageHeight, imageWidth, opencv_core.CV_32FC1);

      try
      {
         OrtEnvironment environment = OrtEnvironment.getEnvironment();
         OrtSession.SessionOptions sessionOptions = new OrtSession.SessionOptions();
         OrtSession session = environment.createSession(modelFilePath + onnxFileName, sessionOptions);

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

         // set the image to be in the first input
         String inputName = (String) session.getInputNames().toArray()[0];
         long[] tensorInputShape = {1, 1, imageHeight, imageWidth};
         FloatBuffer data = inputImage.getByteBuffer().asFloatBuffer();

         // set a 1x200 vector to be second input
         String inputName2 = (String) session.getInputNames().toArray()[1];
         long[] tensorInputShape2 = {1, LINEAR_INPUT_SIZE};
         float[] data2 = new float[LINEAR_INPUT_SIZE];
         FloatBuffer data2Buffer = FloatBuffer.wrap(data2);

         // create a map
         Map<String, OnnxTensor> inputs = new HashMap<>();
         inputs.put(inputName, OnnxTensor.createTensor(environment, data, tensorInputShape));
         inputs.put(inputName2, OnnxTensor.createTensor(environment, data2Buffer, tensorInputShape2));

         // Measure start time
         long startTime = System.nanoTime();

         OrtSession.Result output = session.run(inputs);

         // Measure end time
         long endTime = System.nanoTime();

         // Print time difference in ms
         LogTools.info("Inference time: {} ms", (endTime - startTime) / 1000000.0);

         LogTools.info("Size: {}", output.size());

         for (Map.Entry<String, OnnxValue> stringOnnxValueEntry : output)
         {
            LogTools.info("{}: {}", stringOnnxValueEntry.getKey(), stringOnnxValueEntry.getValue());
            OnnxTensor outputTensor = (OnnxTensor) stringOnnxValueEntry.getValue();
            float[][] outputArray = (float[][]) outputTensor.getValue();
            LogTools.info("Output: {}", outputTensor.getInfo());
         }

      }
      catch (Exception exception)

      {
         exception.printStackTrace();
      }

      //  YOLOv5ImagePanel.draw();
   }

   public static void main(String[] args)
   {
      FootstepPredictor footstepPredictor = new FootstepPredictor();
   }
}

