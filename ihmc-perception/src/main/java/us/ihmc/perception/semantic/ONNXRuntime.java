package us.ihmc.perception.semantic;

import ai.onnxruntime.*;

import java.nio.FloatBuffer;
import java.util.Collections;
import java.util.Map;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.tools.IHMCCommonPaths;

public class ONNXRuntime
{
   private OrtEnvironment environment;
   private OrtSession.SessionOptions sessionOptions;
   private OrtSession session;

   private Mat rgbInputImage = new Mat();

   private float[][] outputArray;

   private String[] labels = Semantics.MS_COCO_LABELS;

   private int gpuDeviceId = 0;

   public ONNXRuntime(String weightsFile)
   {
      try
      {
         environment = OrtEnvironment.getEnvironment();

         sessionOptions = new OrtSession.SessionOptions();
         //sessionOptions.addCUDA(gpuDeviceId);

         session = environment.createSession(weightsFile, sessionOptions);

         for (Map.Entry<String, NodeInfo> stringNodeInfoEntry : session.getInputInfo().entrySet())
         {
            LogTools.info("{}: {}", stringNodeInfoEntry.getKey(), stringNodeInfoEntry.getValue());
         }
         for (Map.Entry<String, NodeInfo> stringNodeInfoEntry : session.getOutputInfo().entrySet())
         {
            LogTools.info("{}: {}", stringNodeInfoEntry.getKey(), stringNodeInfoEntry.getValue());
         }
      }
      catch (Exception exception)
      {
         exception.printStackTrace();
      }
   }

   public void detect(Mat bgrInputImage)
   {
      try
      {
         opencv_imgproc.cvtColor(bgrInputImage, rgbInputImage, opencv_imgproc.COLOR_BGR2RGB);

         BytedecoImage float32Image = new BytedecoImage(640, 640, opencv_core.CV_32FC3);
         double delta = 0.0;               // no delta added
         double scaleFactor = 1.0 / 255.0; // scale from 0-255 to 0.0-1.0
         rgbInputImage.convertTo(float32Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC3, scaleFactor, delta);

         String inputName = session.getInputNames().iterator().next();
         long[] tensorInputShape = {1, 3, 640, 640};

         float32Image.rewind();
         FloatBuffer data = float32Image.getBackingDirectByteBuffer().asFloatBuffer();

         OnnxTensor testTensor = OnnxTensor.createTensor(environment, data, tensorInputShape);

         OrtSession.Result output = session.run(Collections.singletonMap(inputName, testTensor));

         LogTools.info("Size: {}", output.size());

         for (Map.Entry<String, OnnxValue> stringOnnxValueEntry : output)
         {
            LogTools.info("{}: {}", stringOnnxValueEntry.getKey(), stringOnnxValueEntry.getValue());
         }

         outputArray = ((float[][][]) output.get(0).getValue())[0];
         LogTools.info("Name: {}", outputArray[0].toString());

         plotResults(bgrInputImage, outputArray);
      }
      catch (Exception exception)
      {
         exception.printStackTrace();
      }
   }

   public void plotResults(Mat image, float[][] outputArray)
   {
      // See what classes have a high confidence
      for (int c = 0; c < 8400; c++)
      {
         int largestIndex = 5;
         for (int i = 5; i < 84; i++)
         {
            largestIndex = (outputArray[i][c] > outputArray[largestIndex][c]) ? i : largestIndex;
         }

         LogTools.info("c=:" + c + " and label=" + labels[largestIndex - 5] + " and confidence=" + outputArray[largestIndex][c]);

         if (outputArray[largestIndex][c] > 0.1)
         {
            Point topLeft = new Point((int) (outputArray[0][c] - outputArray[2][c] / 2), (int) (outputArray[1][c] + outputArray[3][c] / 2));
            Point bottomRight = new Point((int) (outputArray[0][c] + outputArray[2][c] / 2), (int) (outputArray[1][c] - outputArray[3][c] / 2));

            opencv_imgproc.rectangle(image, topLeft, bottomRight, new Scalar(255, 0, 0, 255));
         }

      }
   }

   public static void main(String[] args)
   {
      new ONNXRuntime(IHMCCommonPaths.DOT_IHMC_DIRECTORY.resolve("yolov8s.onnx").toString());
   }
}
