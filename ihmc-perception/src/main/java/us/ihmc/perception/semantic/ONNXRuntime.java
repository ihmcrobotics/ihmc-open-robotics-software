package us.ihmc.perception.semantic;

import ai.onnxruntime.*;
import java.nio.FloatBuffer;
import java.util.Collections;
import java.util.Map;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.tools.IHMCCommonPaths;

public class ONNXRuntime {
  String[] labels = {"person",        "bicycle",      "car",
                     "motorcycle",    "airplane",     "bus",
                     "train",         "truck",        "boat",
                     "traffic light", "fire hydrant", "stop sign",
                     "parking meter", "bench",        "bird",
                     "cat",           "dog",          "horse",
                     "sheep",         "cow",          "elephant",
                     "bear",          "zebra",        "giraffe",
                     "backpack",      "umbrella",     "handbag",
                     "tie",           "suitcase",     "frisbee",
                     "skis",          "snowboard",    "sports ball",
                     "kite",          "baseball bat", "baseball glove",
                     "skateboard",    "surfboard",    "tennis racket",
                     "bottle",        "wine glass",   "cup",
                     "fork",          "knife",        "spoon",
                     "bowl",          "banana",       "apple",
                     "sandwich",      "orange",       "broccoli",
                     "carrot",        "hot dog",      "pizza",
                     "donut",         "cake",         "chair",
                     "couch",         "potted plant", "bed",
                     "dining table",  "toilet",       "tv",
                     "laptop",        "mouse",        "remote",
                     "keyboard",      "cell phone",   "microwave",
                     "oven",          "toaster",      "sink",
                     "refrigerator",  "book",         "clock",
                     "vase",          "scissors",     "teddy bear",
                     "hair drier",    "toothbrush"};

  public ONNXRuntime() {
    try {

      Mat bgrInputImage =
          opencv_imgcodecs.imread(IHMCCommonPaths.DOT_IHMC_DIRECTORY_NAME);

      Mat rgbInputImage = bgrInputImage;
      opencv_imgproc.cvtColor(bgrInputImage, rgbInputImage,
                              opencv_imgproc.COLOR_BGR2RGB);

      BytedecoImage float32Image =
          new BytedecoImage(640, 640, opencv_core.CV_32FC3);
      double delta = 0.0;               // no delta added
      double scaleFactor = 1.0 / 255.0; // scale from 0-255 to 0.0-1.0
      rgbInputImage.convertTo(float32Image.getBytedecoOpenCVMat(),
                              opencv_core.CV_32FC3, scaleFactor, delta);

      OrtEnvironment environment = OrtEnvironment.getEnvironment();
      OrtSession.SessionOptions sessionOptions =
          new OrtSession.SessionOptions();
      OrtSession session = environment.createSession(
          IHMCCommonPaths.DOT_IHMC_DIRECTORY.resolve("yolov8s.onnx").toString(),
          sessionOptions);

      for (Map.Entry<String, NodeInfo> stringNodeInfoEntry :
           session.getInputInfo().entrySet()) {
        LogTools.info("{}: {}", stringNodeInfoEntry.getKey(),
                      stringNodeInfoEntry.getValue());
      }
      for (Map.Entry<String, NodeInfo> stringNodeInfoEntry :
           session.getOutputInfo().entrySet()) {
        LogTools.info("{}: {}", stringNodeInfoEntry.getKey(),
                      stringNodeInfoEntry.getValue());
      }

      String inputName = session.getInputNames().iterator().next();
      long[] tensorInputShape = {1, 3, 640, 640};
      float32Image.rewind();
      FloatBuffer data =
          float32Image.getBackingDirectByteBuffer().asFloatBuffer();

      OnnxTensor testTensor =
          OnnxTensor.createTensor(environment, data, tensorInputShape);

      OrtSession.Result output =
          session.run(Collections.singletonMap(inputName, testTensor));

      LogTools.info("Size: {}", output.size());

      for (Map.Entry<String, OnnxValue> stringOnnxValueEntry : output) {
        LogTools.info("{}: {}", stringOnnxValueEntry.getKey(),
                      stringOnnxValueEntry.getValue());
      }

      float[][] outputArray = ((float[][][])output.get(0).getValue())[0];
      LogTools.info("Name: {}", outputArray[0].toString());

      // See what classes have a high confidence
      for (int c = 0; c < 25200; c++) {
        if (outputArray[c][4] > 0.13) {
          int largestIndex = 5;
          for (int i = 5; i < 85; i++) {
            largestIndex = (outputArray[c][i] > outputArray[c][largestIndex])
                               ? i
                               : largestIndex;
          }
          LogTools.info("c=:" + c + " and label=" + labels[largestIndex - 5]);

          // Point topLeft = new Point((int) outputArray[c][0], (int)
          // outputArray[c][1]); Point bottomRight = new Point((int)
          // outputArray[c][2], (int) outputArray[c][3]);

          Point topLeft =
              new Point((int)(outputArray[c][0] - outputArray[c][2] / 2),
                        (int)(outputArray[c][1] + outputArray[c][3] / 2));
          Point bottomRight =
              new Point((int)(outputArray[c][0] + outputArray[c][2] / 2),
                        (int)(outputArray[c][1] - outputArray[c][3] / 2));

          // Point topLeft = new Point((int) (outputArray[c][0]), (int)
          // (outputArray[c][1])); Point bottomRight =new Point((int)
          // (outputArray[c][0] + outputArray[c][2]/2), (int) (outputArray[c][1]
          // + outputArray[c][3]/2));

          opencv_imgproc.rectangle(rgbInputImage, topLeft, bottomRight,
                                   new Scalar(255, 0, 0, 255));

          Point centerOfCircle =
              new Point((int)outputArray[c][0], (int)outputArray[c][1]);

          opencv_imgproc.putText(rgbInputImage, labels[largestIndex - 5],
                                 centerOfCircle,
                                 opencv_imgproc.CV_FONT_HERSHEY_PLAIN, 5.0,
                                 new Scalar(255, 255, 0, 255));

          opencv_imgproc.circle(rgbInputImage, centerOfCircle, 5,
                                new Scalar(0, 255, 0, 255));
        }
      }

    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  public static void main(String[] args) { new ONNXRuntime(); }
}
