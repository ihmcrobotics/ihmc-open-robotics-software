package us.ihmc.perception.detections.yolo;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.tools.io.resources.ResourceTools;

import java.io.IOException;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.List;
import java.util.Objects;

public class YOLOv8ModelTest
{
   public static RawImage loadImage() throws IOException
   {
      URL imageURL = YOLOv8ModelTest.class.getResource("/yolo/yolo_test_image.png");
      Objects.requireNonNull(imageURL);

      Mat image = new Mat();
      ResourceTools.processAsPath(imageURL, path ->
      {
         byte[] pngBytes = Files.readAllBytes(path);
         Mat pngEncodedImage = new Mat(1, pngBytes.length, opencv_core.CV_8UC1, new BytePointer(pngBytes));
         opencv_imgcodecs.imdecode(pngEncodedImage, opencv_imgcodecs.IMREAD_UNCHANGED, image);
      });

      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics(600, 960, 365.0995, 364.9875, 484.3945, 289.454);
      return new RawImage(image, null, PixelFormat.BGRA8, cameraIntrinsics, CameraModel.PINHOLE, new RigidBodyTransform(), Instant.now(), 0, 0.0f);
   }

   public static void runDoorPanelDetection() throws IOException
   {
      LogTools.info("Loading model...");
      YOLOv8Model model = new YOLOv8Model(Objects.requireNonNull(YOLOv8ModelTest.class.getResource(
            "/yolo/multi_class_real_yolov8_train_2585_valid_27_20251020_165330/")));
      model.setConfidenceThresholds(0.9f);
      model.setMaskThresholds(0.0f);
      model.setNMSThreshold(0.2f);

      LogTools.info("Loading image...");
      RawImage image = loadImage();

      LogTools.info("Running model...");
      YOLOv8DetectionList result = model.run(image);

      // Should have detected the door and traffic barrier
      LogTools.info("Checking result...");
      assert result.size() == 2;

      // Make sure one door is detected
      List<YOLOv8Detection> doorDetections = result.stream().filter(detection -> detection.objectClass().equals("door_panel")).toList();
      assert doorDetections.size() == 1;

      // Make sure one traffic barrier is detected
      List<YOLOv8Detection> barrierDetections = result.stream().filter(detection -> detection.objectClass().equals("traffic_barrier")).toList();
      assert barrierDetections.size() == 1;

      LogTools.info("Cleaning up...");
      image.release();
      result.destroy();
      model.destroy();

      LogTools.info("Done!");
   }

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      runDoorPanelDetection();
   }
}
