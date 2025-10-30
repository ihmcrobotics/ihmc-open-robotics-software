package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.imageMessage.PixelFormat;

import java.net.URL;
import java.time.Instant;
import java.util.List;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

public class YOLOv8ModelTest
{
   private static RawImage DOOR_BARRIER_CHARGE_IMAGE;

   @BeforeAll
   public static void load()
   {
      URL imageURL = YOLOv8ModelTest.class.getResource("/yolo/yolo_test_image.png");
      Objects.requireNonNull(imageURL);
      Mat image = opencv_imgcodecs.imread(imageURL.getPath());
      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics(600, 960, 365.0995, 364.9875, 484.3945, 289.454);
      DOOR_BARRIER_CHARGE_IMAGE = new RawImage(image,
                                               null,
                                               PixelFormat.BGR8,
                                               cameraIntrinsics,
                                               CameraModel.PINHOLE,
                                               new RigidBodyTransform(),
                                               Instant.now(),
                                               0,
                                               0.0f);
   }

   @AfterAll
   public static void close()
   {
      DOOR_BARRIER_CHARGE_IMAGE.release();
   }

   @Test
   public void testDoorPanelDetection()
   {
      YOLOv8Model model = new YOLOv8Model(Objects.requireNonNull(getClass().getResource("/yolo/multi_class_real_yolov8_train_2585_valid_27_20251020_165330/")));
      model.setConfidenceThresholds(0.9f);
      model.setMaskThresholds(0.0f);
      model.setNMSThreshold(0.2f);

      YOLOv8DetectionList result = model.run(DOOR_BARRIER_CHARGE_IMAGE);

      // Should have detected the door and traffic barrier
      assertEquals(2, result.size());

      // Make sure one door is detected
      List<YOLOv8Detection> doorDetections = result.stream().filter(detection -> detection.objectClass().equals("door_panel")).toList();
      assertEquals(1, doorDetections.size());

      // Make sure one traffic barrier is detected
      List<YOLOv8Detection> barrierDetections = result.stream().filter(detection -> detection.objectClass().equals("traffic_barrier")).toList();
      assertEquals(1, barrierDetections.size());

      result.destroy();
      model.destroy();
   }
}
