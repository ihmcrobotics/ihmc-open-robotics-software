package us.ihmc.avatar.colorVision.stereo;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.zed.SL_CalibrationParameters;
import org.bytedeco.zed.SL_InitParameters;
import org.bytedeco.zed.SL_RuntimeParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensors.ZEDModelData;
import us.ihmc.tools.time.FrequencyStatisticPrinter;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;

import static org.bytedeco.zed.global.zed.*;

public class DualZEDUDPSender
{
   private static final String LEFT_DESTINATION_IP_ADDRESS = System.getProperty("zed.left.udp.dest.address", "127.0.0.1");
   private static final String RIGHT_DESTINATION_IP_ADDRESS = System.getProperty("zed.right.udp.dest.address", "127.0.0.1");
   public static final int LEFT_UDP_PORT = 9200;
   public static final int RIGHT_UDP_PORT = 9201;
   public static final int IPV4_HEADER_LENGTH = 28;
   public static final int DATAGRAM_MAX_LENGTH = (int) (Math.pow(2, 16) - 1) - IPV4_HEADER_LENGTH;

   private final SideDependentList<Thread> publishThreads = new SideDependentList<>();
   private volatile boolean running;

   public void start()
   {
      running = true;

      startZED();

      for (RobotSide side : RobotSide.values())
      {
         Thread publishThread = new Thread(() ->
         {
            DatagramSocket socket;
            try
            {
               socket = new DatagramSocket();
            }
            catch (SocketException e)
            {
               LogTools.error(e);
               return;
            }
            InetAddress address;
            try
            {
               address = InetAddress.getByName(side == RobotSide.LEFT ? LEFT_DESTINATION_IP_ADDRESS : RIGHT_DESTINATION_IP_ADDRESS);
            }
            catch (UnknownHostException e)
            {
               LogTools.error(e);
               return;
            }

            FrequencyStatisticPrinter frequencyStatisticPrinter = new FrequencyStatisticPrinter();

            int frameNumber = 0;

            Pointer colorImagePointer = new Pointer(sl_mat_create_new(imageWidth, imageHeight, SL_MAT_TYPE_U8_C4, SL_MEM_CPU));

            while (running)
            {
               checkError("sl_grab", sl_grab(cameraID, zedRuntimeParameters));

               int slViewSide = side == RobotSide.LEFT ? SL_VIEW_LEFT : SL_VIEW_RIGHT;
               checkError("sl_retrieve_image", sl_retrieve_image(cameraID, colorImagePointer, slViewSide, SL_MEM_CPU, imageWidth, imageHeight));

               Mat mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, sl_mat_get_ptr(colorImagePointer, SL_MEM_CPU), sl_mat_get_step_bytes(colorImagePointer, SL_MEM_CPU));

               int latestImageDataLength = (int) (mat.total() * mat.elemSize());

               byte[] imageData = new byte[latestImageDataLength];
               mat.ptr().get(imageData, 0, latestImageDataLength);

               int numberOfDatagramFragments = (int) Math.ceil((double) latestImageDataLength / DATAGRAM_MAX_LENGTH);

               for (int fragment = 0; fragment < numberOfDatagramFragments; fragment++)
               {
                  int fragmentHeaderLength = 1 + 4 + 4 + 4 + 4 + 4 + 4;
                  int maxFragmentDataLength = DATAGRAM_MAX_LENGTH - fragmentHeaderLength;
                  int fragmentDataOffset = fragment * maxFragmentDataLength;
                  int fragmentDataLength = Math.min(maxFragmentDataLength, latestImageDataLength - fragmentDataOffset);
                  int datagramLength = fragmentHeaderLength + fragmentDataLength;

                  byte[] datagramData = new byte[DATAGRAM_MAX_LENGTH];
                  ByteBuffer datagramBuffer = ByteBuffer.wrap(datagramData);

                  // Sanity check
                  if (datagramLength > DATAGRAM_MAX_LENGTH)
                     throw new RuntimeException("Too many bytes");

                  // Write header data
                  {
                     datagramBuffer.put(side.toByte());
                     datagramBuffer.putInt(imageWidth);
                     datagramBuffer.putInt(imageHeight);
                     datagramBuffer.putInt(latestImageDataLength);
                     datagramBuffer.putInt(frameNumber);
                     datagramBuffer.putInt(fragment);
                     datagramBuffer.putInt(fragmentDataLength);
                  }

                  // Write fragment data
                  {
                     for (int i = 0; i < fragmentDataLength; i++)
                     {
                        datagramBuffer.put(imageData[fragmentDataOffset + i]);
                     }
                  }

                  DatagramPacket packet = new DatagramPacket(datagramData, datagramLength, address, side == RobotSide.LEFT ? LEFT_UDP_PORT : RIGHT_UDP_PORT);
                  try
                  {
                     socket.send(packet);
                  }
                  catch (IOException e)
                  {
                     LogTools.error(e);
                  }
               }


               frameNumber++;

               frequencyStatisticPrinter.ping();
            }

            socket.close();
         });

         publishThreads.put(side, publishThread);

         publishThread.start();
      }
   }

   public void stop()
   {
      running = false;

      for (Thread publishThread : publishThreads)
      {
         try
         {
            publishThread.join();
         }
         catch (InterruptedException e)
         {
            LogTools.error(e);
         }
      }

      stopZED();
   }

   private static final int CAMERA_FPS = 30;

   private final int cameraID = 0;
   private ZEDModelData zedModelData;
   private SL_RuntimeParameters zedRuntimeParameters;

   private int imageWidth; // Width of rectified image in pixels (color image width == depth image width)
   private int imageHeight; // Height of rectified image in pixels (color image height ==  depth image height)
   private final SideDependentList<Float> cameraFocalLengthX = new SideDependentList<>();
   private final SideDependentList<Float> cameraFocalLengthY = new SideDependentList<>();
   private final SideDependentList<Float> cameraPrincipalPointX = new SideDependentList<>();
   private final SideDependentList<Float> cameraPrincipalPointY = new SideDependentList<>();

   private void stopZED()
   {
      System.out.println("Destroying " + getClass().getSimpleName());

      sl_close_camera(cameraID);
      System.out.println("Destroyed " + getClass().getSimpleName());
   }

   private boolean startZED()
   {
      boolean success;

      zedRuntimeParameters = new SL_RuntimeParameters();

      LogTools.info("ZED SDK version: " + sl_get_sdk_version().getString());

      if (sl_is_opened(cameraID))
         sl_close_camera(cameraID);

      LogTools.info("Starting ZED...");

      // Create and initialize the camera
      success = sl_create_camera(cameraID);

      SL_InitParameters zedInitializationParameters = new SL_InitParameters();

      // Open camera with default parameters to find model
      // Can't get the model number without opening the camera first
      success = checkError("sl_open_camera", sl_open_camera(cameraID, zedInitializationParameters, 0, "", "", 0, "", "", ""));
      if (!success)
         return success;
      setZEDConfiguration(cameraID);
      sl_close_camera(cameraID);

      // Set initialization parameters based on camera model
      zedInitializationParameters.camera_fps(CAMERA_FPS);
      zedInitializationParameters.resolution(SL_RESOLUTION_HD720);
      zedInitializationParameters.input_type(SL_INPUT_TYPE_USB);
      zedInitializationParameters.camera_device_id(cameraID);
      zedInitializationParameters.camera_image_flip(SL_FLIP_MODE_OFF);
      zedInitializationParameters.camera_disable_self_calib(false);
      zedInitializationParameters.enable_image_enhancement(true);
      zedInitializationParameters.svo_real_time_mode(true);
      zedInitializationParameters.depth_mode(SL_DEPTH_MODE_ULTRA);
      zedInitializationParameters.depth_stabilization(1);
      zedInitializationParameters.depth_maximum_distance(zedModelData.getMaximumDepthDistance());
      zedInitializationParameters.depth_minimum_distance(zedModelData.getMinimumDepthDistance());
      zedInitializationParameters.coordinate_unit(SL_UNIT_METER);
      zedInitializationParameters.coordinate_system(SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD);
      zedInitializationParameters.sdk_gpu_id(-1); // Will find and use the best available GPU
      zedInitializationParameters.sdk_verbose(0); // false
      zedInitializationParameters.sensors_required(true);
      zedInitializationParameters.enable_right_side_measure(false);
      zedInitializationParameters.open_timeout_sec(5.0f);
      zedInitializationParameters.async_grab_camera_recovery(false);

      // Reopen camera with specific parameters set
      success = checkError("sl_open_camera", sl_open_camera(cameraID, zedInitializationParameters, 0, "", "", 0, "", "", ""));
      if (!success)
         return success;

      zedRuntimeParameters.enable_depth(true);
      zedRuntimeParameters.confidence_threshold(100);
      zedRuntimeParameters.reference_frame(SL_REFERENCE_FRAME_CAMERA);
      zedRuntimeParameters.texture_confidence_threshold(100);
      zedRuntimeParameters.remove_saturated_areas(true);
      zedRuntimeParameters.enable_fill_mode(false);

      // Get camera's parameters
      SL_CalibrationParameters zedCalibrationParameters = sl_get_calibration_parameters(cameraID, false);
      cameraFocalLengthX.put(RobotSide.LEFT, zedCalibrationParameters.left_cam().fx());
      cameraFocalLengthX.put(RobotSide.RIGHT, zedCalibrationParameters.right_cam().fx());
      cameraFocalLengthY.put(RobotSide.LEFT, zedCalibrationParameters.left_cam().fy());
      cameraFocalLengthY.put(RobotSide.RIGHT, zedCalibrationParameters.right_cam().fy());
      cameraPrincipalPointX.put(RobotSide.LEFT, zedCalibrationParameters.left_cam().cx());
      cameraPrincipalPointX.put(RobotSide.RIGHT, zedCalibrationParameters.right_cam().cx());
      cameraPrincipalPointY.put(RobotSide.LEFT, zedCalibrationParameters.left_cam().cy());
      cameraPrincipalPointY.put(RobotSide.RIGHT, zedCalibrationParameters.right_cam().cy());

      imageWidth = sl_get_width(cameraID);
      imageHeight = sl_get_height(cameraID);

      LogTools.info("Started {} camera", getCameraModel(cameraID));
      LogTools.info("Firmware version: {}", sl_get_camera_firmware(cameraID));
      LogTools.info("Image resolution: {} x {}", imageWidth, imageHeight);

      return success;
   }

   private boolean checkError(String functionName, int returnedState)
   {
      if (returnedState != SL_ERROR_CODE_SUCCESS)
      {
         LogTools.error(String.format("%s returned '%d'", functionName, returnedState));
      }
      return returnedState == SL_ERROR_CODE_SUCCESS;
   }

   private String getCameraModel(int cameraID)
   {
      return switch (sl_get_camera_model(cameraID))
      {
         case 0 -> "ZED";
         case 1 -> "ZED Mini";
         case 2 -> "ZED 2";
         case 3 -> "ZED 2i";
         case 4 -> "ZED X";
         case 5 -> "ZED XM";
         default -> "Unknown model";
      };
   }

   private void setZEDConfiguration(int cameraID)
   {
      switch (sl_get_camera_model(cameraID))
      {
         case 0 -> zedModelData = ZEDModelData.ZED;
         case 1 -> zedModelData = ZEDModelData.ZED_MINI;
         case 2 -> zedModelData = ZEDModelData.ZED_2;
         case 3 -> zedModelData = ZEDModelData.ZED_2I;
         case 4 -> zedModelData = ZEDModelData.ZED_X;
         case 5 -> zedModelData = ZEDModelData.ZED_X_MINI;
         default ->
         {
            zedModelData = ZEDModelData.ZED;
            LogTools.error("Failed to associate model number with a ZED sensor model");
         }
      }
   }

   public static void main(String[] args)
   {
      DualZEDUDPSender dualZEDUDPSender = new DualZEDUDPSender();
      dualZEDUDPSender.start();

      Runtime.getRuntime().addShutdownHook(new Thread(dualZEDUDPSender::stop));

      ThreadTools.sleepForever();
   }
}
