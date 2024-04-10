package us.ihmc.perception.realsense.example;

import static org.bytedeco.librealsense2.global.realsense2.rs2_release_frame;
import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.FAST_RTPS;

import java.io.IOException;
import java.util.function.Consumer;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.librealsense2.rs2_config;
import org.bytedeco.librealsense2.rs2_context;
import org.bytedeco.librealsense2.rs2_device;
import org.bytedeco.librealsense2.rs2_device_list;
import org.bytedeco.librealsense2.rs2_error;
import org.bytedeco.librealsense2.rs2_frame;
import org.bytedeco.librealsense2.rs2_frame_queue;
import org.bytedeco.librealsense2.rs2_pipeline;
import org.bytedeco.librealsense2.rs2_pipeline_profile;
import org.bytedeco.librealsense2.rs2_processing_block;
import org.bytedeco.librealsense2.rs2_stream_profile;
import org.bytedeco.librealsense2.rs2_vertex;
import org.bytedeco.librealsense2.global.realsense2;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.KryoObjectServer;
import us.ihmc.communication.net.PubSubNetClassListAdapter;
import us.ihmc.communication.packets.StereoPointCloudCompression;
import us.ihmc.communication.packets.StereoPointCloudCompression.ColorAccessor;
import us.ihmc.communication.packets.StereoPointCloudCompression.CompressionIntermediateVariablesPackage;
import us.ihmc.communication.packets.StereoPointCloudCompression.PointAccessor;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

/**
 * Realsense2 API from bytedeco to poll depth data from an L515. See
 * http://bytedeco.org/javacpp-presets/librealsense2/apidocs/ for more info
 */
public class StandAloneL515Streamer
{
   private static final boolean USE_KRYO = true;

   //   private static final int depthWidth = 1024;
   //   private static final int depthHeight = 768;
   //
   //   private static final int colorWidth = 1280;
   //   private static final int colorHeight = 720;

   private static final int depthWidth = 640;
   private static final int depthHeight = 480;

   private static final int colorWidth = 640;
   private static final int colorHeight = 480;

   private static final int depthFps = 30;
   private static final int colorFps = 30;

   private static final int DEPTH_STREAM_INDEX = -1;
   private static final int COLOR_STREAM_INDEX = -1;
   private static CompressionIntermediateVariablesPackage compressionIntermediateVariablesPackage = new CompressionIntermediateVariablesPackage();

   public static void main(String[] args) throws IOException
   {
      Consumer<StereoVisionPointCloudMessage> pointcloudPublisher;

      if (USE_KRYO)
      {
         KryoObjectServer server = new KryoObjectServer(6666,
                                                        new PubSubNetClassListAdapter(),
                                                        Conversions.megabytesToBytes(256),
                                                        Conversions.megabytesToBytes(4));

         pointcloudPublisher = msg ->
         {
            if (server.isConnected())
            {
               server.send(msg);
            }
         };

         server.connect();
      }
      else
      {
         ROS2Node ros2Node = ROS2Tools.createROS2Node(FAST_RTPS, "L515");
         ROS2Topic<StereoVisionPointCloudMessage> topic = ROS2Tools.IHMC_ROOT.withTypeName(StereoVisionPointCloudMessage.class);
         pointcloudPublisher = ros2Node.createPublisher(topic)::publish;

         System.out.println("Created Publisher on: " + topic.getName());
      }

      rs2_error e = new rs2_error();
      realsense2.rs2_log_to_console(realsense2.RS2_LOG_SEVERITY_ERROR, e);
      if (!check_error(e))
      {
         return;
      }
      rs2_context ctx = realsense2.rs2_create_context(realsense2.RS2_API_VERSION, e);
      if (!check_error(e))
      {
         return;
      }
      rs2_device_list list = realsense2.rs2_query_devices(ctx, e);
      if (!check_error(e))
      {
         return;
      }
      int rs2_list_size = realsense2.rs2_get_device_count(list, e);
      if (!check_error(e))
      {
         return;
      }
      System.out.printf("realsense device %d\n", rs2_list_size);
      if (rs2_list_size == 0)
      {
         return;
      }
      rs2_device rsdev = realsense2.rs2_create_device(list, 0, e);
      if (!check_error(e))
      {
         return;
      }

      if (rsdev == null)
      {
         System.err.println("device not found. serial number = ");
         return;
      }

      // Declare RealSense pipeline, encapsulating the actual device and sensors
      rs2_pipeline pipe = realsense2.rs2_create_pipeline(ctx, e);

      //Create a configuration for configuring the pipeline with a non default profile
      rs2_config cfg = realsense2.rs2_create_config(e);

      //Add desired streams to configuration
      realsense2.rs2_config_enable_stream(cfg,
                                          realsense2.RS2_STREAM_COLOR,
                                          COLOR_STREAM_INDEX,
                                          colorWidth,
                                          colorHeight,
                                          realsense2.RS2_FORMAT_RGB8,
                                          colorFps,
                                          e);
      realsense2.rs2_config_enable_stream(cfg,
                                          realsense2.RS2_STREAM_DEPTH,
                                          DEPTH_STREAM_INDEX,
                                          depthWidth,
                                          depthHeight,
                                          realsense2.RS2_FORMAT_Z16,
                                          depthFps,
                                          e);
      if (!check_error(e))
      {
         return;
      }

      // Start streaming with default recommended configuration
      // The default video configuration contains Depth and Color streams
      // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
      rs2_pipeline_profile selection = realsense2.rs2_pipeline_start_with_config(pipe, cfg, e);
      if (!check_error(e))
      {
         return;
      }

      rs2_frame_queue pointCloudProccesingInputQueue = realsense2.rs2_create_frame_queue(1, e);
      rs2_processing_block pointCloudProcessingBlock = realsense2.rs2_create_pointcloud(e);

      // Define align object that will be used to align to color viewport.
      // Creating align object is an expensive operation
      // that should not be performed in the main loop.
      rs2_processing_block align_to_color = realsense2.rs2_create_align(realsense2.RS2_STREAM_COLOR, e);
      if (!check_error(e))
      {
         return;
      }

      rs2_frame_queue align_queue = realsense2.rs2_create_frame_queue(1, e);
      if (!check_error(e))
      {
         return;
      }

      realsense2.rs2_start_processing_queue(align_to_color, align_queue, e);
      if (!check_error(e))
      {
         return;
      }

      realsense2.rs2_start_processing_queue(pointCloudProcessingBlock, pointCloudProccesingInputQueue, e);

      int psize = 100;
      IntPointer stream = new IntPointer(psize);
      IntPointer format = new IntPointer(psize);
      IntPointer indexP = new IntPointer(psize);
      IntPointer unique_id = new IntPointer(psize);
      IntPointer framerate = new IntPointer(psize);
      rs2_frame color_frame = null;
      rs2_frame depth_frame = null;

      byte[] color_byte = null;
      float[] verticeArray = null;

      MutableBytePointer color_pointer = null;

      long averageComputeTime = 0;
      long totalComputeTime = 0;
      long numberOfIterations = 0;

      System.out.println("Starting Stream");
      while (true)
      {
         rs2_frame tmpFrames = realsense2.rs2_pipeline_wait_for_frames(pipe, realsense2.RS2_DEFAULT_TIMEOUT, e);
         long startTime = System.nanoTime();
         numberOfIterations++;

         if (!check_error(e))
         {
            continue;
         }

         // Align depth frame to color viewport
         realsense2.rs2_frame_add_ref(tmpFrames, e);
         realsense2.rs2_process_frame(align_to_color, tmpFrames, e);
         rs2_frame frames = realsense2.rs2_wait_for_frame(align_queue, 5000, e);
         if (!check_error(e))
         {
            realsense2.rs2_release_frame(color_frame);
            realsense2.rs2_release_frame(depth_frame);
            realsense2.rs2_release_frame(tmpFrames);
            realsense2.rs2_release_frame(frames);

            continue;
         }
         realsense2.rs2_release_frame(tmpFrames);

         // retrieve each frame
         int num_of_frames = realsense2.rs2_embedded_frames_count(frames, e);
         for (int i = 0; i < num_of_frames; i++)
         {
            rs2_frame frame = realsense2.rs2_extract_frame(frames, i, e);
            rs2_stream_profile mode = realsense2.rs2_get_frame_stream_profile(frame, e);
            realsense2.rs2_get_stream_profile_data(mode, stream, format, indexP, unique_id, framerate, e);

            if (stream.get() == realsense2.RS2_STREAM_DEPTH)
            {
               depth_frame = frame;
            }

            if (stream.get() == realsense2.RS2_STREAM_COLOR)
            {
               color_frame = frame;
            }
         }

         if (color_frame == null || depth_frame == null)
         {
            // release frames
            realsense2.rs2_release_frame(color_frame);
            realsense2.rs2_release_frame(depth_frame);
            realsense2.rs2_release_frame(frames);
            continue;
         }

         //convert depth to point cloud
         realsense2.rs2_process_frame(pointCloudProcessingBlock, depth_frame, e);
         rs2_frame pointCloudFrame = realsense2.rs2_wait_for_frame(pointCloudProccesingInputQueue, 10000, e);
         check_error(e);

         //get color data
         int color_data_size = realsense2.rs2_get_frame_data_size(color_frame, e);
         if (color_pointer == null)
         {
            color_pointer = new MutableBytePointer(realsense2.rs2_get_frame_data(color_frame, e));
            color_byte = new byte[color_data_size];
         }
         else
         {
            long address = realsense2.rs2_get_frame_data_address(color_frame, e);
            color_pointer.setAddress(address);
         }

         color_pointer.get(color_byte, 0, color_data_size);

         //get point cloud data
         rs2_vertex vertices = realsense2.rs2_get_frame_vertices(pointCloudFrame, e);
         FloatPointer verticePointer = new FloatPointer(vertices);
         int numberOfPoints = realsense2.rs2_get_frame_points_count(pointCloudFrame, e);

         if (verticeArray == null)
         {
            verticeArray = new float[numberOfPoints * 3];
         }

         verticePointer.get(verticeArray, 0, numberOfPoints * 3);
         verticePointer.close();

         //         convertPointCloudToWorld.updatePointCoud(verticeArray, numberOfPoints);

         //         long startCompressTime = System.nanoTime();
         convertToStereoVisionPointCloudMessageFast(pointcloudPublisher, verticeArray, color_byte, numberOfPoints);
         //         long endTime = System.nanoTime() - startCompressTime;
         //         System.out.println(endTime);

         long computeTime = System.nanoTime() - startTime;
         totalComputeTime += computeTime;

         if (numberOfIterations == 30)
         {
            numberOfIterations = 0;
            averageComputeTime = totalComputeTime / 30;
            totalComputeTime = 0;
            System.out.println("averageComputeTime: " + averageComputeTime * 1.0e-6 + "[ms]");
         }

         rs2_release_frame(pointCloudFrame);

         realsense2.rs2_release_frame(color_frame);
         realsense2.rs2_release_frame(depth_frame);
         realsense2.rs2_release_frame(frames);
      }
   }

   private static void convertToStereoVisionPointCloudMessage(Consumer<StereoVisionPointCloudMessage> pointcloudPublisher,
                                                              float[] points,
                                                              byte[] rawColors,
                                                              int numberOfPoints)
   {
      Point3D[] pointCloud = new Point3D[numberOfPoints];
      int[] colors = new int[numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D point = new Point3D();

         double x = points[3 * i + 0];
         double y = points[3 * i + 1];
         double z = points[3 * i + 2];
         point.set(x, y, z);

         pointCloud[i] = point;

         byte r = rawColors[3 * i + 0];
         byte g = rawColors[3 * i + 1];
         byte b = rawColors[3 * i + 2];

         colors[i] = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | ((b & 0xFF) << 0);
      }

      StereoVisionPointCloudMessage compressPointCloud = StereoPointCloudCompression.compressPointCloud(1, pointCloud, colors, numberOfPoints, 0.002, null);
      pointcloudPublisher.accept(compressPointCloud);
   }

   private static void convertToStereoVisionPointCloudMessageFast(Consumer<StereoVisionPointCloudMessage> pointcloudPublisher,
                                                                  float[] points,
                                                                  byte[] rawColors,
                                                                  int numberOfPoints)
   {

      StereoVisionPointCloudMessage msg = StereoPointCloudCompression.compressPointCloud(System.nanoTime(),
                                                                                         PointAccessor.wrap(points),
                                                                                         ColorAccessor.wrapRGB(rawColors),
                                                                                         numberOfPoints,
                                                                                         0.002,
                                                                                         compressionIntermediateVariablesPackage);

      //      System.out.println(msg.getPointCloud().size() + ", " + msg.getColors().size());
      //      //      while (msg.getColors().size() > 100000)
      //      //         msg.getColors().removeAt(msg.getColors().size() - 1);
      //      msg.getPointCloud().clear();
      //      msg.getColors().clear();
      //      msg.getPointCloud().fill(0, 66416, (byte) 200);
      //            msg.getColors().fill(0, 0000, (byte)200);

      pointcloudPublisher.accept(msg);
   }

   private static boolean check_error(rs2_error e)
   {
      if (!e.isNull())
      {
         System.err.printf("%s(%s): %s%n",
                           realsense2.rs2_get_failed_function(e).getString(),
                           realsense2.rs2_get_failed_args(e).getString(),
                           realsense2.rs2_get_error_message(e).getString());
         return false;
      }
      return true;
   }
}
