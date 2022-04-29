package us.ihmc.perception.realsense;

import static org.bytedeco.librealsense2.global.realsense2.rs2_release_frame;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.ShortBuffer;
import java.util.ArrayList;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.MutableBytePointer;
import org.bytedeco.librealsense2.rs2_config;
import org.bytedeco.librealsense2.rs2_context;
import org.bytedeco.librealsense2.rs2_device;
import org.bytedeco.librealsense2.rs2_error;
import org.bytedeco.librealsense2.rs2_frame;
import org.bytedeco.librealsense2.rs2_intrinsics;
import org.bytedeco.librealsense2.rs2_options;
import org.bytedeco.librealsense2.rs2_pipeline;
import org.bytedeco.librealsense2.rs2_pipeline_profile;
import org.bytedeco.librealsense2.rs2_sensor;
import org.bytedeco.librealsense2.rs2_sensor_list;
import org.bytedeco.librealsense2.rs2_stream_profile;
import org.bytedeco.librealsense2.global.realsense2;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * Realsense2 API from bytedeco to poll depth data from an L515. See
 * http://bytedeco.org/javacpp-presets/librealsense2/apidocs/ for more info It has been tested to be
 * Garbage free and is required to be garbage free to run properly within a real time thread Please
 * profile if you make any changes. Most of the Realsense2 pointer methods return a new pointer and
 * are not real time compatible
 */
public class RealtimeL515
{
   protected final int width = 1024;
   protected final int height = 768;
   protected final int fps = 30;
   protected final int DEPTH_STREAM_INDEX = -1;

   protected final String name = getClass().getSimpleName();
   protected final YoRegistry registry;

   //The device (a device contains sensors like cameras and IMUS)
   protected final rs2_device device;

   // Declare RealSense pipeline, encapsulating the actual device and sensors
   protected final rs2_pipeline pipeline;

   //Create a configuration for configuring the pipeline with a non default profile
   protected final rs2_config config;

   //the depth sensor
   protected rs2_sensor sensor;

   //error pointer, have to check it after every call
   protected final rs2_error e = new rs2_error();

   // pipeline configuration
   protected rs2_pipeline_profile selection;

   //camera intrinsic parameters
   protected rs2_intrinsics intrinsic_parameters = new rs2_intrinsics();
   protected rs2_stream_profile stream_profile;
   protected double depthToMeterConversion;

   //Variables to avoid garbage
   protected rs2_frame depthFrame = new rs2_frame();
   protected MutableBytePointer depth_pointer = null;
   protected byte[] depth_bytes = null;
   
   private ShortBuffer shortBuffer;

   //YoVariables
   protected long lastReceivedFrameTime;
   protected final YoLong timeBetweenFrames;
   protected final YoLong frameIndex;
   
   //objects that receive the depth data
   protected final ArrayList<L515DepthImageReceiver> depthDataReceivers = new ArrayList<>();
   private ReferenceFrame sensorFrame;

   /**
    * This class uses the Realsense2 API from bytedeco to poll perceptoin data from an L515
    * 
    * @param prefix               - prefix for yovariable names
    * @param context              - The Realsense library context
    * @param device               - the L515 Device
    * @param parentRegistry
    * @param graphicsListRegistry
    */
   public RealtimeL515(String prefix, rs2_context context, rs2_device device, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.device = device;
      pipeline = realsense2.rs2_create_pipeline(context, e);
      config = realsense2.rs2_create_config(e);

      //Realsense Configuration
      //Add desired streams to configuration
      realsense2.rs2_config_enable_stream(config, realsense2.RS2_STREAM_DEPTH, DEPTH_STREAM_INDEX, width, height, realsense2.RS2_FORMAT_Z16, fps, e);
      if (!check_error(e))
      {
         System.err.println("Failed to enable stream");
      }

      rs2_sensor_list sensorList = realsense2.rs2_query_sensors(device, e);
      sensor = realsense2.rs2_create_sensor(sensorList, 0, e);

      depthToMeterConversion = realsense2.rs2_get_depth_scale(sensor, e);

      BytePointer deviceInfo = realsense2.rs2_get_device_info(device, realsense2.RS2_CAMERA_INFO_SERIAL_NUMBER, e);
      String sn = deviceInfo.getString();
      System.out.println("Configured Depth Stream of L515 SN Device: " + sn);

      //YoVariable Stuff
      registry = new YoRegistry(prefix + name + "_" + sn);
      timeBetweenFrames = new YoLong(prefix + "timeBetweenFrames", registry);
      frameIndex = new YoLong(prefix + "frameIndex", registry);

      parentRegistry.addChild(registry);
   }

   /**
    * Call this method if you plan on trigger the laser from an external signal See
    * https://dev.intelrealsense.com/docs/lidar-camera-l515-multi-camera-setup for more information
    */
   public void enableInterCamSyncMode()
   {
      rs2_options options = new rs2_options(sensor);
      realsense2.rs2_set_option(options, realsense2.RS2_OPTION_INTER_CAM_SYNC_MODE, 1.0f, e);
      if (!check_error(e))
      {
         System.err.println("Failed to set sync mode");
      }
   }

   /**
    * This starts the depth data streaming
    */
   public void initialize()
   {
      // Start streaming with default recommended configuration
      selection = realsense2.rs2_pipeline_start_with_config(pipeline, config, e);
      if (!check_error(e))
      {
         return;
      }
   }

   public void update()
   {
      if (realsense2.rs2_pipeline_poll_for_frames(pipeline, depthFrame, e) == 1)
      {
         check_error(e);

         int depth_data_size = realsense2.rs2_get_frame_data_size(depthFrame, e);
         check_error(e);

         if (depth_data_size > 0)
         {
            long nanoTime = System.nanoTime();
            timeBetweenFrames.set(nanoTime - lastReceivedFrameTime);
            lastReceivedFrameTime = nanoTime;
            frameIndex.increment();

            //Stuff to make this garbage free. The deoth pointer is configured correctly on instantiation and then we can
            //update the pointer reference from then on
            if (depth_pointer == null)
            {
               depth_pointer = new MutableBytePointer(realsense2.rs2_get_frame_data(depthFrame, e));
               depth_bytes = new byte[depth_data_size];
            }
            else
            {
               long address = realsense2.rs2_get_frame_data_address(depthFrame, e);
               depth_pointer.setAddress(address);
            }

            //Get the intrinsic parameters once so we can go in and out of pixel space
            //Do it only once per session do avoid garbage
            if (stream_profile == null)
            {
               stream_profile = realsense2.rs2_get_frame_stream_profile(depthFrame, e);
               realsense2.rs2_get_video_stream_intrinsics(stream_profile, intrinsic_parameters, e);
            }

            //get the actual depth data 
            depth_pointer.get(depth_bytes, 0, depth_data_size);
            
            //the depth data is a Little Indianess unsigned short
            if (shortBuffer == null)
            {
               //When using the short buffer, be sure to use an explicit call to deal with unsigned numbers. Java doesn't like them
               shortBuffer = ByteBuffer.wrap(depth_bytes).order(ByteOrder.LITTLE_ENDIAN).asShortBuffer();
               //int val = Short.toUnsignedInt(shortBuffer.get(index));
            }
            
            //let the receiving classes get direct access to the depth data and the depth frame
            for(int i = 0; i < depthDataReceivers.size(); i++)
            {
               depthDataReceivers.get(i).receivedDepthData(depthFrame, shortBuffer);
            }
         }

         //these allows the C side to deallocate objects. Need to keep an eye on this and make sure we didn't miss anything.
         rs2_release_frame(depthFrame);
      }
   }
   
   /**
    * Add a depth data receiver,
    * This is essentially a call back once a depth image comes in
    * @param receiver
    */
   public void addDepthDataReceiver(L515DepthImageReceiver receiver)
   {
      depthDataReceivers.add(receiver);
   }

   public void removeDepthDataReceiver(L515DepthImageReceiver receiver)
   {
      depthDataReceivers.remove(receiver);
   }
   
   public void close()
   {
      realsense2.rs2_pipeline_stop(pipeline, e);

      // Release resources
      realsense2.rs2_delete_pipeline_profile(selection);
      realsense2.rs2_delete_config(config);
      realsense2.rs2_delete_pipeline(pipeline);
      realsense2.rs2_delete_device(device);
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

   public double getDepthToMeterConversion()
   {
      return depthToMeterConversion;
   }

   public rs2_intrinsics getIntrinsicParameters()
   {
      return intrinsic_parameters;
   }

   public int getDepthWidth()
   {
      return width;
   }

   public int getDepthHeight()
   {
      return height;
   }
   
   public void setSensorFrame(ReferenceFrame sensorFrame)
   {
      this.sensorFrame = sensorFrame;
   }

   public ReferenceFrame getSensorFrame()
   {
      return sensorFrame;
   }
}
