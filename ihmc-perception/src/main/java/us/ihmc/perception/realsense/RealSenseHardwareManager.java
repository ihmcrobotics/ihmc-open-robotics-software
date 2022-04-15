package us.ihmc.perception.realsense;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.librealsense2.rs2_context;
import org.bytedeco.librealsense2.rs2_device;
import org.bytedeco.librealsense2.rs2_device_list;
import org.bytedeco.librealsense2.rs2_error;
import org.bytedeco.librealsense2.global.realsense2;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This classes manages Realsense2 sensors using the API from bytedeco. See http://bytedeco.org/javacpp-presets/librealsense2/apidocs/ for more info
 * It has been tested to be Garbage free and is required to be garbage free to run properly within a real time thread
 * Please profile if you make any changes. Most of the Realsense2 pointer methods return a new pointer and are not real time compatible
 */
public class RealSenseHardwareManager
{
   private String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final YoGraphicsListRegistry graphicsListRegistry;

   private final YoDouble numberOfDevices = new YoDouble("numberOfDevices", registry);
   
   private rs2_context context;
   private rs2_device_list list;
   
   private final rs2_error e = new rs2_error();
   public RealSenseHardwareManager(YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.graphicsListRegistry = graphicsListRegistry;
      realsense2.rs2_log_to_console(realsense2.RS2_LOG_SEVERITY_ERROR, e);
      if (!check_error(e)) {
          return;
      }
      
      context = realsense2.rs2_create_context(realsense2.RS2_API_VERSION, e);
      if (!check_error(e)) {
          return;
      }
      
      list = realsense2.rs2_query_devices(context, e);
      if (!check_error(e)) {
          return;
      }
      
      int rs2_list_size = realsense2.rs2_get_device_count(list, e);
      if (!check_error(e)) {
          return;
      }
      
      numberOfDevices.set(rs2_list_size);
      if (rs2_list_size == 0) {
          System.out.println("No Realsense devices found!");;
      }
      
      parentRegistry.addChild(registry);
   }
   
   public RealtimeL515 getL515(String prefix, String serialNumber)
   {
      int rs2_list_size = realsense2.rs2_get_device_count(list, e);
      for(int i = 0; i < rs2_list_size; i++)
      {
         rs2_device rsdev = realsense2.rs2_create_device(list, i, e);
         if (!check_error(e)) 
         {
            return null;
         }
         
         int supportsSN = realsense2.rs2_supports_device_info(rsdev, realsense2.RS2_CAMERA_INFO_SERIAL_NUMBER, e);
         if (!check_error(e)) 
         {
            return null;
         }
         
         if(supportsSN == 1)
         {
            
            BytePointer deviceInfo = realsense2.rs2_get_device_info(rsdev, realsense2.RS2_CAMERA_INFO_SERIAL_NUMBER, e);
            if (!check_error(e)) 
            {
               return null;
            }
            
            String sn = deviceInfo.getString();
            System.out.println("Realsense Sensor detected. Serial Number: = " + sn);
            
            if(sn.contains(serialNumber))
            {
               return new RealtimeL515(prefix, context, rsdev, registry, graphicsListRegistry);
            }
         }
      }
      
       System.err.println("Device not found - Serial Number: = " + serialNumber);
       return null;
   }
   
   public void close()
   {
      realsense2.rs2_delete_device_list(list);
      realsense2.rs2_delete_context(context);
      System.out.println("Finished");
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
