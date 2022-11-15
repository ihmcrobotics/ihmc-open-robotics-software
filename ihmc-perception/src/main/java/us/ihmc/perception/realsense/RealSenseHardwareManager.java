package us.ihmc.perception.realsense;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.librealsense2.rs2_context;
import org.bytedeco.librealsense2.rs2_device;
import org.bytedeco.librealsense2.rs2_device_list;
import org.bytedeco.librealsense2.rs2_error;
import org.bytedeco.librealsense2.global.realsense2;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.tools.string.StringTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.function.Supplier;

/**
 * This classes manages Realsense2 sensors using the API from bytedeco. See http://bytedeco.org/javacpp-presets/librealsense2/apidocs/ for more info
 * It has been tested to be garbage free and is required to be garbage free to run properly within a real time thread
 * Please profile if you make any changes. Most of the Realsense2 pointer methods return a new pointer and are not real time compatible
 */
public class RealSenseHardwareManager
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final YoGraphicsListRegistry graphicsListRegistry;
   private final YoDouble numberOfDevices = new YoDouble("numberOfDevices", registry);

   private final rs2_context context;
   private final rs2_device_list devices;
   private final rs2_error error = new rs2_error();

   public RealSenseHardwareManager()
   {
      this(null, null);
   }

   public RealSenseHardwareManager(YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.graphicsListRegistry = graphicsListRegistry;

      realsense2.rs2_log_to_console(realsense2.RS2_LOG_SEVERITY_ERROR, error);
      checkError();

      context = realsense2.rs2_create_context(realsense2.RS2_API_VERSION, error);
      checkError();

      devices = realsense2.rs2_query_devices(context, error);
      checkError();

      updateDeviceCount();

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public RealtimeL515 createRealtimeL515(String prefix, String serialNumberToFind)
   {
      String sanitizedSerialNumberToFind = serialNumberToFind.toLowerCase();
      return new RealtimeL515(prefix, context, createDevice(sanitizedSerialNumberToFind), sanitizedSerialNumberToFind, registry, graphicsListRegistry);
   }

   public BytedecoRealsense createFullFeaturedL515(String serialNumberToFind)
   {
      return createFullFeaturedL515(serialNumberToFind, 1024, 768, 30);
   }

   public BytedecoRealsense createFullFeaturedL515(String serialNumberToFind, int depthWidth, int depthHeight, int fps)
   {
      String sanitizedSerialNumberToFind = serialNumberToFind.toLowerCase();
      return new BytedecoRealsense(context, createDevice(sanitizedSerialNumberToFind), sanitizedSerialNumberToFind, depthWidth, depthHeight, fps);
   }

   public BytedecoRealsense createD435(String serialNumberToFind)
   {
      return createD435(serialNumberToFind, 848, 480, 30);
   }

   public BytedecoRealsense createD435(String serialNumberToFind, int depthWidth, int depthHeight, int fps)
   {
      String sanitizedSerialNumberToFind = serialNumberToFind.toLowerCase();
      return new BytedecoRealsense(context, createDevice(sanitizedSerialNumberToFind), sanitizedSerialNumberToFind, depthWidth, depthHeight, fps);
   }

   public BytedecoRealsense createBytedecoRealsense(String serialNumberToFind, int depthWidth, int depthHeight, int fps)
   {
      String sanitizedSerialNumberToFind = serialNumberToFind.toLowerCase();
      return new BytedecoRealsense(context, createDevice(sanitizedSerialNumberToFind), sanitizedSerialNumberToFind, depthWidth, depthHeight, fps);
   }

   public rs2_device createDevice(String serialNumberToFind)
   {
      int rs2DeviceCount = updateDeviceCount();

      for (int i = 0; i < rs2DeviceCount; i++)
      {
         rs2_device rs2Device = realsense2.rs2_create_device(devices, i, error);
         checkError();

         int supportsSerialNumber = realsense2.rs2_supports_device_info(rs2Device, realsense2.RS2_CAMERA_INFO_SERIAL_NUMBER, error);
         checkError();

         if (supportsSerialNumber == 1)
         {
            BytePointer deviceSerialNumberBytePointer = realsense2.rs2_get_device_info(rs2Device, realsense2.RS2_CAMERA_INFO_SERIAL_NUMBER, error);
            checkError();

            String serialNumberFromRS2 = deviceSerialNumberBytePointer.getString();
            LogTools.info("Realsense Sensor detected. Serial Number: = {}", serialNumberFromRS2);

            if (serialNumberFromRS2.contains(serialNumberToFind.toLowerCase()))
            {
               return rs2Device;
            }
         }
      }

      LogTools.error("Device not found. Serial Number: = {}", serialNumberToFind);
      return null;
   }

   private int updateDeviceCount()
   {
      int rs2DeviceCount = realsense2.rs2_get_device_count(devices, error);
      checkError();

      numberOfDevices.set(rs2DeviceCount);
      if (rs2DeviceCount == 0)
      {
         LogTools.info("No Realsense devices found!");
      }
      return rs2DeviceCount;
   }

   public void deleteContext()
   {
      realsense2.rs2_delete_device_list(devices);
      realsense2.rs2_delete_context(context);
      LogTools.info("Deleted realsense2 context");
   }

   private void checkError()
   {
      if (!error.isNull())
      {
         Supplier<String> errorMessage = StringTools.format("{}({}): {}",
                                                            realsense2.rs2_get_failed_function(error).getString(),
                                                            realsense2.rs2_get_failed_args(error).getString(),
                                                            realsense2.rs2_get_error_message(error).getString());
         LogTools.error(errorMessage);
         throw new RuntimeException(errorMessage.get());
      }
   }
}
