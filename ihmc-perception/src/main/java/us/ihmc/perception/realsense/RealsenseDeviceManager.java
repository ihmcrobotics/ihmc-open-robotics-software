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
public class RealsenseDeviceManager
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final YoGraphicsListRegistry graphicsListRegistry;
   private final YoDouble numberOfDevices = new YoDouble("numberOfDevices", registry);

   private final rs2_context context;
   private final rs2_device_list devices;
   private final rs2_error error = new rs2_error();

   public RealsenseDeviceManager()
   {
      this(null, null);
   }

   public RealsenseDeviceManager(YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
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

   public RealsenseDevice createFullFeaturedL515(String serialNumberToFind)
   {
      return createBytedecoRealsenseDevice(serialNumberToFind, RealsenseConfiguration.L515_COLOR_720P_DEPTH_768P_30HZ);
   }

   /**
    *  Creates Realsense Device handler.
    *
    *  @param serialNumberToFind   The device serial number found physically printed on the Realsense sensor
    *  @param configuration The requested device settings
    *  @return BytedecoRealsense device object for accessing sensor data and config information
    */
   public RealsenseDevice createBytedecoRealsenseDevice(String serialNumberToFind, RealsenseConfiguration configuration)
   {
      return createBytedecoRealsenseDevice(serialNumberToFind,
                                           configuration.getDepthWidth(),
                                           configuration.getDepthHeight(),
                                           configuration.getDepthFPS());
   }

   /**
   *  Creates Realsense Device handler.
   *
   *  @param serialNumberToFind   The device serial number found physically printed on the Realsense sensor
   *  @param depthWidth The width of the depth maps to be requested from the sensor firmware
   *  @param depthHeight  The height of depth maps to be requested from the sensor firmware
   *  @param fps Frames Per Second which is the frequency of update to be requested from the sensor firmware
   *  @return BytedecoRealsense device object for accessing sensor data and config information
   */
   public RealsenseDevice createBytedecoRealsenseDevice(String serialNumberToFind, int depthWidth, int depthHeight, int fps)
   {
      String sanitizedSerialNumberToFind = serialNumberToFind.toLowerCase();
      return new RealsenseDevice(context, createDevice(sanitizedSerialNumberToFind), sanitizedSerialNumberToFind, depthWidth, depthHeight, fps);
   }

   public rs2_device createDevice(String serialNumberToFind)
   {
      int rs2DeviceCount = updateDeviceCount();
      LogTools.info("{} Realsense device(s) detected.", rs2DeviceCount);

      for (int i = 0; i < rs2DeviceCount; i++)
      {
         rs2_device rs2Device = realsense2.rs2_create_device(devices, i, error);
         checkError();

         String deviceName = getDeviceInfo(rs2Device, realsense2.RS2_CAMERA_INFO_NAME);
         if (deviceName != null)
            LogTools.info("Realsense device found: {}", deviceName);

         String deviceSerialNumber = getDeviceInfo(rs2Device, realsense2.RS2_CAMERA_INFO_SERIAL_NUMBER);
         if (deviceSerialNumber != null)
         {
            LogTools.info("Realsense device matched serial number: {}", deviceSerialNumber);

            String deviceFirmwareVersion = getDeviceInfo(rs2Device, realsense2.RS2_CAMERA_INFO_FIRMWARE_VERSION);
            if (deviceFirmwareVersion != null)
               LogTools.info("Realsense device firmware version: {}", deviceFirmwareVersion);
            String deviceRecommendedFirmwareVersion = getDeviceInfo(rs2Device, realsense2.RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION);
            if (deviceRecommendedFirmwareVersion != null)
               LogTools.info("Realsense device recommended firmware version: {}", deviceRecommendedFirmwareVersion);

            if (deviceSerialNumber.contains(serialNumberToFind.toLowerCase()))
            {
               return rs2Device;
            }
         }

         // We didn't select this device.
         realsense2.rs2_delete_device(rs2Device);
      }

      LogTools.error("Device not found. Serial Number: {}", serialNumberToFind);
      return null;
   }

   private String getDeviceInfo(rs2_device rs2Device, int deviceInfoEnum)
   {
      int supportsInfo = realsense2.rs2_supports_device_info(rs2Device, deviceInfoEnum, error);
      checkError();

      if (supportsInfo == 1)
      {
         BytePointer infoBytePointer = realsense2.rs2_get_device_info(rs2Device, deviceInfoEnum, error);
         checkError();
         return infoBytePointer.getString();
      }
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
      // LogTools/log4j2 is no longer operational during JVM shutdown
      System.out.println("Deleting device list...");
      realsense2.rs2_delete_device_list(devices);
      System.out.println("Deleting context...");
      realsense2.rs2_delete_context(context);
      System.out.println("Deleted everything.");
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
