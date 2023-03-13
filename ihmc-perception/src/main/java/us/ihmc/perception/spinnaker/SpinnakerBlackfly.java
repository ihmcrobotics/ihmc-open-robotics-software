package us.ihmc.perception.spinnaker;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.LongPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.spinnaker.Spinnaker_C.spinCamera;
import org.bytedeco.spinnaker.Spinnaker_C.spinImage;
import org.bytedeco.spinnaker.Spinnaker_C.spinNodeHandle;
import org.bytedeco.spinnaker.Spinnaker_C.spinNodeMapHandle;
import org.bytedeco.spinnaker.global.Spinnaker_C;
import us.ihmc.log.LogTools;
import us.ihmc.tools.UnitConversions;

import static us.ihmc.perception.spinnaker.SpinnakerTools.assertNoError;

/**
 * Good reference: http://softwareservices.flir.com/BFS-U3-04S2/latest/Model/public/
 *
 * If you don't have it set on boot (didn't use official Spinnaker installation procedure) then
 * you need to run the following command before using this class. It will make sure the USB
 * buffers are large enough to handle the Blackfly stably.
 * <pre>sudo sh -c 'echo 2047 > /sys/module/usbcore/parameters/usbfs_memory_mb'</pre>
 *
 * Confluence page: https://confluence.ihmc.us/display/PER/Blackfly+Cameras
 */
public class SpinnakerBlackfly
{
   public static final double BFLY_U3_23S6C_WIDTH_PIXELS = 1920.0;
   public static final double BFLY_U3_23S6C_HEIGHT_PIXELS = 1200.0;
   // https://www.flir.com/products/blackfly-usb3/?model=BFLY-U3-23S6C-C&vertical=machine+vision&segment=iis
   public static final double BFLY_U3_23S6C_CMOS_SENSOR_FORMAT = UnitConversions.inchesToMeters(1.0 / 1.2);
   public static final double BFLY_U3_23S6C_CMOS_SENSOR_WIDTH = 0.01067;
   public static final double BFLY_U3_23S6C_CMOS_SENSOR_HEIGHT = 0.00800;

   private final spinCamera spinCamera;
   private final String serialNumber;
   private spinNodeMapHandle transportLayerDeviceNodeMap = null;
   private spinNodeMapHandle cameraNodeMap = null;
   private final BytePointer isIncomplete = new BytePointer(1);

   protected SpinnakerBlackfly(spinCamera spinCamera, String serialNumber)
   {
      this.spinCamera = spinCamera;
      this.serialNumber = serialNumber;

      transportLayerDeviceNodeMap = new spinNodeMapHandle();
      cameraNodeMap = new spinNodeMapHandle();
      assertNoError(Spinnaker_C.spinCameraGetTLDeviceNodeMap(spinCamera, transportLayerDeviceNodeMap), "Getting transport layer device node map");
      assertNoError(Spinnaker_C.spinCameraInit(spinCamera), "Initializing camera");
      assertNoError(Spinnaker_C.spinCameraGetNodeMap(spinCamera, cameraNodeMap), "Retrieving GenICam node map");
   }

   /**
    * There are three acquisition modes:
    * Continuous - acquires images continuously. This is the default mode.
    * Multi Frame - acquires a specified number of images before stopping acquisition.
    * Single Frame - acquires 1 image before stopping acquisition.
    * See http://softwareservices.flir.com/BFS-U3-04S2/latest/Model/public/AcquisitionControl.html
    */
   public void setAcquisitionMode(Spinnaker_C.spinAcquisitionModeEnums acquisitionMode)
   {
      // Acquisition mode
      spinNodeHandle acquisitionModeNode = new spinNodeHandle();
      assertNoError(Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("AcquisitionMode"), acquisitionModeNode),
                    "Getting acquisition mode node map node");

      // TODO: What the heck is going on here? Doesn't seem like these are doing anything
      spinNodeHandle setAcquisitionMode = new spinNodeHandle();
      String acquisitionModeString = acquisitionMode.toString();
      String selectorString = acquisitionModeString.substring(acquisitionModeString.lastIndexOf("_") + 1);
      assertNoError(Spinnaker_C.spinEnumerationGetEntryByName(acquisitionModeNode, new BytePointer(selectorString), setAcquisitionMode),
                    "Getting acquisition mode entry by name: " + selectorString);
      LongPointer acquisitionModePointer = new LongPointer(1);
      assertNoError(Spinnaker_C.spinEnumerationEntryGetIntValue(setAcquisitionMode, acquisitionModePointer),
                    "Getting acquisition mode int value");
      assertNoError(Spinnaker_C.spinEnumerationSetIntValue(acquisitionModeNode, acquisitionModePointer.get()),
                    "Setting acquisition mode int value");
   }

   /** See http://softwareservices.flir.com/BFS-U3-04S2/latest/Model/public/ImageFormatControl.html */
   public void setPixelFormat(Spinnaker_C.spinPixelFormatEnums pixelFormat)
   {
      // Pixel format
      spinNodeHandle pixelFormatNode = new spinNodeHandle();
      assertNoError(Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("PixelFormat"), pixelFormatNode), "Getting pixel format node map node");

      spinNodeHandle pixelFormatEntryNodeHandle = new spinNodeHandle();
      String pixelFormatString = pixelFormat.toString();
      String selectorString = pixelFormatString.substring(pixelFormatString.lastIndexOf("_") + 1);
      assertNoError(Spinnaker_C.spinEnumerationGetEntryByName(pixelFormatNode, new BytePointer(selectorString), pixelFormatEntryNodeHandle),
                    "Getting pixel format entry by name: " + selectorString);
      LongPointer ptrPixelFormat = new LongPointer(1L);
      assertNoError(Spinnaker_C.spinEnumerationEntryGetIntValue(pixelFormatEntryNodeHandle, ptrPixelFormat), "Getting pixel format int value");
      assertNoError(Spinnaker_C.spinEnumerationSetIntValue(pixelFormatNode, ptrPixelFormat.get()), "Setting pixel format int value");
   }

   public void startAcquiringImages()
   {
      assertNoError(Spinnaker_C.spinCameraBeginAcquisition(spinCamera), "Beginning camera acquisition");
   }

   public boolean getNextImage(spinImage spinImageToPack)
   {
      Spinnaker_C.spinCameraGetNextImage(spinCamera, spinImageToPack);
      Spinnaker_C.spinImageIsIncomplete(spinImageToPack, isIncomplete);
      boolean incomplete = isIncomplete.getBool();
      if (incomplete)
      {
         LogTools.warn("Camera " + serialNumber + " returned incomplete image");
      }
      return !incomplete;
   }

   public int getHeight(spinImage spinImage)
   {
      SizeTPointer heightPointer = new SizeTPointer(1);
      assertNoError(Spinnaker_C.spinImageGetHeight(spinImage, heightPointer), "Getting image height");
      return (int) heightPointer.get();
   }

   public int getWidth(spinImage spinImage)
   {
      SizeTPointer widthPointer = new SizeTPointer(1);
      assertNoError(Spinnaker_C.spinImageGetWidth(spinImage, widthPointer), "Getting image width");
      return (int) widthPointer.get();
   }

   /**
    * It appears this call will return alternating memory segments, but can switch to different ones over time, too.
    */
   public void setBytedecoPointerToSpinImageData(spinImage spinImage, Pointer pointer)
   {
      Spinnaker_C.spinImageGetData(spinImage, pointer);
   }

   public void stopAcquiringImages()
   {
      assertNoError(Spinnaker_C.spinCameraEndAcquisition(spinCamera), "Ending camera acquisition");
   }

   public void destroy()
   {
      Spinnaker_C.spinCameraRelease(spinCamera);
   }
}
