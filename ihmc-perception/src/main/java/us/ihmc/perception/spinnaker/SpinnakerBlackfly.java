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
import org.bytedeco.spinnaker.global.Spinnaker_C.spinExposureAutoEnums;
import org.bytedeco.spinnaker.global.Spinnaker_C.spinExposureModeEnums;

import java.util.List;

import static us.ihmc.perception.spinnaker.SpinnakerBlackflyTools.printOnError;

/**
 * Good reference: http://softwareservices.flir.com/BFS-U3-04S2/latest/Model/public/
 *
 * Confluence page: https://confluence.ihmc.us/display/PER/Blackfly+Cameras
 */
public class SpinnakerBlackfly
{

   private final spinCamera spinCamera;
   private final String serialNumber;
   private final spinNodeMapHandle cameraNodeMap = new spinNodeMapHandle();
   private final spinNodeMapHandle streamNodeMap = new spinNodeMapHandle();
   private final BytePointer isIncomplete = new BytePointer(1);

   /**
    * Use SpinnakerBlackflyManager#createBlackfly
    */
   protected SpinnakerBlackfly(spinCamera spinCamera, String serialNumber)
   {
      this.spinCamera = spinCamera;
      this.serialNumber = serialNumber;

      spinNodeMapHandle transportLayerDeviceNodeMap = new spinNodeMapHandle();
      printOnError(Spinnaker_C.spinCameraGetTLDeviceNodeMap(spinCamera, transportLayerDeviceNodeMap), "Getting transport layer device node map");
      printOnError(Spinnaker_C.spinCameraInit(spinCamera), "Initializing camera");
      printOnError(Spinnaker_C.spinCameraGetNodeMap(spinCamera, cameraNodeMap), "Retrieving GenICam node map");
      printOnError(Spinnaker_C.spinCameraGetTLStreamNodeMap(spinCamera, streamNodeMap), "Retrieving stream node map");
   }

   public spinCamera getSpinCamera()
   {
      return spinCamera;
   }

   public String getSerialNumber()
   {
      return serialNumber;
   }

   /**
    * Set the buffer handling mode
    * @see Spinnaker_C.spinTLStreamBufferHandlingModeEnums
    */
   public void setBufferHandlingMode(Spinnaker_C.spinTLStreamBufferHandlingModeEnums bufferHandlingMode)
   {
      spinNodeHandle bufferHandlingModeNode = new spinNodeHandle();
      printOnError(Spinnaker_C.spinNodeMapGetNode(streamNodeMap, new BytePointer("StreamBufferHandlingMode"), bufferHandlingModeNode),
                   "Getting stream buffer handling mode node map node");
      spinNodeHandle setBufferHandlingMode = new spinNodeHandle();
      String bufferHandlingModeString = bufferHandlingMode.toString();
      String selectorString = bufferHandlingModeString.substring(bufferHandlingModeString.lastIndexOf("_") + 1);
      printOnError(Spinnaker_C.spinEnumerationGetEntryByName(bufferHandlingModeNode, new BytePointer(selectorString), setBufferHandlingMode),
                    "Getting stream buffer handling mode entry by name: " + selectorString);
      LongPointer bufferHandlingModePointer = new LongPointer(1);
      printOnError(Spinnaker_C.spinEnumerationEntryGetIntValue(setBufferHandlingMode, bufferHandlingModePointer),
                   "Getting stream buffer handling mode int value");
      printOnError(Spinnaker_C.spinEnumerationSetIntValue(bufferHandlingModeNode, bufferHandlingModePointer.get()),
                   "Setting stream buffer handling mode int value");
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
      printOnError(Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("AcquisitionMode"), acquisitionModeNode),
                   "Getting acquisition mode node map node");

      // TODO: What the heck is going on here? Doesn't seem like these are doing anything
      spinNodeHandle setAcquisitionMode = new spinNodeHandle();
      String acquisitionModeString = acquisitionMode.toString();
      String selectorString = acquisitionModeString.substring(acquisitionModeString.lastIndexOf("_") + 1);
      printOnError(Spinnaker_C.spinEnumerationGetEntryByName(acquisitionModeNode, new BytePointer(selectorString), setAcquisitionMode),
                    "Getting acquisition mode entry by name: " + selectorString);
      LongPointer acquisitionModePointer = new LongPointer(1);
      printOnError(Spinnaker_C.spinEnumerationEntryGetIntValue(setAcquisitionMode, acquisitionModePointer), "Getting acquisition mode int value");
      printOnError(Spinnaker_C.spinEnumerationSetIntValue(acquisitionModeNode, acquisitionModePointer.get()), "Setting acquisition mode int value");
   }

   /**
    * See http://softwareservices.flir.com/BFS-U3-04S2/latest/Model/public/ImageFormatControl.html
    */
   public void setPixelFormat(Spinnaker_C.spinPixelFormatEnums pixelFormat)
   {
      // Pixel format
      spinNodeHandle pixelFormatNode = new spinNodeHandle();
      printOnError(Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("PixelFormat"), pixelFormatNode), "Getting pixel format node map node");

      spinNodeHandle pixelFormatEntryNodeHandle = new spinNodeHandle();
      String pixelFormatString = pixelFormat.toString();
      String selectorString = pixelFormatString.substring(pixelFormatString.lastIndexOf("_") + 1);
      printOnError(Spinnaker_C.spinEnumerationGetEntryByName(pixelFormatNode, new BytePointer(selectorString), pixelFormatEntryNodeHandle),
                    "Getting pixel format entry by name: " + selectorString);
      LongPointer ptrPixelFormat = new LongPointer(1L);
      printOnError(Spinnaker_C.spinEnumerationEntryGetIntValue(pixelFormatEntryNodeHandle, ptrPixelFormat), "Getting pixel format int value");
      printOnError(Spinnaker_C.spinEnumerationSetIntValue(pixelFormatNode, ptrPixelFormat.get()), "Setting pixel format int value");
   }

   public void setResolution(int width, int height)
   {
      spinNodeHandle widthNode = new spinNodeHandle();
      printOnError(Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("Width"), widthNode), "Getting width node map node");
      Spinnaker_C.spinIntegerSetValue(widthNode, width);

      spinNodeHandle heightNode = new spinNodeHandle();
      printOnError(Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("Height"), heightNode), "Getting height node map node");
      Spinnaker_C.spinIntegerSetValue(heightNode, height);
   }


   public void setOffset(int xOffset, int yOffset)
   {
      spinNodeHandle xOffsetNode = new spinNodeHandle();
      printOnError(Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("OffsetX"), xOffsetNode), "Getting OffsetX node map node");
      Spinnaker_C.spinIntegerSetValue(xOffsetNode, xOffset);

      spinNodeHandle yOffsetNode = new spinNodeHandle();
      printOnError(Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("OffsetY"), yOffsetNode), "Getting OffsetY node map node");
      Spinnaker_C.spinIntegerSetValue(yOffsetNode, yOffset);

//      spinNodeHandle yOffsetGet = new spinNodeHandle();
//      Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("OffsetY"), yOffsetGet);
//      LongPointer yOffsetPointer = new LongPointer();
//      Spinnaker_C.spinIntegerGetValue(yOffsetGet, yOffsetPointer);
//      System.out.println("yOffsetPointer " + yOffsetPointer.get());
//
//      spinNodeHandle xOffsetGet = new spinNodeHandle();
//      Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("OffsetX"), yOffsetGet);
//      LongPointer xOffsetPointer = new LongPointer();
//      Spinnaker_C.spinIntegerGetValue(xOffsetGet, xOffsetPointer);
//      System.out.println("xOffsetPointer " + xOffsetPointer.get());
   }

   // http://softwareservices.flir.com/Spinnaker/latest/_programmer_guide.html#Setting_Exposure_Time
   public void setExposure(float exposureTimeMicroSecs)
   {
      // Disable auto exposure
      spinNodeHandle exposureAutoHandle = new spinNodeHandle();
      printOnError(Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("ExposureAuto"), exposureAutoHandle), "Getting ExposureAuto node map node");
      printOnError(Spinnaker_C.spinEnumerationSetEnumValue(exposureAutoHandle, spinExposureAutoEnums.ExposureAuto_Off.value), "Could not set ExposureAuto");

      // Set timed exposure mode
      spinNodeHandle exposureModeHandle = new spinNodeHandle();
      printOnError(Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("ExposureMode"), exposureModeHandle), "Getting ExposureMode node map node");
      printOnError(Spinnaker_C.spinEnumerationSetEnumValue(exposureModeHandle, spinExposureModeEnums.ExposureMode_Timed.value), "Could not set ExposureMode");

      // Set timed exposure value
      spinNodeHandle exposureTimeHandle = new spinNodeHandle();
      printOnError(Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("ExposureTime"), exposureTimeHandle), "Getting ExposureTime node map node");
      // This has to be a float, the docs were unclear
      printOnError(Spinnaker_C.spinFloatSetValue(exposureTimeHandle, exposureTimeMicroSecs), "Unable to set ExposureTime");
   }

   public void startAcquiringImages()
   {
      printOnError(Spinnaker_C.spinCameraBeginAcquisition(spinCamera), "Beginning camera acquisition");
   }

   public boolean getNextImage(spinImage spinImageToPack)
   {
      return printOnError(Spinnaker_C.spinCameraGetNextImage(spinCamera, spinImageToPack), "Grabbing Image").value
             == Spinnaker_C.spinError.SPINNAKER_ERR_SUCCESS.value;
   }

   public void releaseImage(spinImage spinImage)
   {
      Spinnaker_C.spinImageRelease(spinImage);
   }

   public int getHeight(spinImage spinImage)
   {
      SizeTPointer heightPointer = new SizeTPointer(1);
      printOnError(Spinnaker_C.spinImageGetHeight(spinImage, heightPointer), "Getting image height");
      int height = (int) heightPointer.get();
      heightPointer.close();
      return height;
   }

   public int getWidth(spinImage spinImage)
   {
      SizeTPointer widthPointer = new SizeTPointer(1);
      printOnError(Spinnaker_C.spinImageGetWidth(spinImage, widthPointer), "Getting image width");
      int width = (int) widthPointer.get();
      widthPointer.close();
      return width;
   }

   /**
    * It appears this call will return alternating memory segments, but can switch to different ones over time, too.
    */
   public void setPointerToSpinImageData(spinImage spinImage, Pointer pointer)
   {
      Spinnaker_C.spinImageGetData(spinImage, pointer);
   }

   public void stopAcquiringImages()
   {
      System.out.println("Stopping spinnaker blackfly");
      printOnError(Spinnaker_C.spinCameraEndAcquisition(spinCamera), "Ending camera acquisition");
   }
}
