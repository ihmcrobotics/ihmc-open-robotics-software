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
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.perception.spinnaker.SpinnakerTools.assertNoError;

public class BytedecoBlackfly
{
   private final spinCamera spinCamera;
   private final String acquisitionMode;
   private final String serialNumber;
   private spinNodeMapHandle transportLayerDeviceNodeMap = null;
   private spinNodeMapHandle cameraNodeMap = null;

   private AtomicBoolean doImageAcquisition;
   private Thread imageAcquisitionService;

   private AtomicReference<spinImage> currentUnprocessedImage;
   private spinImage previousImage = null;
   private spinImage currentImage = null;

   protected BytedecoBlackfly(spinCamera spinCamera, String acquisitionMode, String serialNumber)
   {
      this.spinCamera = spinCamera;
      this.acquisitionMode = acquisitionMode;
      this.serialNumber = serialNumber;
   }

   public void initialize()
   {
      transportLayerDeviceNodeMap = new spinNodeMapHandle();
      cameraNodeMap = new spinNodeMapHandle();
      assertNoError(Spinnaker_C.spinCameraGetTLDeviceNodeMap(spinCamera, transportLayerDeviceNodeMap), "Unable to get transport layer device node map");
      assertNoError(Spinnaker_C.spinCameraInit(spinCamera), "Unable to initialize camera");
      assertNoError(Spinnaker_C.spinCameraGetNodeMap(spinCamera, cameraNodeMap), "Unable to retrieve GenICam nodemap");

      // Acquisition mode
      spinNodeHandle acquisitionModeNode = new spinNodeHandle();
      assertNoError(Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("AcquisitionMode"), acquisitionModeNode), "Unable to set acquisition mode");

      // TODO: What the heck is going on here? Doesn't seem like these are doing anything
      spinNodeHandle setAcquisitionMode = new spinNodeHandle();
      assertNoError(Spinnaker_C.spinEnumerationGetEntryByName(acquisitionModeNode, new BytePointer(acquisitionMode), setAcquisitionMode),
                    "Unable to set acquisition mode");
      LongPointer acquisitionModePointer = new LongPointer(1);
      assertNoError(Spinnaker_C.spinEnumerationEntryGetIntValue(setAcquisitionMode, acquisitionModePointer),
                    "Unable to set acquisition mode (int value retrieval)");
      assertNoError(Spinnaker_C.spinEnumerationSetIntValue(acquisitionModeNode, acquisitionModePointer.get()),
                    "Unable to set acquisition mode (int value set)");

      // Pixel format
      spinNodeHandle pixelFormatNode = new spinNodeHandle();
      assertNoError(Spinnaker_C.spinNodeMapGetNode(cameraNodeMap, new BytePointer("PixelFormat"), pixelFormatNode), "Unable to set pixel format");

      spinNodeHandle setPixelFormat = new spinNodeHandle();
      assertNoError(Spinnaker_C.spinEnumerationGetEntryByName(pixelFormatNode, new BytePointer("RGB8"), setPixelFormat), "Unable to set pixel format");
      LongPointer ptrPixelFormat = new LongPointer(1);
      assertNoError(Spinnaker_C.spinEnumerationEntryGetIntValue(setPixelFormat, ptrPixelFormat), "Unable to set pixel format (int value retrieval)");
      assertNoError(Spinnaker_C.spinEnumerationSetIntValue(pixelFormatNode, ptrPixelFormat.get()), "Unable to set pixel format (int value set)");

      assertNoError(Spinnaker_C.spinCameraBeginAcquisition(spinCamera), "Failed to begin acquiring images");

      // Image acquisition needs to run on a different thread so that the whole program doesn't need to wait for new images
      doImageAcquisition = new AtomicBoolean(true);
      currentUnprocessedImage = new AtomicReference<>(null);
      imageAcquisitionService = ThreadTools.startAThread(() ->
      {
         while (doImageAcquisition.get())
         {
            spinImage spinImage = new spinImage();
            Spinnaker_C.spinCameraGetNextImage(spinCamera, spinImage);

            BytePointer isIncomplete = new BytePointer(1);
            Spinnaker_C.spinImageIsIncomplete(spinImage, isIncomplete);
            if (isIncomplete.getBool())
            {
               LogTools.warn("Camera " + serialNumber + " returned incomplete image");
               Spinnaker_C.spinImageRelease(spinImage);
               continue;
            }

            spinImage oldImage = currentUnprocessedImage.get();
            currentUnprocessedImage.set(spinImage);
            Spinnaker_C.spinImageRelease(oldImage);
         }
      }, "Blackfly " + this.serialNumber + " Image Acquisition");
   }

   public int getHeight()
   {
      if (currentImage == null)
         return 0;

      SizeTPointer heightPointer = new SizeTPointer(1);
      assertNoError(Spinnaker_C.spinImageGetHeight(currentImage, heightPointer), "Height could not be determined");
      return (int) heightPointer.get();
   }

   public int getWidth()
   {
      if (currentImage == null)
         return 0;

      SizeTPointer widthPointer = new SizeTPointer(1);
      assertNoError(Spinnaker_C.spinImageGetWidth(currentImage, widthPointer), "Width could not be determined");
      return (int) widthPointer.get();
   }

   public boolean readFrameData()
   {
      if (currentUnprocessedImage.get() == previousImage || currentUnprocessedImage.get() == null)
         return false;

      spinImage spinImage = new spinImage();
      Spinnaker_C.spinImageCreateEmpty(spinImage);
      Spinnaker_C.spinImageConvert(currentUnprocessedImage.get(), Spinnaker_C.spinPixelFormatEnums.PixelFormat_RGBa8, spinImage);

      spinImage oldImage = currentImage;

      previousImage = currentUnprocessedImage.get();
      currentImage = spinImage;

      if (oldImage != null)
         Spinnaker_C.spinImageDestroy(oldImage);

      return true;
   }

   /**
    * It appears this call will return two alternating memory segments so it can do
    * a zero copy double buffer internally.
    */
   public void getImageData(Pointer pointer)
   {
      Spinnaker_C.spinImageGetData(currentImage, pointer);
   }

   public void destroy()
   {
      doImageAcquisition.set(false);
      try
      {
         imageAcquisitionService.wait();
      }
      catch (InterruptedException ex)
      {
      } // this is probably fine
      Spinnaker_C.spinImageRelease(currentUnprocessedImage.get());

      Spinnaker_C.spinCameraRelease(spinCamera);
   }
}
