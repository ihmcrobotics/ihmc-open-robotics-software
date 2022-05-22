package us.ihmc.perception.spinnaker;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.LongPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.SizeTPointer;
import org.bytedeco.spinnaker.Spinnaker_C.*;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static org.bytedeco.spinnaker.global.Spinnaker_C.*;
import static us.ihmc.perception.spinnaker.SpinnakerTools.assertNoError;

public class BytedecoBlackfly
{
   String serial;
   private spinNodeMapHandle nmTLDevice = null;
   private spinCamera camera;
   private spinNodeMapHandle camNodeMap = null;

   private String acquisitionMode;

   AtomicBoolean doImageAcquisition;
   Thread imageAcquisitionService;

   AtomicReference<spinImage> currentUnprocessedImage;
   spinImage previousImage = null;
   spinImage currentImage = null;

   protected BytedecoBlackfly(spinCamera camera, String acqMode, String serial)
   {
      this.camera = camera;
      this.acquisitionMode = acqMode;
      this.serial = serial;
   }

   public void initialize()
   {
      nmTLDevice = new spinNodeMapHandle();
      camNodeMap = new spinNodeMapHandle();
      assertNoError(spinCameraGetTLDeviceNodeMap(camera, nmTLDevice), "Unable to get TL device nodemap");
      assertNoError(spinCameraInit(camera), "Unable to initialize camera");
      assertNoError(spinCameraGetNodeMap(camera, camNodeMap), "Unable to retrieve GenICam nodemap");

      //Acquisition mode
      spinNodeHandle camAcquisitionMode = new spinNodeHandle();
      assertNoError(spinNodeMapGetNode(camNodeMap, new BytePointer("AcquisitionMode"), camAcquisitionMode), "Unable to set acquisition mode");

      spinNodeHandle setAcquisitionMode = new spinNodeHandle();
      assertNoError(spinEnumerationGetEntryByName(camAcquisitionMode, new BytePointer(acquisitionMode), setAcquisitionMode), "Unable to set acquisition mode");
      LongPointer ptrAcquisitionMode = new LongPointer(1);
      assertNoError(spinEnumerationEntryGetIntValue(setAcquisitionMode, ptrAcquisitionMode), "Unable to set acquisition mode (int value retrieval)");
      assertNoError(spinEnumerationSetIntValue(camAcquisitionMode, ptrAcquisitionMode.get()), "Unable to set acquisition mode (int value set)");

      //Pixel format
      spinNodeHandle camPixelFormat = new spinNodeHandle();
      assertNoError(spinNodeMapGetNode(camNodeMap, new BytePointer("PixelFormat"), camPixelFormat), "Unable to set pixel format");

      spinNodeHandle setPixelFormat = new spinNodeHandle();
      assertNoError(spinEnumerationGetEntryByName(camPixelFormat, new BytePointer("RGB8"), setPixelFormat), "Unable to set pixel format");
      LongPointer ptrPixelFormat = new LongPointer(1);
      assertNoError(spinEnumerationEntryGetIntValue(setPixelFormat, ptrPixelFormat), "Unable to set pixel format (int value retrieval)");
      assertNoError(spinEnumerationSetIntValue(camPixelFormat, ptrPixelFormat.get()), "Unable to set pixel format (int value set)");

      assertNoError(spinCameraBeginAcquisition(camera), "Failed to begin acquiring images");

      //Image acquisition needs to run on a different thread so that the whole program doesn't need to wait for new images
      doImageAcquisition = new AtomicBoolean(true);
      currentUnprocessedImage = new AtomicReference<>(null);
      imageAcquisitionService = ThreadTools.startAThread(new Runnable()
      {
         @Override
         public void run()
         {
            while (doImageAcquisition.get())
            {
               spinImage image = new spinImage();
               spinCameraGetNextImage(camera, image);

               BytePointer isIncomplete = new BytePointer(1);
               spinImageIsIncomplete(image, isIncomplete);
               if (isIncomplete.getBool())
               {
                  LogTools.warn("Camera " + serial + " returned incomplete image");
                  spinImageRelease(image);
                  continue;
               }

               spinImage oldImage = currentUnprocessedImage.get();
               currentUnprocessedImage.set(image);
               spinImageRelease(oldImage);
            }
         }
      }, "Blackfly " + this.serial + " Image Acquisition");
   }

   public int getHeight()
   {
      if (currentImage == null)
         return 0;

      SizeTPointer val = new SizeTPointer(1);
      assertNoError(spinImageGetHeight(currentImage, val), "Height could not be determined");
      return (int) val.get();
   }

   public int getWidth()
   {
      if (currentImage == null)
         return 0;

      SizeTPointer val = new SizeTPointer(1);
      assertNoError(spinImageGetWidth(currentImage, val), "Width could not be determined");
      return (int) val.get();
   }

   public boolean readFrameData()
   {
      if (currentUnprocessedImage.get() == previousImage || currentUnprocessedImage.get() == null)
         return false;

      spinImage image = new spinImage();
      spinImageCreateEmpty(image);
      spinImageConvert(currentUnprocessedImage.get(), spinPixelFormatEnums.PixelFormat_RGBa8, image);

      spinImage oldImage = currentImage;

      previousImage = currentUnprocessedImage.get();
      currentImage = image;

      if (oldImage != null)
         spinImageDestroy(oldImage);

      return true;
   }

   /**
    * It appears this call will return two alternating memory segments so it can do
    * a zero copy double buffer internally.
    */
   public void getImageData(Pointer pointer)
   {
      spinImageGetData(currentImage, pointer);
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
      } //this is probably fine
      spinImageRelease(currentUnprocessedImage.get());

      spinCameraRelease(camera);
   }
}
