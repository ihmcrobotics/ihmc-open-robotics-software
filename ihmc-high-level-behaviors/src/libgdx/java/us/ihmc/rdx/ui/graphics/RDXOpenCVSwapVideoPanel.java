package us.ihmc.rdx.ui.graphics;

import us.ihmc.rdx.imgui.ImGuiVideoPanel;
import us.ihmc.tools.thread.SwapReference;

/**
 * This class is designed to (at most) double the display frame rate of images which
 * need asynchronous updates because those updates are either computationaly
 * expensive (i.e. slow) or rely on blocking calls as for reading from a webcam
 * or sensor.
 *
 * It manages two OpenGL textures that get swapped out and provides an easy way to
 * operate on them with OpenCV.
 */
public class RDXOpenCVSwapVideoPanel
{
   private final ImGuiVideoPanel videoPanel;
   private final SwapReference<RDXOpenCVSwapVideoPanelData> dataSwapReference;

   public RDXOpenCVSwapVideoPanel(String panelName)
   {
      this.videoPanel = new ImGuiVideoPanel(panelName, false);
      dataSwapReference = new SwapReference<>(RDXOpenCVSwapVideoPanelData::new);
   }

   /**
    * If you know the dimensions in advance, you can call this once on initialization.
    * Don't call this after the threads are running, though.
    */
   public void allocateInitialTextures(int imageWidth, int imageHeight)
   {
      dataSwapReference.initializeBoth(data -> data.ensureTextureDimensions(imageWidth, imageHeight));
   }

   public ImGuiVideoPanel getVideoPanel()
   {
      return videoPanel;
   }

   /**
    * Synchronize on this object around accessing this data.
    */
   public RDXOpenCVSwapVideoPanelData getUIThreadData()
   {
      return dataSwapReference.getForThreadOne();
   }

   /**
    * Most of the time this is all you need to do on the UI thread.
    *
    * This method has internal synchronization so you don't have to do it outside
    * of this if it is the only thing you need to do on the UI thread.
    */
   public void updateTextureAndDrawOnUIThread()
   {
      synchronized (dataSwapReference)
      {
         RDXOpenCVSwapVideoPanelData data = getUIThreadData();
         if (data.getRGBA8Image() != null)
         {
            data.updateTextureAndDraw(videoPanel);
         }
      }
   }

   /**
    * Access this data at any time on the asynchronous thread, no synchronization
    * required, just call swap() when you're done.
    */
   public RDXOpenCVSwapVideoPanelData getAsynchronousThreadData()
   {
      return dataSwapReference.getForThreadTwo();
   }

   /**
    * Atomic swap operation. Call only from the asynchronous thread.
    */
   public void swap()
   {
      dataSwapReference.swap();
   }
}
