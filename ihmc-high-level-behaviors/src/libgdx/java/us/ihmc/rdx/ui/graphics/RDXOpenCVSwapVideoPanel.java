package us.ihmc.rdx.ui.graphics;

import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.tools.thread.SwapReference;

import java.util.function.Consumer;

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
   private final RDXImagePanel imagePanel;
   private final SwapReference<RDXImagePanelTexture> dataSwapReference;

   public RDXOpenCVSwapVideoPanel(String panelName)
   {
      this.imagePanel = new RDXImagePanel(panelName, RDXImagePanel.DO_NOT_FLIP_Y);
      dataSwapReference = new SwapReference<>(RDXImagePanelTexture::new);
   }

   /**
    * If you know the dimensions in advance, you can call this once on initialization.
    * Don't call this after the threads are running, though.
    */
   public void allocateInitialTextures(int imageWidth, int imageHeight)
   {
      dataSwapReference.initializeBoth(data -> data.ensureTextureDimensions(imageWidth, imageHeight));
   }

   public RDXImagePanel getImagePanel()
   {
      return imagePanel;
   }

   /**
    * Synchronize on getSyncObject() around accessing this data.
    */
   public RDXImagePanelTexture getUIThreadData()
   {
      return dataSwapReference.getForThreadOne();
   }

   /**
    * Used for synchronized over access of the UI thread data.
    * We intentionally erase the type because we don't want the user
    * calling methods directly on dataSwapReference. It would be confusing.
    */
   public Object getSyncObject()
   {
      return dataSwapReference;
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
         RDXImagePanelTexture texture = getUIThreadData();
         if (texture.getRGBA8Image() != null)
         {
            texture.updateTextureAndDraw(imagePanel);
         }
      }
   }

   /**
    * Access this data at any time on the asynchronous thread, no synchronization
    * required, just call swap() when you're done.
    */
   public RDXImagePanelTexture getAsynchronousThreadData()
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
