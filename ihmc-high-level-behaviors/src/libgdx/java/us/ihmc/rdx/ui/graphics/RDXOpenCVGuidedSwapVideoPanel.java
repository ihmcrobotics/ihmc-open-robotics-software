package us.ihmc.rdx.ui.graphics;

import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.tools.thread.GuidedSwapReference;

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
public class RDXOpenCVGuidedSwapVideoPanel
{
   private final RDXImagePanel imagePanel;
   private final GuidedSwapReference<RDXImagePanelTexture> dataSwapReferenceManager;

   public RDXOpenCVGuidedSwapVideoPanel(String panelName, Consumer<RDXImagePanelTexture> updateOnAsynchronousThread)
   {
      this(panelName, updateOnAsynchronousThread, null);
   }

   public RDXOpenCVGuidedSwapVideoPanel(String panelName,
                                        Consumer<RDXImagePanelTexture> updateOnAsynchronousThread,
                                        Consumer<RDXImagePanelTexture> updateOnUIThread)
   {
      this(panelName, false, updateOnAsynchronousThread, updateOnUIThread);
   }

   public RDXOpenCVGuidedSwapVideoPanel(String panelName,
                                        boolean flipY,
                                        Consumer<RDXImagePanelTexture> updateOnAsynchronousThread,
                                        Consumer<RDXImagePanelTexture> updateOnUIThread)
   {
      this.imagePanel = new RDXImagePanel(panelName, flipY);
      dataSwapReferenceManager = new GuidedSwapReference<>(RDXImagePanelTexture::new,
                                                           updateOnAsynchronousThread,
                                                           updateOnUIThread == null ? this::defaultUpdateOnUIThread : updateOnUIThread);
   }

   /**
    * If you know the dimensions in advance, you can call this once on initialization.
    * Don't call this after the threads are running, though.
    */
   public void allocateInitialTextures(int imageWidth, int imageHeight)
   {
      dataSwapReferenceManager.initializeBoth(data -> data.ensureTextureDimensions(imageWidth, imageHeight));
   }

   public void updateOnAsynchronousThread()
   {
      dataSwapReferenceManager.accessOnLowPriorityThread();
   }

   public void updateOnUIThread()
   {
      dataSwapReferenceManager.accessOnHighPriorityThread();
   }

   private void defaultUpdateOnUIThread(RDXImagePanelTexture texture)
   {
      if (texture.getRGBA8Image() != null)
      {
         texture.updateTextureAndDraw(imagePanel);
      }
   }

   public RDXImagePanel getImagePanel()
   {
      return imagePanel;
   }
}
