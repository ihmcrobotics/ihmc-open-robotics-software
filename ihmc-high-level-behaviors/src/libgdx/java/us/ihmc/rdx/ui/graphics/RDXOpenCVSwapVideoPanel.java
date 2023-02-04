package us.ihmc.rdx.ui.graphics;

import us.ihmc.rdx.imgui.ImGuiVideoPanel;
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
public class RDXOpenCVSwapVideoPanel
{
   private final ImGuiVideoPanel videoPanel;
   private final GuidedSwapReference<RDXOpenCVSwapVideoPanelData> dataSwapReferenceManager;

   public RDXOpenCVSwapVideoPanel(String panelName, Consumer<RDXOpenCVSwapVideoPanelData> updateOnAsynchronousThread)
   {
      this(panelName, updateOnAsynchronousThread, null);
   }

   public RDXOpenCVSwapVideoPanel(String panelName,
                                  Consumer<RDXOpenCVSwapVideoPanelData> updateOnAsynchronousThread,
                                  Consumer<RDXOpenCVSwapVideoPanelData> updateOnUIThread)
   {
      this(panelName, false, updateOnAsynchronousThread, updateOnUIThread);
   }

   public RDXOpenCVSwapVideoPanel(String panelName,
                                  boolean flipY,
                                  Consumer<RDXOpenCVSwapVideoPanelData> updateOnAsynchronousThread,
                                  Consumer<RDXOpenCVSwapVideoPanelData> updateOnUIThread)
   {
      this.videoPanel = new ImGuiVideoPanel(panelName, flipY);
      dataSwapReferenceManager = new GuidedSwapReference<>(RDXOpenCVSwapVideoPanelData::new,
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

   private void defaultUpdateOnUIThread(RDXOpenCVSwapVideoPanelData data)
   {
      if (data.getRGBA8Image() != null)
      {
         data.updateOnUIThread(videoPanel);
      }
   }

   public ImGuiVideoPanel getVideoPanel()
   {
      return videoPanel;
   }
}
