package us.ihmc.jMonkeyEngineToolkit.jme.context;

import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL12.*;
import static org.lwjgl.opengl.GL15.*;
import static org.lwjgl.opengl.GL21.*;

import java.awt.AWTException;
import java.awt.BufferCapabilities;
import java.awt.Canvas;
import java.awt.Graphics2D;
import java.awt.ImageCapabilities;
import java.awt.RenderingHints;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.image.BufferStrategy;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.awt.image.WritableRaster;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;

import com.jme3.post.SceneProcessor;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.Image.Format;
import com.jme3.util.BufferUtils;

/**
 * PBO - Pixel Buffer Object
 *
 * It's more efficient than reading the pixels. @jsmith
 */
public class PBOAwtPanel extends Canvas implements SceneProcessor
{
   private static final long serialVersionUID = -8871753166643132265L;

   private static final boolean DEBUG = false;

   private boolean attachAsMain = false;

   private BufferedImage bufferedImage;
   private FrameBuffer frameBuffer;
   private RenderManager renderManager;
   private ArrayList<ViewPort> viewPorts = new ArrayList<ViewPort>();
   private ArrayList<PBOAwtPanelListener> pboAwtPanelListeners;

   // Visibility/drawing vars
   private BufferStrategy strategy;
   private AtomicBoolean hasNativePeer = new AtomicBoolean(false);
   private AtomicBoolean showing = new AtomicBoolean(false);
   private AtomicBoolean repaintRequest = new AtomicBoolean(false);

   // Reshape vars
   private int newWidth = 1;
   private int newHeight = 1;
   private AtomicBoolean reshapeNeeded = new AtomicBoolean(false);
   private final Object lock = new Object();

   // PBOs
   private int gpuToVram, vramToSys;
   private int dataSize;

   public PBOAwtPanel(ArrayList<PBOAwtPanelListener> pboAwtPanelListeners)
   {
      this.pboAwtPanelListeners = pboAwtPanelListeners;

      setIgnoreRepaint(true);

      addComponentListener(new ComponentAdapter()
      {
         @Override
         public void componentResized(ComponentEvent e)
         {
            synchronized (lock)
            {
               int newWidth2 = Math.max(getWidth(), 1);
               int newHeight2 = Math.max(getHeight(), 1);
               if ((newWidth != newWidth2) || (newHeight != newHeight2))
               {
                  newWidth = newWidth2;
                  newHeight = newHeight2;
                  reshapeNeeded.set(true);
               }
            }
         }
      });
   }

   @Override
   public void addNotify()
   {
      super.addNotify();

      synchronized (lock)
      {
         hasNativePeer.set(true);
         printIfDebug("EDT: addNotify");
      }

      requestFocusInWindow();
   }

   @Override
   public void removeNotify()
   {
      synchronized (lock)
      {
         hasNativePeer.set(false);
         printIfDebug("EDT: removeNotify");
      }

      super.removeNotify();
   }

   public boolean checkVisibilityState()
   {
      if (!hasNativePeer.get())
      {
         if (strategy != null)
         {
//          strategy.dispose();
            strategy = null;
            printIfDebug(getClass().getSimpleName() + ": Not visible. Destroy strategy.");
         }

         return false;
      }

      boolean currentShowing = isShowing();
      if (showing.getAndSet(currentShowing) != currentShowing)
      {
         if (currentShowing)
         {
            printIfDebug(getClass().getSimpleName() + ": Enter showing state.");

            for (PBOAwtPanelListener listener : pboAwtPanelListeners)
            {
               listener.isShowing(this);
            }
         }
         else
         {
            printIfDebug(getClass().getSimpleName() + ": Exit showing state.");
         }
      }

      return currentShowing;
   }

   public static void convertScreenShot2(IntBuffer bgraBuf, BufferedImage out)
   {
      WritableRaster writableRaster = out.getRaster();
      DataBufferInt dataBufferInt = (DataBufferInt) writableRaster.getDataBuffer();

      int[] cpuArray = dataBufferInt.getData();

      bgraBuf.clear();
      int width = writableRaster.getWidth();
      int height = writableRaster.getHeight();

      for (int y = height - 1; y >= 0; y--)
      {
         bgraBuf.get(cpuArray, y * width, width);
      }
   }

   public void drawFrameInThread()
   {
      if (frameBuffer == null)
      {
         return;    // Framebuffer is not up yet
      }

      // Select gpuToVram PBO an read pixels from GPU to VRAM
      glBindBuffer(GL_PIXEL_PACK_BUFFER, gpuToVram);
      glReadPixels(0, 0, frameBuffer.getWidth(), frameBuffer.getHeight(), GL_BGRA, GL_UNSIGNED_BYTE, 0);


      // Select vramToSys PBO, bind it to systemRam and copy the data over
      glBindBuffer(GL_PIXEL_PACK_BUFFER, vramToSys);
      ByteBuffer byteBuf = glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY, null);

      // Swap indices
      int previousGpuToVram = gpuToVram;
      gpuToVram = vramToSys;
      vramToSys = previousGpuToVram;


      if (byteBuf == null)
      {
         return;
      }

      convertScreenShot2(byteBuf.asIntBuffer(), bufferedImage);


      // Unmap buffer
      glUnmapBuffer(GL_PIXEL_PACK_BUFFER);

      // Unbind PBO
      glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

      synchronized (lock)
      {
         // All operations on strategy should be synchronized (?)
         if (strategy == null)
         {
            try
            {
               createBufferStrategy(1, new BufferCapabilities(new ImageCapabilities(true), new ImageCapabilities(true),
                       BufferCapabilities.FlipContents.UNDEFINED));
            }
            catch (AWTException ex)
            {
               ex.printStackTrace();
            }

            strategy = getBufferStrategy();
            printIfDebug(getClass().getSimpleName() + ": Visible. Create strategy.");
         }

         // Draw screenshot.
         do
         {
            do
            {
               Graphics2D g2d = (Graphics2D) strategy.getDrawGraphics();
               if (g2d == null)
               {
                  printIfDebug(getClass().getSimpleName() + ": DrawGraphics was null.");

                  return;
               }

               g2d.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_SPEED);

//             g2d.drawImage(img, transformOp, 0, 0);
               g2d.drawImage(bufferedImage, 0, 0, null);
               g2d.dispose();
               strategy.show();
            }
            while (strategy.contentsRestored());
         }
         while (strategy.contentsLost());
      }
   }

   private void printIfDebug(String string)
   {
      if (DEBUG) System.out.println(string);      
   }

   public boolean isActiveDrawing()
   {
      return showing.get();
   }

   public void attachTo(boolean overrideMainFramebuffer, ViewPort... vps)
   {
      if (viewPorts.size() > 0)
      {
         for (ViewPort vp : viewPorts)
         {
            vp.setOutputFrameBuffer(null);
         }

         viewPorts.get(viewPorts.size() - 1).removeProcessor(this);
      }

      viewPorts.addAll(Arrays.asList(vps));
      viewPorts.get(viewPorts.size() - 1).addProcessor(this);

      this.attachAsMain = overrideMainFramebuffer;
   }

   public void initialize(RenderManager renderManager, ViewPort vp)
   {
      if (alreadyClosing) return;

      if (this.renderManager == null)
      {
         // First time called in OGL thread
         this.renderManager = renderManager;

         // Get two PBO buffers, put their indices in buffer
         IntBuffer buffer = BufferUtils.createIntBuffer(2);
         glGenBuffers(buffer);

         gpuToVram = buffer.get(0);
         vramToSys = buffer.get(1);

         reshapeInThread(1, 1);
      }
   }

   private void reshapeInThread(int width, int height)
   {
      if (alreadyClosing) return;

      frameBuffer = new FrameBuffer(width, height, 1);
      frameBuffer.setDepthBuffer(Format.Depth);
      frameBuffer.setColorBuffer(Format.RGB8);

      if (attachAsMain)
      {
         renderManager.getRenderer().setMainFrameBufferOverride(frameBuffer);
      }

      synchronized (lock)
      {
         bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
      }


      dataSize = width * height * 4;


      // Map both PBO buffers, and put the indices in bufferIds
      for (int bufferId : new int[] {gpuToVram, vramToSys})
      {
         glBindBuffer(GL_PIXEL_PACK_BUFFER, bufferId);
         glBufferData(GL_PIXEL_PACK_BUFFER, dataSize, GL_STATIC_READ);
      }

      // Unbind PBO buffers
      glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

      for (ViewPort vp : viewPorts)
      {
         if (!attachAsMain)
         {
            vp.setOutputFrameBuffer(frameBuffer);
         }

         vp.getCamera().resize(width, height, true);

         // NOTE: Hack alert. This is done ONLY for custom framebuffers.
         // Main framebuffer should use RenderManager.notifyReshape().
         for (SceneProcessor sp : vp.getProcessors())
         {
            sp.reshape(vp, width, height);
         }
      }
   }

   public boolean isInitialized()
   {
      return frameBuffer != null;
   }

   public void preFrame(float tpf)
   {
   }

   public void postQueue(RenderQueue rq)
   {
   }

   @Override
   public void invalidate()
   {
      // For "PaintMode.OnDemand" only.
      repaintRequest.set(true);
   }

   public void postFrame(FrameBuffer out)
   {
      if (alreadyClosing) return;
      
      if (!attachAsMain && (out != frameBuffer))
      {
         throw new IllegalStateException("Why did you change the output framebuffer?");
      }

      if (reshapeNeeded.getAndSet(false))
      {
         reshapeInThread(newWidth, newHeight);
      }
      else
      {
         if (!checkVisibilityState())
         {
            return;
         }

         drawFrameInThread();
      }
   }

   public void reshape(ViewPort vp, int w, int h)
   {
   }

   public void cleanup()
   {
   }

   private boolean alreadyClosing = false;
   
   public void closeAndDispose()
   {
      if (alreadyClosing) return;
      alreadyClosing = true;
      
      bufferedImage = null;
      frameBuffer = null;
      renderManager = null;

      if (viewPorts != null)
      {
         viewPorts.clear();
         viewPorts = null;
      }

      if (pboAwtPanelListeners != null)
      {
         pboAwtPanelListeners.clear();
         pboAwtPanelListeners = null;
      }
   }
}
