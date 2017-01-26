package us.ihmc.jMonkeyEngineToolkit.jme;

/*
 * Copyright (c) 2009-2012 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import static org.lwjgl.opengl.EXTFramebufferObject.GL_COLOR_ATTACHMENT0_EXT;
import static org.lwjgl.opengl.GL11.GL_UNSIGNED_BYTE;
import static org.lwjgl.opengl.GL11.glReadBuffer;
import static org.lwjgl.opengl.GL11.glReadPixels;
import static org.lwjgl.opengl.GL12.GL_BGRA;
import static org.lwjgl.opengl.GL15.GL_READ_ONLY;
import static org.lwjgl.opengl.GL15.GL_STATIC_READ;
import static org.lwjgl.opengl.GL15.glBindBuffer;
import static org.lwjgl.opengl.GL15.glBufferData;
import static org.lwjgl.opengl.GL15.glGenBuffers;
import static org.lwjgl.opengl.GL15.glMapBuffer;
import static org.lwjgl.opengl.GL15.glUnmapBuffer;
import static org.lwjgl.opengl.GL21.GL_PIXEL_PACK_BUFFER;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.image.WritableRaster;
import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javax.imageio.ImageIO;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraStreamer;
import us.ihmc.jMonkeyEngineToolkit.camera.CaptureDevice;
import us.ihmc.tools.thread.ThreadTools;

import com.jme3.app.Application;
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.post.SceneProcessor;
import com.jme3.renderer.Camera;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.Renderer;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.FrameBuffer.RenderBuffer;
import com.jme3.util.BufferUtils;

/**
 * Transfer data from GPU to CPU quickly by using two Pixel Buffer Objects (PBO's).
 *
 * Steps taken are
 *
 * First copy the data from the GPU framebuffer to PBO "gpuToVram" (async)
 * Copy the data from PBO "vramToSys" to a byteBuffer
 * Swap  gpuToVram and gpuToVram
 *
 * For more information see
 * http://www.songho.ca/opengl/gl_pbo.html
 * http://www.opengl.org/discussion_boards/showthread.php/165780-PBO-glReadPixels-not-so-fast
 *
 * @author jesper
 *
 */
public class JMEFastCaptureDevice extends AbstractAppState implements SceneProcessor, CaptureDevice
{
   private ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("JMEFastCaptureDevice"));
   private final static boolean USE_PBO = JMERenderer.USE_PBO;
   private final static boolean CAPTURE_IMMEDIATLY_AFTER_PREVIOUS_VIDEOFRAME = true;

   private Renderer renderer;
   private RenderManager renderManager;
   private int width, height;
   private ViewPort viewport;

   private BufferedImage bufferedImage;

   private ByteBuffer systemRam;
   private byte[] cpuArray;

   private int bufferId;

   // private int gpuToVram = 0, vramToSys = 1;
   private int dataSize;

   private boolean captureFrame = true;
   private Object syncObject = new Object();
   private Object captureHolder = new Object();

   private CameraStreamer cameraStreamer;
   private Runnable graphicsUpdaterTimerTask;

   public JMEFastCaptureDevice(ViewPort viewport)
   {
      this.viewport = viewport;
   }

   @Override
   public void initialize(AppStateManager stateManager, Application app)
   {
      if (!super.isInitialized())
      {
         viewport.addProcessor(this);
      }

      super.initialize(stateManager, app);
   }

   public void initialize(RenderManager rm, ViewPort vp)
   {
      renderer = rm.getRenderer();
      this.renderManager = rm;
      reshape(vp, vp.getCamera().getWidth(), vp.getCamera().getHeight());

   }

   @Override
   public boolean isInitialized()
   {
      return super.isInitialized() && (renderer != null);
   }

   public void reshape(ViewPort vp, int w, int h)
   {
      dataSize = w * h * 4;
      width = w;
      height = h;
      bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);

      // Create PBO buffer in the system memory to store data from vramToSys in
      systemRam = BufferUtils.createByteBuffer(dataSize);
      cpuArray = new byte[dataSize];

      if (USE_PBO)
      {
         // Get two PBO buffers, put their indices in buffer
         IntBuffer buffer = BufferUtils.createIntBuffer(1);
         glGenBuffers(buffer);

         // Map both PBO buffers, and put the indices in bufferIds
         //       for(int i = 0; i < 2; i++)
         //       {
         bufferId = buffer.get(0);
         glBindBuffer(GL_PIXEL_PACK_BUFFER, bufferId);
         glBufferData(GL_PIXEL_PACK_BUFFER, dataSize, GL_STATIC_READ);

         //       bufferIds = bufferId;
         //         }

         // Unbind PBO buffers
         glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
      }
   }

   public void preFrame(float tpf)
   {
   }

   public void postQueue(RenderQueue rq)
   {
   }

   public void postFrame(FrameBuffer out)
   {
      if (alreadyClosing)
         return;

      synchronized (syncObject)
      {
         if (captureFrame)
         {
            // Setup view
            Camera curCamera = renderManager.getCurrentCamera();
            int viewX = (int) (curCamera.getViewPortLeft() * curCamera.getWidth());
            int viewY = (int) (curCamera.getViewPortBottom() * curCamera.getHeight());
            int viewWidth = (int) ((curCamera.getViewPortRight() - curCamera.getViewPortLeft()) * curCamera.getWidth());
            int viewHeight = (int) ((curCamera.getViewPortTop() - curCamera.getViewPortBottom()) * curCamera.getHeight());
            renderer.setViewPort(0, 0, width, height);

            if (USE_PBO)
            {
               renderer.setFrameBuffer(out);

               if (out != null)
               {
                  RenderBuffer rb = out.getColorBuffer();
                  glReadBuffer(GL_COLOR_ATTACHMENT0_EXT + rb.getSlot());
               }

               // Select gpuToVram PBO an read pixels from GPU to VRAM
               glBindBuffer(GL_PIXEL_PACK_BUFFER, bufferId);
               glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_BYTE, 0);

               // Select vramToSys PBO, bind it to systemRam and copy the data over
               //             glBindBuffer(GL_PIXEL_PACK_BUFFER, bufferIds[vramToSys]);
               systemRam = glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY, null);
               if (systemRam != null)
               {
                  systemRam.get(cpuArray);
               }

               // Unmap buffer
               glUnmapBuffer(GL_PIXEL_PACK_BUFFER);

               // Unbind PBO
               glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
               renderer.setFrameBuffer(null);
            }
            else
            {
               systemRam.clear();
               renderer.readFrameBuffer(out, systemRam);
               if (systemRam != null)
               {
                  systemRam.get(cpuArray);
               }
               systemRam.clear();
            }

            // Return to previous settings
            renderer.setViewPort(viewX, viewY, viewWidth, viewHeight);

            // Swap indices
            //          int previousGpuToVram = gpuToVram;
            //          gpuToVram = vramToSys;
            //          vramToSys = previousGpuToVram;

            // Notify waiting thread

            captureFrame = false;
            syncObject.notifyAll();
         }
      }
   }

   public void cleanup()
   {
      // TODO Auto-generated method stub

   }

   private class GraphicsUpdater implements Runnable
   {
      public long timeStamp = 0;
      public Point3d position = new Point3d();
      public Quat4d orientation = new Quat4d();
      public double fov = Math.PI / 2.0;

      public void run()
      {
         if (alreadyClosing)
            return;

         if (!cameraStreamer.isReadyForNewData())
         {
            return;
         }

         if (!CAPTURE_IMMEDIATLY_AFTER_PREVIOUS_VIDEOFRAME)
         {
            startCaptureOfVideoFrame();
         }

         synchronized (syncObject)
         {
            if (captureFrame)
            {
               try
               {
                  syncObject.wait();
               }
               catch (InterruptedException e)
               {
                  return;
               }
            }
         }

         if (alreadyClosing)
            return;

         convertScreenShot();

         cameraStreamer.updateImage(bufferedImage, timeStamp, position, orientation, fov);

         if (CAPTURE_IMMEDIATLY_AFTER_PREVIOUS_VIDEOFRAME)
         {
            startCaptureOfVideoFrame();
         }

      }

      private void startCaptureOfVideoFrame()
      {
         if (alreadyClosing)
            return;

         synchronized (syncObject)
         {
            captureFrame = true;
            timeStamp = cameraStreamer.getTimeStamp();
            position = cameraStreamer.getCameraPosition();
            orientation = cameraStreamer.getCameraOrientation();
            fov = cameraStreamer.getFieldOfView();
         }
      }
   }

   public void convertScreenShot()
   {
      if (alreadyClosing)
         return;

      WritableRaster wr = bufferedImage.getRaster();
      DataBufferByte db = (DataBufferByte) wr.getDataBuffer();

      int width = wr.getWidth();
      int height = wr.getHeight();

      byte[] imageArray = db.getData();

      // flip the components the way AWT likes them
      for (int y = 0; y < height; y++)
      {
         for (int x = 0; x < width; x++)
         {
            int upperHalfPtrAlpha = (y * width + x) * 4;
            int lowerHalfPtr = ((height - y - 1) * width + x) * 3;

            byte b1 = cpuArray[upperHalfPtrAlpha + 0];
            byte g1 = cpuArray[upperHalfPtrAlpha + 1];
            byte r1 = cpuArray[upperHalfPtrAlpha + 2];
            imageArray[lowerHalfPtr + 0] = b1;
            imageArray[lowerHalfPtr + 1] = g1;
            imageArray[lowerHalfPtr + 2] = r1;
         }
      }
   }

   public BufferedImage exportSnapshotAsBufferedImage()
   {
      if (alreadyClosing)
         return bufferedImage;

      synchronized (captureHolder)
      {
         synchronized (syncObject)
         {
            captureFrame = true;

            do
            {
               try
               {
                  syncObject.wait();
               }
               catch (InterruptedException e)
               {
               }
            }
            while (captureFrame & !alreadyClosing);
         }

         convertScreenShot();

         return bufferedImage;
      }
   }

   public void exportSnapshot(File snapshotFile)
   {
      BufferedImage img = exportSnapshotAsBufferedImage();
      try
      {
         ImageIO.write(img, "png", snapshotFile);
      }
      catch (IOException exp)
      {
         System.out.println("I/O exception!" + exp.toString());
      }
   }

   public int getHeight()
   {
      return height;
   }

   public int getWidth()
   {
      return width;
   }

   public void setSize(int width, int height)
   {
      // Ignore
   }

   public void streamTo(CameraStreamer cameraStreamer, int framesPerSecond)
   {
      if (this.cameraStreamer != null)
      {
         throw new RuntimeException("This capture device is already streaming data");
      }

      this.cameraStreamer = cameraStreamer;

      graphicsUpdaterTimerTask = new GraphicsUpdater();

      executor.scheduleAtFixedRate(graphicsUpdaterTimerTask, 0, 1000000 / framesPerSecond, TimeUnit.MICROSECONDS);
   }

   private boolean alreadyClosing = false;

   public void closeAndDispose()
   {
      if (alreadyClosing)
         return;

      alreadyClosing = true;

      // Wake up the graphics updater task...
      synchronized (syncObject)
      {
         syncObject.notifyAll();
      }
      //      syncObject = null;

      
      executor.shutdown();
      executor = null;
      
      renderer = null;
      renderManager = null;
      viewport = null;

      bufferedImage = null;

      systemRam = null;
      cpuArray = null;

      captureHolder = null;

      cameraStreamer = null;
   }
}
