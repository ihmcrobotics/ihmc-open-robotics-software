package us.ihmc.jMonkeyEngineToolkit.jme;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

import org.lwjgl.opengl.GL11;

import com.jme3.app.Application;
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.post.SceneProcessor;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.Renderer;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.FrameBuffer.RenderBuffer;
import com.jme3.util.BufferUtils;

public class JMEZBufferGrabber extends AbstractAppState implements SceneProcessor
{
   private Renderer renderer;
   private int width, height;
   private final ViewPort viewport;

   private ByteBuffer outBuf;

   public JMEZBufferGrabber(ViewPort viewport)
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
      reshape(vp, vp.getCamera().getWidth(), vp.getCamera().getHeight());
   }

   @Override
   public boolean isInitialized()
   {
      return super.isInitialized() && renderer != null;
   }

   public void reshape(ViewPort vp, int w, int h)
   {
      outBuf = BufferUtils.createByteBuffer(w * h * 4);
      width = w;
      height = h;
   }

   public void preFrame(float tpf)
   {
   }

   public void postQueue(RenderQueue rq)
   {
   }

   public void postFrame(FrameBuffer out)
   {

      //         renderer.readDepthBuffer(out, outBuf, width, height);

      if(out == null)
      {
         return;
      }
      RenderBuffer rb = out.getDepthBuffer();
      if (rb == null)
      {
         throw new IllegalArgumentException("Specified framebuffer" + " does not have a depthbuffer");
      }

      renderer.setFrameBuffer(out);
      //            context is private: no access
      //            if (context.boundReadBuf != rb.getSlot()) {
      //                glReadBuffer(GL_COLOR_ATTACHMENT0_EXT + rb.getSlot());
      //                context.boundReadBuf = rb.getSlot();
      //            }
      outBuf.clear();
      GL11.glReadPixels(0, 0, width, height, GL11.GL_DEPTH_COMPONENT, GL11.GL_FLOAT, outBuf);
   }

   public void cleanup()
   {
      // TODO Auto-generated method stub

   }

   public int getHeight()
   {
      return height;
   }

   public int getWidth()
   {
      return width;
   }

   public double[][] getZBuffer()
   {
      if(outBuf == null)
      {
         return new double[0][0];
      }

      outBuf.rewind();
      FloatBuffer outBufAsFloatBuffer = outBuf.asFloatBuffer();

      double[][] depthBuffer = new double[width][height];
      int x = 0;
      int y = 0;
      while (outBufAsFloatBuffer.hasRemaining())
      {
         depthBuffer[x][y] = outBufAsFloatBuffer.get();
         x++;
         if (x >= width)
         {
            x = 0;
            y++;
         }
      }

      return depthBuffer;
   }

}