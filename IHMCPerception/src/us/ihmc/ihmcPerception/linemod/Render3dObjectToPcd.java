package us.ihmc.ihmcPerception.linemod;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import org.lwjgl.opengl.GL11;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.light.AmbientLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.post.SceneProcessor;
import com.jme3.renderer.Camera;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeContext.Type;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.Image.Format;
import com.jme3.texture.Texture2D;
import com.jme3.util.BufferUtils;

public class Render3dObjectToPcd extends SimpleApplication implements SceneProcessor
{

   static final boolean DEBUG = true;
   private static double fovY = Math.PI / 4;
   private Spatial offObject;
   private float angle = 0;

   private FrameBuffer offBuffer;
   private ViewPort offView;
   private Texture2D offTex;
   private Camera offCamera;
   private ImageDisplay display;

   private static final int width = 1024, height = 544;

   private final ByteBuffer cpuBuf = BufferUtils.createByteBuffer(width * height * 4);
   private final byte[] cpuArray = new byte[width * height * 4];
   private final BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_4BYTE_ABGR);
   private final float[][] depthImage = new float[height][];

   private class ImageDisplay extends JPanel
   {

      private long t;
      private long total;
      private int frames;
      private int fps;

      @Override
      public void paintComponent(Graphics gfx)
      {
         super.paintComponent(gfx);
         Graphics2D g2d = (Graphics2D) gfx;

         if (t == 0)
            t = timer.getTime();

         //            g2d.setBackground(Color.BLACK);
         //            g2d.clearRect(0,0,width,height);

         synchronized (image)
         {
            g2d.drawImage(image, null, 0, 0);
         }

         long t2 = timer.getTime();
         long dt = t2 - t;
         total += dt;
         frames++;
         t = t2;

         if (total > 1000)
         {
            fps = frames;
            total = 0;
            frames = 0;
         }

         g2d.setColor(Color.white);
         g2d.drawString("FPS: " + fps, 0, getHeight() - 100);
      }
   }

   public static void main(String[] args)
   {
      Render3dObjectToPcd app = new Render3dObjectToPcd();
      app.setPauseOnLostFocus(false);
      AppSettings settings = new AppSettings(true);
      settings.setResolution(1, 1);
      app.setSettings(settings);
      app.start(Type.OffscreenSurface);
   }

   public void createDisplayFrame()
   {
      SwingUtilities.invokeLater(new Runnable()
      {
         public void run()
         {
            JFrame frame = new JFrame("Render Display");
            display = new ImageDisplay();
            display.setPreferredSize(new Dimension(width, height));
            frame.getContentPane().add(display);
            frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
            frame.addWindowListener(new WindowAdapter()
            {
               public void windowClosed(WindowEvent e)
               {
                  stop();
               }
            });
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setResizable(false);
            frame.setVisible(true);
         }
      });
   }

   public void updateImageContents()
   {
      cpuBuf.clear();
      renderer.readFrameBuffer(offBuffer, cpuBuf);

      synchronized (image)
      {
         //         Screenshots.convertScreenShot(cpuBuf, image);
         TestRenderToMemory.convertScreenShot(cpuBuf, image);
      }

      if (display != null)
         display.repaint();
   }

   public void setupOffscreenView()
   {
      offCamera = new Camera(width, height);

      // create a pre-view. a view that is rendered before the main view
      offView = renderManager.createPreView("Offscreen View", offCamera);
      offView.setBackgroundColor(ColorRGBA.DarkGray);
      offView.setClearFlags(true, true, true);

      // this will let us know when the scene has been rendered to the 
      // frame buffer
      offView.addProcessor(this);

      // create offscreen framebuffer
      offBuffer = new FrameBuffer(width, height, 1);

      //setup framebuffer's cam
      offCamera.setFrustumPerspective((float) (fovY * 180 / Math.PI), (float) width / (float) height, 1f, 1000f);

      offCamera.setLocation(new Vector3f(0f, 0f, -1.2f));
      offCamera.lookAt(new Vector3f(0f, 0f, 0f), Vector3f.UNIT_Y);

      //setup framebuffer's texture
      //        offTex = new Texture2D(width, height, Format.RGBA8);

      //setup framebuffer to use renderbuffer
      // this is faster for gpu -> cpu copies
      offBuffer.setDepthBuffer(Format.Depth);
      offBuffer.setColorBuffer(Format.RGBA8);
      //        offBuffer.setColorTexture(offTex);

      //set viewport to render to offscreen framebuffer
      offView.setOutputFrameBuffer(offBuffer);

      // setup framebuffer's scene
      if (false)
      {
         Box boxMesh = new Box(Vector3f.ZERO, .1f, .1f, .1f);
         Material material = assetManager.loadMaterial("Interface/Logo/Logo.j3m");
         offObject = new Geometry("box", boxMesh);
         offObject.setMaterial(material);
      }
      else
      {
         assetManager.registerLocator("/examples/drill", ClasspathLocator.class);
         offObject = assetManager.loadModel("drill.obj");
         AmbientLight al = new AmbientLight();
         al.setColor(ColorRGBA.White.mult(1.0f));
         offObject.addLight(al);
      }

      // attach the scene to the viewport to be rendered
      offView.attachScene(offObject);

   }

   @Override
   public void simpleInitApp()
   {
      setupOffscreenView();
      createDisplayFrame();
   }

   public void readDepthBuffer()
   {

      ByteBuffer depthBuf = BufferUtils.createByteBuffer(width * height * 4);
      GL11.glReadPixels(0, 0, width, height, GL11.GL_DEPTH_COMPONENT, GL11.GL_FLOAT, depthBuf);

      FloatBuffer fb = depthBuf.asFloatBuffer();
      for (int i = 0; i < depthImage.length; i++)
      {
         depthImage[i] = new float[width];
         fb.get(depthImage[i]);
      }

      for (int h = 0; h < height; h++)
         for (int w = 0; w < width; w++)
         {
            if (depthImage[h][w] == 0 || depthImage[h][w] == 1)
            {
               depthImage[h][w] = Float.NaN;
            }
            else
            {
               depthImage[h][w] = (float) (1 / (2 - 2.0 * depthImage[h][w]));
               //               depthImage[h][w] = 1 / (1 - depthImage[h][w]);
               //               depthImage[h][w] = zDeviceToZEye(depthImage[h][w]);
               //               System.out.println("w" + w + "h" + h + " " + depthImage[h][w]);
            }
         }

   }

   private static void writePcd(BufferedImage image, float[][] depthImage, Camera camera)
   {

      float f = (float) (height / 2.0 / Math.tan(fovY / 2.0));
      if (DEBUG)
      {
         System.out.println("view matrix");
         System.out.println(camera.getViewMatrix());
         System.out.println("proj matrix");
         System.out.println(camera.getProjectionMatrix());
         System.out.println("focalLength=" + f);
      }
      try
      {
         PrintWriter writer = new PrintWriter(new File("object.pcd"));
         writer.println("# .PCD v0.7 - Point Cloud Data file format");
         writer.println("VERSION 0.7");
         writer.println("FIELDS x y z rgba");
         writer.println("SIZE 4 4 4 4");
         writer.println("TYPE F F F U");
         writer.println("COUNT 1 1 1 1");
         writer.println("WIDTH " + width);
         writer.println("HEIGHT " + height);
         writer.println("VIEWPOINT 0 0 -1 1 0 0 0");
         writer.println("POINTS " + width * height);
         writer.println("DATA ascii");
         for (int h = 0; h < height; h++)
            for (int w = 0; w < width; w++)
            {
               float x = 0, y = 0, z = 0;
               if (Float.isNaN(depthImage[h][w]))
               {
                  z = 2;//Float.NaN;
                  x = (float) ((w - width / 2) * z / f);
                  y = (float) ((h - height / 2) * z / f);
               }
               else
               {
                  z = depthImage[h][w];
                  x = (float) ((w - width / 2) * z / f);
                  y = (float) ((h - height / 2) * z / f);
               }
               writer.print((x + " " + y + " " + (z) + " ").toLowerCase());

               int c = image.getRGB(w, h);
               writer.println(Integer.toUnsignedLong(c));
               //               writer.println(c);
            }

         writer.close();
      }
      catch (FileNotFoundException e)
      {
         System.out.println("file open failed");
         e.printStackTrace();
      }
   }

   private static void writePng(float[][] depthImage)
   {

      int width = depthImage[0].length;
      int height = depthImage.length;
      //find min max
      float maxDepth = Float.NEGATIVE_INFINITY, minDepth = Float.POSITIVE_INFINITY;
      for (int h = 0; h < height; h++)
         for (int w = 0; w < width; w++)
         {
            float depth = depthImage[h][w];
            if (depth < minDepth)
               minDepth = depth;
            if (depth > maxDepth)
               maxDepth = depth;
         }

      //fill image
      BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
      int rgb;
      for (int h = 0; h < height; h++)
      {
         for (int w = 0; w < width; w++)
         {
            float depth = depthImage[h][w];
            int grayVal = (int) (255 * (depth - minDepth) / (maxDepth - minDepth));

            if (Float.isNaN(depth))
            {
               grayVal = 0;
            }
            try
            {
               image.setRGB(w, h, new Color(grayVal, grayVal, grayVal, 255).getRGB());
            }
            catch (IllegalArgumentException e)
            {
               System.out.println("Illegal Depth - max " + maxDepth + " min " + minDepth + " depth " + depth + " grayVal " + grayVal);
            }
         }
      }
      File imageOutput = new File("depth.png");
      try
      {
         ImageIO.write(image, "png", imageOutput);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public void simpleUpdate(float tpf)
   {
      Quaternion q = new Quaternion();
      angle += tpf;
      angle %= FastMath.TWO_PI;
      //      q.fromAngles(angle, 0, angle);
      q.fromAngles((float) (Math.PI / 2), 0, 0);

      offObject.setLocalRotation(q);
      offObject.updateLogicalState(tpf);
      offObject.updateGeometricState();
   }

   public void initialize(RenderManager rm, ViewPort vp)
   {
   }

   public void reshape(ViewPort vp, int w, int h)
   {
   }

   public boolean isInitialized()
   {
      return true;
   }

   public void preFrame(float tpf)
   {
   }

   public void postQueue(RenderQueue rq)
   {
   }

   /**
    * Update the CPU image's contents after the scene has
    * been rendered to the framebuffer.
    */
   public void postFrame(FrameBuffer out)
   {
      updateImageContents();
      readDepthBuffer();
      writePng(depthImage);
      try
      {
         ImageIO.write(image, "png", new File("rgb.png"));
      }
      catch (IOException e)
      {
         System.out.println("rgb.png output failed");
         e.printStackTrace();
      }
      writePcd(image, depthImage, offCamera);
      stop();
   }

   public void cleanup()
   {
   }

}
