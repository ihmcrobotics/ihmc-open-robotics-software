package us.ihmc.ihmcPerception.linemod;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.imageio.ImageIO;

import org.lwjgl.opengl.GL11;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.light.AmbientLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix4f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.post.SceneProcessor;
import com.jme3.renderer.Camera;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.debug.Arrow;
import com.jme3.scene.shape.Box;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeContext.Type;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.Image.Format;
import com.jme3.util.BufferUtils;

public class Render3dObject extends SimpleApplication implements SceneProcessor
{

   static boolean DEBUG = false;
   private static double fovY = 0.90;//Math.PI / 4; //0.90 is closer to multisense SL
   private Spatial offObject;

   private FrameBuffer offBuffer;
   private ViewPort offView;
   private Camera offCamera;

   private static final int width = 544, height = 544;
//   private static final int width = 1024, height = 544;

   private final ByteBuffer cpuBuf = BufferUtils.createByteBuffer(width * height * 4);
   
   private final BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_4BYTE_ABGR);
   private final float[][] depthImage = new float[height][];
   private Object syncObj = new Object();

   private Transform objectTransform = new Transform();
   private AtomicBoolean objectTransformUpdated = new AtomicBoolean();
   private AtomicBoolean imageRendered = new AtomicBoolean();

   private final File model;

   public static void main(String[] args)
   {
//      Render3dObject app = new Render3dObject(new File("/examples/drill/drill.obj"));
      Render3dObject app = new Render3dObject(new File("drill_DCS551/drillUI.obj"));
      app.DEBUG=true;

      //rotation in camera frame - Xright Ydown Zforward
      float angle = 0;
      for (int i = 0; i < 1; i++)
      {
         angle = (i * FastMath.TWO_PI / 8) % FastMath.TWO_PI;
         app.renderImage(angle, 0.0f, 0.0f, +1.0f);
         Render3dObject.writeDepthImage(app.getDepthImage(), "test_depth" + i + ".png");
         Render3dObject.writeImage(app.getRGBImage(), "test_rgb" + i + ".png");
         Render3dObject.writePcd(app.getPointcloud().xyzrgb, app.getCamera(), "test_cloud" + i + ".pcd");
      }

   }

   public BufferedImage getRGBImage()
   {
      return image;
   }

   public float[][] getDepthImage()
   {
      return depthImage;
   }

   public Camera getCamera()
   {
      return offCamera;
   }

   public Render3dObject(File model)
   {
      Logger.getLogger("com.jme3").setLevel(Level.SEVERE);
      this.model = model;
      setPauseOnLostFocus(false);
      AppSettings settings = new AppSettings(true);
      settings.setAudioRenderer(null);
      settings.setResolution(1, 1);
      setSettings(settings);
//      start(Type.OffscreenSurface);
   }

   public void renderImage(float yaw, float pitch, float roll, float distance)
   {
      final Quaternion qX = new Quaternion();
      qX.fromAngles(FastMath.PI/2, 0.0f, 0.0f);
      Quaternion qYaw = new Quaternion();
      qYaw.fromAngles(0.0f, 0.0f, yaw);
      Quaternion qPitch = new Quaternion();
      qPitch.fromAngles(0.0f, pitch, 0.0f);
      Quaternion qRoll = new Quaternion();
      qRoll.fromAngles(roll, 0.0f, 0.0f);
      
      Transform transform = new Transform();
      transform.setRotation(qX.mult(qYaw).mult(qPitch).mult(qRoll));
      transform.setTranslation(0.0f, 0.0f, distance);
      renderImage(transform);
      
   }
   public void renderImage(Transform transform)
   {
      start(Type.OffscreenSurface);
      this.objectTransform.set(transform);
      objectTransformUpdated.set(false);
      imageRendered.set(false);
      synchronized (syncObj)
      {
         try
         {
            syncObj.wait();
         }
         catch (InterruptedException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }
      }
      stop(true);
   }

   public void setupOffscreenView()
   {
      offCamera = new Camera(width, height);

      // create a pre-view. a view that is rendered before the main view
      offView = renderManager.createPreView("Offscreen View", offCamera);
      offView.setBackgroundColor(ColorRGBA.Black);
      offView.setClearFlags(true, true, true);

      // this will let us know when the scene has been rendered to the 
      // frame buffer
      offView.addProcessor(this);

      // create offscreen framebuffer
      offBuffer = new FrameBuffer(width, height, 1);

      //setup framebuffer's cam
      offCamera.setFrustumPerspective((float) (fovY * 180 / Math.PI), (float) width / (float) height, 0.1f, 2f);

      offCamera.setLocation(new Vector3f(0f, 0f, 0f));
      offCamera.lookAt(new Vector3f(0f, 0f, 1.0f), Vector3f.UNIT_Y.negate());

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
         assetManager.registerLocator(model.getParent(), ClasspathLocator.class);
         offObject = assetManager.loadModel(model.getName());
         AmbientLight al = new AmbientLight();
         al.setColor(ColorRGBA.White.mult(1.2f));
         offObject.addLight(al);
      }

      // attach the scene to the viewport to be rendered
      rootNode.attachChild(offObject);
      if(DEBUG)
      {
         attachCoordinateAxes(rootNode,new Vector3f());
      }
      offView.attachScene(rootNode);

   }
   
   private void attachCoordinateAxes(Node node, Vector3f pos){
      Arrow arrow = new Arrow(Vector3f.UNIT_X);
      arrow.setLineWidth(4); // make arrow thicker
      putShape(node, arrow, ColorRGBA.Red).setLocalTranslation(pos);
     
      arrow = new Arrow(Vector3f.UNIT_Y);
      arrow.setLineWidth(4); // make arrow thicker
      putShape(node, arrow, ColorRGBA.Green).setLocalTranslation(pos);
     
      arrow = new Arrow(Vector3f.UNIT_Z);
      arrow.setLineWidth(4); // make arrow thicker
      putShape(node, arrow, ColorRGBA.Blue).setLocalTranslation(pos);
    }
     
    private Geometry putShape(Node node, Mesh shape, ColorRGBA color){
      Geometry g = new Geometry("coordinate axis", shape);
      Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
      mat.getAdditionalRenderState().setWireframe(true);
      mat.setColor("Color", color);
      g.setMaterial(mat);
      g.rotate(0.0f,0.0f,0.0f);
      g.scale(0.25f);
      node.attachChild(g);
      return g;
    }

    
   @Override
   public void simpleInitApp()
   {
         setupOffscreenView();
   }

   @Override
   public void simpleUpdate(float tpf)
   {
//      offObject.setLocalTransform(objectTransform);
//      offObject.updateLogicalState(tpf);
//      offObject.updateGeometricState();
      rootNode.setLocalTransform(objectTransform);
      rootNode.updateLogicalState(tpf);
      rootNode.updateGeometricState();
      objectTransformUpdated.set(true);
   }

   // Called after FrameBuffer is rendered
   @Override
   public void postFrame(FrameBuffer out)
   {
      if (!objectTransformUpdated.get() || imageRendered.get())
      {
         return;
      }
      imageRendered.set(true);
      cpuBuf.clear();
      renderer.readFrameBuffer(offBuffer, cpuBuf);
      //         Screenshots.convertScreenShot(cpuBuf, image);
      readDepthBuffer(depthImage);
      TestRenderToMemory.convertScreenShot(cpuBuf, image);

      synchronized (syncObj)
      {
         syncObj.notifyAll();
      }

      if (DEBUG)
      {
         String prefix = "renderDebug";
         writeImage(image, prefix + "_image.png");

         writeDepthImage(depthImage, prefix + "_depth.png");

         writePcd(getPointcloud().xyzrgb, offCamera, prefix + "_cloud.pcd");
         System.out.println("prefix=" + prefix);
      }

   }

   //Static Methods
   private void readDepthBuffer(float[][] outDepthImage)
   {

      ByteBuffer depthBuf = BufferUtils.createByteBuffer(width * height * 4);
      GL11.glReadPixels(0, 0, width, height, GL11.GL_DEPTH_COMPONENT, GL11.GL_FLOAT, depthBuf); //output [0 1];

      FloatBuffer fb = depthBuf.asFloatBuffer();
      int heigh = outDepthImage.length;
      for (int i = 0; i < heigh; i++)
      {
         outDepthImage[height-i-1] = new float[width];
         fb.get(outDepthImage[height-i-1]);
      }
      
      Matrix4f projectionMatrix = offCamera.getProjectionMatrix();
      float A = projectionMatrix.m22;
      float B = projectionMatrix.m23;
      for (int h = 0; h < height; h++)
         for (int w = 0; w < width; w++)
         {
            if (outDepthImage[h][w] == 0 || outDepthImage[h][w] == 1)
            {
               outDepthImage[h][w] = Float.NaN;
            }
            else
            {
                  float rawDepth=outDepthImage[h][w];
                  rawDepth = 2*rawDepth-1;
//               outDepthImage[h][w] = (float) (1 / (outDepthImage[h][w])-1);
               outDepthImage[h][w] = (float) (B / (A + rawDepth));
               //               depthImage[h][w] = 1 / (1 - depthImage[h][w]);
               //               depthImage[h][w] = zDeviceToZEye(depthImage[h][w]);
               //               System.out.println("w" + w + "h" + h + " " + depthImage[h][w]);
            }
         }
   }

   public OrganizedPointCloud getPointcloud()
   {
      float xyzrgb[] = new float[width * height * 4];
      int ptr = 0;
      float n = offCamera.getFrustumNear();
      float frustrumWidthToImageWidth = (offCamera.getFrustumRight() - offCamera.getFrustumLeft()) / width;
      float frustrumHeightToImageHeight = (offCamera.getFrustumTop() - offCamera.getFrustumBottom()) / height;
      for (int h = 0; h < height; h++)
         for (int w = 0; w < width; w++)
         {
            float x = 0, y = 0, z = 0;
            if (Float.isNaN(depthImage[h][w]))
            {
               z = Float.NaN;//Float.NaN;
               x = (float) ((w - width / 2) * frustrumWidthToImageWidth * z / n);
               y = (float) ((h - height / 2) * frustrumHeightToImageHeight * z / n);
            }
            else
            {
               z = depthImage[h][w];
               x = (float) ((w - width / 2) * frustrumWidthToImageWidth * z / n);
               y = (float) ((h - height / 2) * frustrumHeightToImageHeight * z / n);
            }

            xyzrgb[ptr++] = x;
            xyzrgb[ptr++] = y;
            xyzrgb[ptr++] = z;
            xyzrgb[ptr++] = Float.intBitsToFloat(image.getRGB(w, h));
         }

      return new OrganizedPointCloud(width, height, xyzrgb);
   }

   private static void writePcd(float xyzrgb[], Camera camera, String fileName)
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
         PrintWriter writer = new PrintWriter(new File(fileName));
         writer.println("# .PCD v0.7 - Point Cloud Data file format");
         writer.println("VERSION 0.7");
         writer.println("FIELDS x y z rgba");
         writer.println("SIZE 4 4 4 4");
         writer.println("TYPE F F F U");
         writer.println("COUNT 1 1 1 1");
         writer.println("WIDTH " + width);
         writer.println("HEIGHT " + height);
         writer.println("VIEWPOINT 0 0 0 1 0 0 0");
         writer.println("POINTS " + width * height);
         writer.println("DATA ascii");
         int counter=0;
         for (int h = 0; h < height; h++)
            for (int w = 0; w < width; w++)
            {
               float x = xyzrgb[counter++];
               float y = xyzrgb[counter++];
               float z = xyzrgb[counter++];
               float rgb = xyzrgb[counter++];
               writer.print((x + " " + y + " " + (z) + " ").toLowerCase());
               writer.println(Float.floatToRawIntBits(rgb));
            }

         writer.close();
      }
      catch (FileNotFoundException e)
      {
         System.out.println("file open failed");
         e.printStackTrace();
      }
   }

   private static void writeDepthImage(float[][] depthImage, String fileName)
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
      if(DEBUG)
      {
               System.out.println("Overal Depth - max " + maxDepth + " min " + minDepth);
      }

      //fill image
      BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
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
      File imageOutput = new File(fileName);
      try
      {
         ImageIO.write(image, "png", imageOutput);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private static void writeImage(BufferedImage image, String fileName)
   {

      try
      {
         ImageIO.write(image, "png", new File(fileName));
      }
      catch (IOException e)
      {
         System.out.println("rgb.png output failed");
         e.printStackTrace();
      }
   }

   @Override
   public void initialize(RenderManager rm, ViewPort vp)
   {
   }

   @Override
   public void reshape(ViewPort vp, int w, int h)
   {
   }

   @Override
   public boolean isInitialized()
   {
      return true;
   }

   @Override
   public void preFrame(float tpf)
   {
   }

   @Override
   public void postQueue(RenderQueue rq)
   {
   }

   @Override
   public void cleanup()
   {
   }
   

}
