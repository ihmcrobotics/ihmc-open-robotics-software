package us.ihmc.jMonkeyEngineToolkit.jme.lidar;

import java.util.concurrent.Callable;

import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;
import us.ihmc.tools.time.Timer;

import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.post.SceneProcessor;
import com.jme3.renderer.Camera;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.Image.Format;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture.MagFilter;
import com.jme3.texture.Texture.MinFilter;
import com.jme3.texture.Texture2D;

public class LidarSceneViewPort implements SceneProcessor
{
   private static final int TEXTURE_TO_SCAN_INTERPOLATION_FACTOR = 4;

   private final Camera cam;
   private final Texture2D sceneTexture;
   private final LidarMaterial lidarMaterial; // Pass in lidarMaterial to reuse between scene viewports

   private JMERenderer jmeRenderer;
   private RenderManager renderManager;
   private ViewPort viewPort;
   
   public LidarSceneViewPort(JMERenderer jmeRenderer, LidarMaterial lidarMaterial, int numberOfCameras, int scansPerSweep, int scanHeight, float fieldOfView,
                             float minRange, float maxRange)
   {
      this.jmeRenderer = jmeRenderer;
      int cameraScansPerSweep = scansPerSweep / numberOfCameras;
      this.lidarMaterial = lidarMaterial;
      this.cam = new Camera(cameraScansPerSweep * TEXTURE_TO_SCAN_INTERPOLATION_FACTOR, scanHeight);
      viewPort = jmeRenderer.getRenderManager().createMainView("LidarViewPort", cam);

      setupCamera(numberOfCameras, cameraScansPerSweep, scanHeight, fieldOfView, minRange, maxRange);

      sceneTexture = new Texture2D(cameraScansPerSweep * TEXTURE_TO_SCAN_INTERPOLATION_FACTOR, scanHeight, Format.RGB32F);
      // Use Nearest Neighbour interpolation to avoid intermediate points between objects that are in front of each other
      sceneTexture.setMagFilter(MagFilter.Nearest); // Magnification set to nearest neighbor
      sceneTexture.setMinFilter(MinFilter.NearestNoMipMaps); // Minification set to nearest neighbor no mipmaps

      FrameBuffer sceneFrameBuffer = new FrameBuffer(cameraScansPerSweep * TEXTURE_TO_SCAN_INTERPOLATION_FACTOR, scanHeight, 1);
      sceneFrameBuffer.setDepthBuffer(Format.Depth24);
      sceneFrameBuffer.setColorTexture(sceneTexture);

      viewPort.setClearFlags(true, true, true);
      viewPort.setOutputFrameBuffer(sceneFrameBuffer);
      viewPort.addProcessor(this);
   }
   
   public void updateScene(final Node sceneToUse)
   {
      jmeRenderer.enqueue(new Callable<Object>()
      {
         @Override
         public Object call() throws Exception
         {
            viewPort.clearScenes();
            viewPort.attachScene(sceneToUse);
            
            return null;
         }
      });
   }

   public void setRotation(Quaternion rotation)
   {
      cam.setRotation(rotation);
   }

   public void setLocation(Vector3f location)
   {
      cam.setLocation(location);
   }

   private void setupCamera(int numberOfCameras, int cameraScansPerSweep, int scanHeight, float fieldOfView, float minRange, float maxRange)
   {
      float cameraFoV = fieldOfView / (numberOfCameras);

      float frustrumX = FastMath.tan(cameraFoV / 2.0f) * minRange;
      float frustrumY = frustrumX / cameraScansPerSweep * scanHeight;

      cam.setParallelProjection(false);
      cam.setFrustum(minRange, maxRange, -frustrumX, frustrumX, frustrumY, -frustrumY);
   }

   @Override
   public void initialize(RenderManager renderManager, ViewPort vp)
   {
      this.renderManager = renderManager;
   }

   @Override
   public void reshape(ViewPort vp, int w, int h)
   {
      throw new RuntimeException("Cannot resize LIDAR!");
   }

   @Override
   public boolean isInitialized()
   {
      return renderManager != null;
   }

   // Debugging values used to measure performance
   Timer timer = new Timer().start();
   long frameNum = 0;
   
   @Override
   public void preFrame(float tpf)
   {
      if (JMERenderer.USE_GPU_LIDAR_PARALLEL_SCENE)
      {
         if (!viewPort.getScenes().isEmpty())
         {
            timer.resetLap();
            jmeRenderer.syncSubsceneToMain((Node) viewPort.getScenes().get(0));
            timer.lap();

            if (JMERenderer.DEBUG_GPU_LIDAR_PARALLEL_SCENE)
            {
               if (frameNum++ % 50 == 0)
                  System.out.println("[GPULidar] Average scene sync: " + timer.averageLap() + " (s)");
            }
         }
      }
      
      renderManager.setForcedMaterial(lidarMaterial);
   }

   @Override
   public void postQueue(RenderQueue rq)
   {

   }

   @Override
   public void postFrame(FrameBuffer out)
   {
      renderManager.setForcedMaterial(null);
   }

   @Override
   public void cleanup()
   {

   }

   public Texture getTexture2D()
   {
      return sceneTexture;
   }

}
