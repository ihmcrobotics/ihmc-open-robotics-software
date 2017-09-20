package us.ihmc.jMonkeyEngineToolkit.jme.lidar;

import java.nio.FloatBuffer;
import java.util.ArrayList;

import org.lwjgl.opengl.GL11;

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.post.SceneProcessor;
import com.jme3.renderer.Camera;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.scene.Spatial.CullHint;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.Image.Format;
import com.jme3.ui.Picture;
import com.jme3.util.BufferUtils;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.jMonkeyEngineToolkit.GPULidarListener;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;

public class LidarDistortionProcessor implements SceneProcessor
{
   private final ArrayList<GPULidarListener> listeners = new ArrayList<GPULidarListener>();
   private final int scansPerSweep;
   private final int scanHeight;
   
   private final FloatBuffer lidarOutFloatBuffer;
   private final float[] scan;
   
   private RenderManager renderManager;

   private final RigidBodyTransform lidarTransform = new RigidBodyTransform();
   private double time = Double.NaN;

   public LidarDistortionProcessor(JMERenderer jmeRenderer, int scansPerSweep, int scanHeight, int numberOfCameras, float startAngle, float fieldOfView,
                                   LidarSceneViewPort[] lidarSceneProcessors)
   {
      ViewPort viewport = jmeRenderer.getRenderManager().createPostView("LidarDistortionViewport", new Camera(scansPerSweep, scanHeight));
      this.scansPerSweep = scansPerSweep;
      this.scanHeight = scanHeight;
      this.scan = new float[scanHeight * scansPerSweep];
      this.lidarOutFloatBuffer = BufferUtils.createFloatBuffer(scansPerSweep * scanHeight);
      
      FrameBuffer distortionFrameBuffer = new FrameBuffer(scansPerSweep, scanHeight, 1);
      distortionFrameBuffer.setColorBuffer(Format.RGBA32F);

      Material distortionMaterial = createDistortionMaterial(jmeRenderer.getAssetManager(), scansPerSweep, numberOfCameras, startAngle, fieldOfView,
                                       lidarSceneProcessors);

      Picture distortionPicture = new Picture("Distortion");
      distortionPicture.setMaterial(distortionMaterial);
      distortionPicture.setHeight(scanHeight);
      distortionPicture.setWidth(scansPerSweep);
      distortionPicture.setQueueBucket(Bucket.Gui);
      distortionPicture.setCullHint(CullHint.Never);

      viewport.attachScene(distortionPicture);
      viewport.setClearFlags(true, true, true);
      viewport.setOutputFrameBuffer(distortionFrameBuffer);
      viewport.addProcessor(this);
      
      distortionPicture.updateGeometricState();
   }
   
   public void addGPULidarListener(GPULidarListener listener)
   {
      listeners.add(listener);
   }
   
   private static Material createDistortionMaterial(AssetManager assetManager, int scansPerSweep, int numberOfCameras, float startAngle, float fieldOfView, LidarSceneViewPort[] lidarSceneProcessors)
   {
      Material distortionMaterial = new Material(assetManager, "lidar/Distortion.j3md");
      
      for(int i = 0; i < numberOfCameras; i++)
      {
         distortionMaterial.setTexture("tex" + i, lidarSceneProcessors[i].getTexture2D());         
      }
      distortionMaterial.setFloat("startAngle", startAngle);
      distortionMaterial.setFloat("step", fieldOfView);
      distortionMaterial.setFloat("resolution", 1f / (scansPerSweep));
      float camModulo = fieldOfView/numberOfCameras + 1e-7f; //Add a small delta to avoid wrapping around at the last scan line in the modulo calculation
      distortionMaterial.setFloat("camModulo", camModulo);

      float camModuloPreOffset = -startAngle % (fieldOfView/numberOfCameras);
      float camModuloPostOffset = -fieldOfView/(2.0f * numberOfCameras);
      distortionMaterial.setFloat("camModuloPreOffset", camModuloPreOffset);
      distortionMaterial.setFloat("camModuloPostOffset", camModuloPostOffset);
      distortionMaterial.setFloat("oneOverCameras", 1.0f/numberOfCameras);
      
      float cameraAngleToTextureAngle = (float) (1.0 / Math.tan(fieldOfView/(2.0 * numberOfCameras)));;
      distortionMaterial.setFloat("cameraAngleToTextureAngle", cameraAngleToTextureAngle);
      
      
      return distortionMaterial;
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

   @Override
   public void preFrame(float tpf)
   {
   }

   @Override
   public void postQueue(RenderQueue rq)
   {
   }

   @Override
   public void postFrame(FrameBuffer out)
   {
      // renderManager.getRenderer().setFrameBuffer(out); // Probably unnecessary

      lidarOutFloatBuffer.clear();
      GL11.glReadPixels(0, 0, scansPerSweep, scanHeight, GL11.GL_RED, GL11.GL_FLOAT, lidarOutFloatBuffer);
      lidarOutFloatBuffer.rewind();
      
      for (int i = 0; i < scanHeight; i++)
      {
         lidarOutFloatBuffer.get(scan, i * scansPerSweep, scansPerSweep);
      }

      for (GPULidarListener listener : listeners)
      {
         listener.scan(scan, lidarTransform, time);
      }
   }
   
   public void setTransform(RigidBodyTransform transform, double time)
   {
      this.lidarTransform.set(transform);
      this.time = time;
   }

   @Override
   public void cleanup()
   {         
   }
   
}