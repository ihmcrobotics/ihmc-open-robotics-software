package us.ihmc.rdx.simulation.sensors;

import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.glutils.SensorFrameBuffer;
import com.badlogic.gdx.graphics.glutils.SensorFrameBufferBuilder;
import org.lwjgl.opengl.GL41;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.rdx.sceneManager.RDX3DScene;

public abstract class RDXSensorSimulator
{
   // RDX Scene to grab simulated data from
   private final RDX3DScene simulationScene;

   // Sensor intrinsics
   private final CameraIntrinsics sensorIntrinsics;
   private final float minRange;
   private final float maxRange;

   private PerspectiveCamera camera;
   private RigidBodyTransform cameraToWorldTransform;
//   private ScreenViewport viewport;
   private SensorFrameBuffer frameBuffer;

   private Throttler throttler;

   public RDXSensorSimulator(RDX3DScene simulationScene, CameraIntrinsics sensorIntrinsics, float minRange, float maxRange, double fps)
   {
      this.simulationScene = simulationScene;
      this.sensorIntrinsics = sensorIntrinsics;
      this.minRange = minRange;
      this.maxRange = maxRange;

      if (fps > 0.0)
         throttler = new Throttler().setFrequency(fps);

      // Initialize the camera
      float verticalFOV = (float) Math.toDegrees(calculateFOV(sensorIntrinsics.getHeight(), sensorIntrinsics.getFy()));
      camera = new PerspectiveCamera(verticalFOV, sensorIntrinsics.getWidth(), sensorIntrinsics.getHeight());
      camera.near = minRange;
      camera.far = maxRange;

      // Initialize the frame buffer
      SensorFrameBufferBuilder frameBufferBuilder = new SensorFrameBufferBuilder(sensorIntrinsics.getWidth(), sensorIntrinsics.getHeight());
      frameBufferBuilder.addColorTextureAttachment(GL41.GL_RGBA8, GL41.GL_RGBA, GL41.GL_UNSIGNED_BYTE);
   }

   private double calculateFOV(int imageHeight, double focalLength)
   {
      return 2.0 * Math.atan((0.5 * imageHeight) / focalLength);
   }

   public RawImage grabImage()
   {
      if (throttler != null)
         throttler.waitAndRun();



      return null;
   }
}
