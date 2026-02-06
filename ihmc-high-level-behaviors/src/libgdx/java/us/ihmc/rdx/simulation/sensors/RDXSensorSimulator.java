package us.ihmc.rdx.simulation.sensors;

import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.Pixmap.Format;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer.FrameBufferBuilder;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import org.bytedeco.javacpp.PointerScope;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatExpr;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.sensors.CameraIntrinsics;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;

import java.time.Instant;
import java.util.concurrent.atomic.AtomicBoolean;

/*
 * TODO: use glsl to linearize depth + add noise
 */
public class RDXSensorSimulator
{
   private static final double METER_TO_MILLIMETERS = 1000.0;

   // RDX Scene to grab simulated data from
   private RDX3DScene simulationScene;

   // Sensor intrinsics
   private final CameraIntrinsics cameraIntrinsics;
   private final int imageWidth;
   private final int imageHeight;
   private final float verticalFOV;
   private final float minRange;
   private final float maxRange;

   // Simulated stuff
   private PerspectiveCamera camera;
   private final FramePose3D cameraPose = new FramePose3D();
   private final Matrix4 cameraTransform = new Matrix4();

   private ScreenViewport viewport;
   private FrameBuffer frameBuffer;

   // Images
   private Mat colorImage = null;
   private Mat colorData = null;
   private final AtomicBoolean colorUpToDate = new AtomicBoolean(false);

   private Mat depthImage = null;
   private Mat depthData = null;
   private Mat floatDepthData = null;
   private Mat shortDepthData = null;
   private final AtomicBoolean depthUpToDate = new AtomicBoolean(false);

   // Noise stuff
   private Mat noise = null;
   private final Mat noiseZero;
   private final Mat noiseAmplitude;

   private Instant grabTime = Instant.EPOCH;
   private long sequenceNumber = 0L;

   public RDXSensorSimulator(int imageWidth, int imageHeight, float verticalFOV, float minRange, float maxRange, int noiseAmount)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.verticalFOV = verticalFOV;
      this.minRange = minRange;
      this.maxRange = maxRange;

      double focalLength = calculateFocalLength(imageHeight, verticalFOV);
      cameraIntrinsics = new CameraIntrinsics(imageHeight, imageWidth, focalLength, focalLength, 0.5 * imageWidth, 0.5 * imageHeight);

      Scalar zero = new Scalar(0);
      Scalar amount = new Scalar(noiseAmount);
      noiseZero = new Mat(1, 1, opencv_core.CV_16UC1, zero);
      noiseAmplitude = new Mat(1, 1, opencv_core.CV_16UC1, amount);
      zero.close();
      amount.close();
   }

   private double calculateFocalLength(int imageHeight, float verticalFOV)
   {
      return (0.5 * imageHeight) / Math.tan(Math.toRadians(0.5 * verticalFOV));
   }

   public void create(RDX3DScene scene)
   {
      simulationScene = scene;

      // Initialize the camera
      camera = new PerspectiveCamera(verticalFOV, imageWidth, imageHeight);
      camera.near = minRange;
      camera.far = maxRange;
      viewport = new ScreenViewport(camera);

      // Initialize the frame buffer
      FrameBufferBuilder frameBufferBuilder = new FrameBufferBuilder(imageWidth, imageHeight);
      frameBufferBuilder.addBasicColorTextureAttachment(Format.RGBA8888);
      frameBufferBuilder.addDepthTextureAttachment(GL41.GL_DEPTH_COMPONENT32F, GL41.GL_FLOAT);
      frameBuffer = frameBufferBuilder.build();
   }

   public void enableColor(boolean enable)
   {
      if (enable)
      {
         if (colorImage == null)
         {
            colorImage = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4);
            colorData = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4);
         }
      }
      else
      {
         if (colorImage != null)
         {
            colorImage.close();
            colorData.close();
         }
         colorImage = null;
         colorData = null;
      }
   }

   public void enableDepth(boolean enable)
   {
      if (enable)
      {
         if (depthImage == null)
         {
            depthImage = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1);
            depthData = new Mat(imageHeight, imageWidth, opencv_core.CV_32FC1);
            floatDepthData = new Mat(imageHeight, imageWidth, opencv_core.CV_32FC1);
            shortDepthData = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1);
            noise = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1, new Scalar(0.0));
         }
      }
      else
      {
         if (depthImage != null)
         {
            depthImage.close();
            depthData.close();
            floatDepthData.close();
            shortDepthData.close();
            noise.close();
         }
         depthImage = null;
         depthData = null;
         floatDepthData = null;
         shortDepthData = null;
         noise = null;
      }
   }

   public void render(RigidBodyTransform cameraTransformToWorld)
   {
      if (colorImage == null && depthImage == null) // Ensure we actually want to render
         return;

      // Update camera pose
      LibGDXTools.toLibGDX(cameraTransformToWorld, cameraTransform);
      camera.position.setZero();
      camera.up.set(0.0f, 0.0f, 1.0f);
      camera.direction.set(1.0f, 0.0f, 0.0f);
      camera.transform(cameraTransform);

      cameraPose.set(cameraTransformToWorld);

      frameBuffer.begin();

      // Set RGBA values to R = 0.0, G = 0.0, B = 0.0, A = 1.0 when cleared
      GL41.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
      // Perform the clear
      GL41.glClear(GL41.GL_COLOR_BUFFER_BIT | GL41.GL_DEPTH_BUFFER_BIT);

      // Update viewport to match camera pose
      viewport.update(imageWidth, imageHeight);

      // Render and "grab" the images
      simulationScene.preRender(camera);
      GL41.glViewport(0, 0, imageWidth, imageHeight);
      simulationScene.render(RDXSceneLevel.GROUND_TRUTH.SINGLETON_SET);
      simulationScene.postRender();

      // Save the grab time
      grabTime = Instant.now();

      if (colorImage != null)
      {
         // Read the grabbed color image
         GL41.glReadBuffer(GL41.GL_COLOR_ATTACHMENT0);

         synchronized (colorUpToDate)
         {
            // Put the data into colorData
            GL41.glReadPixels(0, 0, imageWidth, imageHeight, GL41.GL_RGBA, GL41.GL_UNSIGNED_BYTE, colorData.data().address());
            colorUpToDate.set(false);
         }
      }

      if (depthImage != null)
      {
         synchronized (depthUpToDate)
         {
            // Read the grabbed depth image
            GL41.glReadPixels(0, 0, imageWidth, imageHeight, GL41.GL_DEPTH_COMPONENT, GL41.GL_FLOAT, depthData.data().address());
            depthUpToDate.set(false);
         }
      }

      frameBuffer.end();

      ++sequenceNumber;
   }

   public boolean hasColorImage()
   {
      return colorImage != null;
   }

   public RawImage getColorImage()
   {
      if (colorImage == null)
         throw new IllegalStateException("No color image has been rendered.");

      updateColor();
      return new RawImage(colorImage.clone(), null, PixelFormat.RGBA8, cameraIntrinsics, CameraModel.PINHOLE, cameraPose, grabTime, sequenceNumber, 0.0f);
   }

   private void updateColor()
   {
      if (colorUpToDate.getAndSet(true))
         return;

      synchronized (colorUpToDate)
      {
         // libGDX renders this stuff upside down, so the image must be flipped
         opencv_core.flip(colorData, colorImage, OpenCVTools.FLIP_Y);
      }
   }

   public boolean hasDepthImage()
   {
      return depthImage != null;
   }

   public RawImage getDepthImage()
   {
      if (depthImage == null)
         throw new IllegalStateException("No depth image has been rendered.");

      updateDepth();
      return new RawImage(depthImage.clone(), null, PixelFormat.GRAY16, cameraIntrinsics, CameraModel.PINHOLE, cameraPose, grabTime, sequenceNumber, 0.001f);
   }

   private void updateDepth()
   {
      if (depthUpToDate.getAndSet(true))
         return;

      // Linearizing Depth Buffer Data: https://learnopengl.com/Advanced-OpenGL/Depth-testing
      synchronized (depthUpToDate)
      {
         // transform depth values to "normalized device coordinates"
         float viewRange = camera.far - camera.near;
         depthData.convertTo(floatDepthData, opencv_core.CV_32FC1, 2.0 * viewRange, -viewRange);
      }

      PointerScope pointerScope = new PointerScope();

      // use normalized device coordinates to get linear depth
      MatExpr linearizedDepth = opencv_core.subtract(new Scalar(camera.near + camera.far), floatDepthData);
      linearizedDepth = opencv_core.divide(2.0 * camera.near * camera.far, linearizedDepth);

      // Convert float32 in meters to uint16 in millimeters
      linearizedDepth.asMat().convertTo(shortDepthData, opencv_core.CV_16UC1, METER_TO_MILLIMETERS, 0.0);

      // Remove depth values at max range
      double maxRangeThreshold = METER_TO_MILLIMETERS * camera.far - 1.0;
      opencv_imgproc.threshold(shortDepthData, shortDepthData, maxRangeThreshold, 0.0, opencv_imgproc.THRESH_TOZERO_INV);

      // Add noise to the depth data
      opencv_core.add(noise, shortDepthData, shortDepthData);

      // Generate some noise
      opencv_core.randu(noise, noiseZero, noiseAmplitude);
      opencv_core.multiply(shortDepthData, noise, noise, 0.001, -1);

      // Now subtract some noise
      opencv_core.subtract(shortDepthData, noise, shortDepthData);

      // libGDX renders this stuff upside down, so the image must be flipped
      opencv_core.flip(shortDepthData, depthImage, OpenCVTools.FLIP_Y);

      pointerScope.close();
   }

   public void destroy()
   {
      enableColor(false);
      enableDepth(false);

      noiseZero.close();
      noiseAmplitude.close();

      if (frameBuffer != null)
         frameBuffer.dispose();
   }
}
