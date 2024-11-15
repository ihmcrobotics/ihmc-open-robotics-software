package us.ihmc.rdx.simulation.sensors;

import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.glutils.SensorFrameBuffer;
import com.badlogic.gdx.graphics.glutils.SensorFrameBufferBuilder;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
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
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;

import java.time.Instant;

/*
 * TODO:
 *  1. add noise
 *  2. use glsl to linearize depth + add noise
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
   private SensorFrameBuffer frameBuffer;

   // Images
   private Mat colorImage = null;
   private Instant colorGrabTime = Instant.EPOCH;

   private Mat depthImage = null;
   private Mat depthData = null;
   private Instant depthGrabTime = Instant.EPOCH;

   private long sequenceNumber = 0L;

   public RDXSensorSimulator(int imageWidth, int imageHeight, float verticalFOV, float minRange, float maxRange)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.verticalFOV = verticalFOV;
      this.minRange = minRange;
      this.maxRange = maxRange;

      double focalLength = calculateFocalLength(imageHeight, verticalFOV);
      cameraIntrinsics = new CameraIntrinsics(imageHeight, imageWidth, focalLength, focalLength, 0.5 * imageWidth, 0.5 * imageHeight);
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
      SensorFrameBufferBuilder frameBufferBuilder = new SensorFrameBufferBuilder(imageWidth, imageHeight);
      frameBufferBuilder.addColorTextureAttachment(GL41.GL_RGBA8, GL41.GL_RGBA, GL41.GL_UNSIGNED_BYTE);
      frameBufferBuilder.addDepthTextureAttachment(GL41.GL_DEPTH_COMPONENT32F, GL41.GL_FLOAT);
      frameBuffer = frameBufferBuilder.build();
   }

   public void enableColor(boolean enable)
   {
      if (enable)
      {
         if (colorImage == null)
            colorImage = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4);
      }
      else
      {
         if (colorImage != null)
            colorImage.close();
         colorImage = null;
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
         }
      }
      else
      {
         if (depthImage != null)
         {
            depthImage.close();
            depthData.close();
         }
         depthImage = null;
         depthData = null;
      }
   }

   public void grab(RigidBodyTransform cameraTransformToWorld)
   {
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

      if (colorImage != null) // If color is enabled
      {
         // Render and "grab" the color image
         simulationScene.preRender(camera);
         GL41.glViewport(0, 0, imageWidth, imageHeight);
         simulationScene.render(RDXSceneLevel.GROUND_TRUTH.SINGLETON_SET);
         simulationScene.postRender();

         // Save the grab time
         colorGrabTime = Instant.now();

         // Read the grabbed color image
         GL41.glReadBuffer(GL41.GL_COLOR_ATTACHMENT0);
         GL41.glReadPixels(0, 0, imageWidth, imageHeight, GL41.GL_RGBA, GL41.GL_UNSIGNED_BYTE, colorImage.data().address());

         // libGDX renders this stuff upside down, so the image must be flipped
         opencv_core.flip(colorImage, colorImage, OpenCVTools.FLIP_Y);
      }

      if (depthImage != null) // If depth is enabled
      {
         // Render and "grab" the color image
         simulationScene.preRenderDepth(camera);
         GL41.glViewport(0, 0, imageWidth, imageHeight);
         simulationScene.renderDepth(RDXSceneLevel.GROUND_TRUTH.SINGLETON_SET);
         simulationScene.postRenderDepth();

         // Save the grab time
         depthGrabTime = Instant.now();

         // Read the depth buffer
         GL41.glReadBuffer(GL41.GL_DEPTH_ATTACHMENT);
         GL41.glReadPixels(0, 0, imageWidth, imageHeight, GL41.GL_DEPTH_COMPONENT, GL41.GL_FLOAT, depthData.data().address());

         // Linearizing Depth Buffer Data: https://learnopengl.com/Advanced-OpenGL/Depth-testing
         // transform depth values to "normalized device coordinates"
         depthData.convertTo(depthData, opencv_core.CV_32FC1, 2.0, -1.0);

         // use normalized device coordinates to get linear depth
         MatExpr linearizedDepth = opencv_core.multiply(depthData, camera.far - camera.near);
         linearizedDepth = opencv_core.subtract(new Scalar(camera.near + camera.far), linearizedDepth);
         linearizedDepth = opencv_core.divide(2.0 * camera.near * camera.far, linearizedDepth);

         // Convert float32 in meters to uint16 in millimeters
         linearizedDepth.asMat().convertTo(depthImage, opencv_core.CV_16UC1, METER_TO_MILLIMETERS, 0.0);
         linearizedDepth.close();

         // Remove depth values at max range
         double maxRangeThreshold = METER_TO_MILLIMETERS * camera.far - 1.0;
         opencv_imgproc.threshold(depthImage, depthImage, maxRangeThreshold, 0.0, opencv_imgproc.THRESH_TOZERO_INV);

         // libGDX renders this stuff upside down, so the image must be flipped
         opencv_core.flip(depthImage, depthImage, OpenCVTools.FLIP_Y);
      }

      frameBuffer.end();

      ++sequenceNumber;
   }

   public RawImage getColorImage()
   {
      if (colorImage == null)
         throw new IllegalStateException("No color image has been grabbed.");

      return new RawImage(colorImage.clone(), null, PixelFormat.RGBA8, cameraIntrinsics, CameraModel.PINHOLE, cameraPose, colorGrabTime, sequenceNumber, 0.0f);
   }

   public RawImage getDepthImage()
   {
      if (depthImage == null)
         throw new IllegalStateException("No depth image has been grabbed.");

      return new RawImage(depthImage.clone(), null, PixelFormat.GRAY16, cameraIntrinsics, CameraModel.PINHOLE, cameraPose, depthGrabTime, sequenceNumber, 0.001f);
   }

   public void destroy()
   {
      enableColor(false);
      enableDepth(false);
      if (frameBuffer != null)
         frameBuffer.dispose();
   }
}
