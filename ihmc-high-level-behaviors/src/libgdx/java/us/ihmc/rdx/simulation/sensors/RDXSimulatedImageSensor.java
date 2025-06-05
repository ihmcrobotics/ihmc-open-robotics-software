package us.ihmc.rdx.simulation.sensors;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.sensors.ImageSensor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Simulates an image sensor consisting of one or more cameras (e.g. ZED left/right cameras, D455 color/depth cameras)
 */
public class RDXSimulatedImageSensor extends ImageSensor
{
   private final List<RDXSimulatedCamera> cameras = new ArrayList<>();

   private final Map<Integer, RawImage> grabbedImages = new HashMap<>();
   private final Map<Integer, RDXSimulatedCamera> imageKeyToCameraMap = new HashMap<>();
   private int[] imageKeys;

   private final Throttler grabThrottler = new Throttler();

   private final AtomicBoolean renderedNotification = new AtomicBoolean(false);
   private final long waitForRenderDuration;

   private boolean isRunning = false;

   /**
    * Construct an {@link RDXSimulatedImageSensor} with no cameras.
    * To add cameras, use {@code addCamera()}.
    *
    * @param sensorName Name of the sensor (e.g. "ZED 2i", or "D455")
    * @param fps        Grab frequency of the sensor (frames per second)
    */
   public RDXSimulatedImageSensor(String sensorName, double fps)
   {
      super(sensorName);

      double waitPeriod = Conversions.hertzToSeconds(fps);
      grabThrottler.setPeriod(waitPeriod);
      waitForRenderDuration = Math.round(Conversions.secondsToMilliseconds(1.1 * waitPeriod));
   }

   /**
    * Add a camera to the sensor.
    *
    * @param cameraName              Name for the camera (e.g. "left", "depth", etc.). Each camera of a sensor should have a unique name.
    * @param imageWidth              Width, in pixels, of the output image(s).
    * @param imageHeight             Height, in pixels, of the output image(s).
    * @param verticalFOV             Vertical field of view, in degrees, of the camera.
    * @param minRange                Minimum distance the camera can see, in meters.
    * @param maxRange                Maximum distance the camera can see, in meters.
    * @param hasColor                Whether this camera produces color images.
    * @param colorImageKey           Image key for the color images produced by this camera. Unused if {@code hasColor == false}.
    * @param hasDepth                Whether this camera produces depth images.
    * @param depthImageKey           Image key for the depth images produces by this camera. Unused if {@code hasDepth == false}.
    * @param noiseAmount             Amount of noise in the depth images produced by this camera.
    * @param cameraToSensorTransform Transform from the sensor's frame to this camera's frame.
    */
   public void addCamera(String cameraName,
                         int imageWidth,
                         int imageHeight,
                         float verticalFOV,
                         float minRange,
                         float maxRange,
                         boolean hasColor,
                         int colorImageKey,
                         boolean hasDepth,
                         int depthImageKey,
                         int noiseAmount,
                         RigidBodyTransformReadOnly cameraToSensorTransform)
   {
      RDXSimulatedCamera camera = new RDXSimulatedCamera(cameraName,
                                                         imageWidth,
                                                         imageHeight,
                                                         verticalFOV,
                                                         minRange,
                                                         maxRange,
                                                         hasColor,
                                                         colorImageKey,
                                                         hasDepth,
                                                         depthImageKey,
                                                         noiseAmount,
                                                         cameraToSensorTransform);
      cameras.add(camera);

      if (hasColor)
         imageKeyToCameraMap.put(colorImageKey, camera);
      if (hasDepth)
         imageKeyToCameraMap.put(depthImageKey, camera);

      imageKeys = new int[imageKeyToCameraMap.size()];
      int i = 0;
      for (Integer key : imageKeyToCameraMap.keySet())
         imageKeys[i++] = key;
   }

   public void create(RDX3DScene scene)
   {
      for (RDXSimulatedCamera camera : cameras)
         camera.create(scene);
   }

   @Override
   protected void onSensorFrameChanged()
   {
      for (RDXSimulatedCamera camera : cameras)
         camera.updateFrame(sensorFrame);
   }

   @Override
   protected boolean startSensor()
   {
      return isRunning = true;
   }

   @Override
   public boolean isSensorRunning()
   {
      return isRunning;
   }

   @Override
   protected boolean grab()
   {
      // Wait for newly rendered image data
      synchronized (renderedNotification)
      {
         try
         {
            renderedNotification.wait(waitForRenderDuration);

            if (!renderedNotification.getAndSet(false))
               return false;
         }
         catch (InterruptedException interrupted)
         {
            return false;
         }
      }

      // Get the new images and store them
      for (RDXSimulatedCamera camera : cameras)
      {
         if (camera.hasColorImage())
         {
            RawImage previousImage;

            synchronized (grabbedImages)
            {
               previousImage = grabbedImages.put(camera.getColorImageKey(), camera.getColorImage());
            }

            if (previousImage != null)
               previousImage.release();
         }
         if (camera.hasDepthImage())
         {
            RawImage previousImage;

            synchronized (grabbedImages)
            {
               previousImage = grabbedImages.put(camera.getDepthImageKey(), camera.getDepthImage());
            }

            if (previousImage != null)
               previousImage.release();
         }
      }

      return true;
   }

   @Override
   public int[] getImageKeys()
   {
      return imageKeys;
   }

   @Override
   public void close()
   {
      super.close();
      isRunning = false;
   }

   /**
    * Renders the color and depth images at this sensor's FPS. Should be called continuously in the render loop.
    */
   public void render()
   {
      if (!isRunning || !grabThrottler.run())
         return;

      try // Something throws NotARotationMatrixExceptions. Race condition?
      {
         RigidBodyTransform sensorTransformToRoot = sensorFrame.getTransformToRoot();

         for (RDXSimulatedCamera camera : cameras)
         {
            // Render the camera's view
            camera.render(sensorTransformToRoot);
         }
      }
      catch (NotARotationMatrixException notARotationMatrixException)
      {
         LogTools.error(notARotationMatrixException);
      }

      synchronized (renderedNotification)
      {
         renderedNotification.set(true);
         renderedNotification.notify();
      }
   }

   @Override
   public RawImage getImage(int imageKey)
   {
      synchronized (grabbedImages)
      {
         if (grabbedImages.get(imageKey) == null)
            return null;

         return grabbedImages.get(imageKey).get();
      }
   }

   @Override
   public ReferenceFrame getImageFrame(int imageKey)
   {
      if (!imageKeyToCameraMap.containsKey(imageKey))
         return null;

      return imageKeyToCameraMap.get(imageKey).getCameraFrame();
   }

   @Override
   public ReferenceFrame[] getImageFrames()
   {
      return cameras.stream().map(RDXSimulatedCamera::getCameraFrame).toArray(ReferenceFrame[]::new);
   }

   private static class RDXSimulatedCamera extends RDXSensorSimulator
   {
      private final String cameraName;

      private final int colorImageKey;
      private final int depthImageKey;

      private ReferenceFrame cameraFrame;
      private final RigidBodyTransformReadOnly cameraToSensorTransform;

      private RDXSimulatedCamera(String cameraName,
                                 int imageWidth,
                                 int imageHeight,
                                 float verticalFOV,
                                 float minRange,
                                 float maxRange,
                                 boolean hasColor,
                                 int colorImageKey,
                                 boolean hasDepth,
                                 int depthImageKey,
                                 int noiseAmount,
                                 RigidBodyTransformReadOnly cameraToSensorTransform)
      {
         super(imageWidth, imageHeight, verticalFOV, minRange, maxRange, noiseAmount);
         enableColor(hasColor);
         enableDepth(hasDepth);

         this.cameraName = cameraName;

         this.colorImageKey = colorImageKey;
         this.depthImageKey = depthImageKey;
         this.cameraToSensorTransform = cameraToSensorTransform;

         updateFrame(ReferenceFrameTools.getWorldFrame());
      }

      private void updateFrame(ReferenceFrame sensorFrame)
      {
         if (cameraFrame != null)
            cameraFrame.remove();
         cameraFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(cameraName, sensorFrame, cameraToSensorTransform);
      }

      @Override
      public void render(RigidBodyTransform sensorTransformToWorld)
      {
         // Find the world to part transform
         cameraFrame.update();
         RigidBodyTransform cameraToWorldTransform = cameraFrame.getTransformToRoot();

         // Render
         super.render(cameraToWorldTransform);
      }

      public int getColorImageKey()
      {
         return colorImageKey;
      }

      public int getDepthImageKey()
      {
         return depthImageKey;
      }

      public ReferenceFrame getCameraFrame()
      {
         return cameraFrame;
      }
   }
}
