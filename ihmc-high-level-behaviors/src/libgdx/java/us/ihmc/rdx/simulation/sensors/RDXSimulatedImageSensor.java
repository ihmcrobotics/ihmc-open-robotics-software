package us.ihmc.rdx.simulation.sensors;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.RawImage;
import us.ihmc.sensors.ImageSensor;

import java.util.concurrent.atomic.AtomicBoolean;

public class RDXSimulatedImageSensor extends ImageSensor
{
   private final RDXSimulatedSensorPart[] simulatedParts;
   private final RawImage[] grabbedImages;
   private final Throttler grabThrottler;
   private boolean isRunning = false;

   private final AtomicBoolean renderedNotification = new AtomicBoolean(false);
   private final long waitForRenderDuration;

   public RDXSimulatedImageSensor(String sensorName,
                                  double fps,
                                  RDXSimulatedSensorPartDefinition partDefinition,
                                  RDXSimulatedSensorPartDefinition... partDefinitions)
   {
      super(sensorName);

      // Combine partDefinitions into one array
      RDXSimulatedSensorPartDefinition[] allDefinitions;
      if (partDefinitions == null)
         allDefinitions = new RDXSimulatedSensorPartDefinition[1];
      else
         allDefinitions = new RDXSimulatedSensorPartDefinition[partDefinitions.length + 1];

      for (int i = 0; i < partDefinitions.length; ++i)
         allDefinitions[i] = partDefinitions[i];
      allDefinitions[partDefinitions.length] = partDefinition;

      // Create simulated partDefinitions from definitions
      simulatedParts = new RDXSimulatedSensorPart[allDefinitions.length];
      for (int i = 0; i < simulatedParts.length; ++i)
         simulatedParts[i] = new RDXSimulatedSensorPart(allDefinitions[i]);

      // Create array large enough for the image keys
      int maxImageKey = findMaxImageKey(allDefinitions);
      grabbedImages = new RawImage[maxImageKey + 1];

      // Set up throttler
      double waitPeriod = Conversions.hertzToSeconds(fps);
      grabThrottler = new Throttler().setPeriod(waitPeriod);
      waitForRenderDuration = (long) Conversions.secondsToMilliseconds(waitPeriod + waitPeriod * 0.01);
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
      synchronized (renderedNotification)
      {
         try
         {
            renderedNotification.wait(waitForRenderDuration);
            return renderedNotification.getAndSet(false);
         }
         catch (InterruptedException interrupted)
         {
            return false;
         }
      }
   }

   public void render()
   {
      if (!isRunning || !grabThrottler.run())
         return;

      RigidBodyTransform sensorTransformToWorld = sensorFrameSupplier.get().getTransformToWorldFrame();

      synchronized (renderedNotification)
      {
         for (RDXSimulatedSensorPart part : simulatedParts)
            part.render(sensorTransformToWorld);

         renderedNotification.set(true);
         renderedNotification.notify();
      }
   }

   @Override
   public RawImage getImage(int imageKey)
   {
      return null;
   }

   private int findMaxImageKey(RDXSimulatedSensorPartDefinition[] parts)
   {
      int maxKey = 0;
      for (RDXSimulatedSensorPartDefinition part : parts)
      {
         maxKey = Math.max(maxKey, Math.max(part.getColorImageKey(), part.getDepthImageKey()));
      }
      return maxKey;
   }

   private class RDXSimulatedSensorPart extends RDXSensorSimulator
   {
      private final RDXSimulatedSensorPartDefinition definition;

      private RDXSimulatedSensorPart(RDXSimulatedSensorPartDefinition definition)
      {
         super(definition.getImageWidth(), definition.getImageHeight(), definition.getVerticalFOV(), definition.getMinRange(), definition.getMaxRange());
         this.definition = definition;

         enableColor(definition.hasColor());
         enableDepth(definition.hasDepth());
      }

      @Override
      public void render(RigidBodyTransform sensorTransformToWorld)
      {
         // Find the world to part transform
         RigidBodyTransform partTransformToWorld = new RigidBodyTransform(sensorTransformToWorld);
         partTransformToWorld.getTranslation().add(definition.getSensorToPartTransform().getTranslation());
         partTransformToWorld.getRotation().append(definition.getSensorToPartTransform().getRotation());

         // Render
         super.render(partTransformToWorld);

         // Set the grabbed images
         if (definition.hasColor())
         {
            int colorImageKey = definition.getColorImageKey();
            if (grabbedImages[colorImageKey] != null)
               grabbedImages[colorImageKey].release();

            grabbedImages[colorImageKey] = getColorImage();
         }

         if (definition.hasDepth())
         {
            int depthImageKey = definition.getDepthImageKey();
            if (grabbedImages[depthImageKey] != null)
               grabbedImages[depthImageKey].release();

            grabbedImages[definition.getDepthImageKey()] = getDepthImage();
         }
      }
   }
}
