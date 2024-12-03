package us.ihmc.rdx.simulation.sensors;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class RDXSimulatedSensorPartDefinition
{
   private int imageWidth = 1280;
   private int imageHeight = 720;
   private float verticalFOV = 70.0f;
   private float minRange = 0.2f;
   private float maxRange = 40.0f;
   private boolean hasColor = false;
   private int colorImageKey = 0;
   private boolean hasDepth = false;
   private int depthImageKey = 0;
   private final RigidBodyTransform sensorToPartTransform = new RigidBodyTransform();

   public RDXSimulatedSensorPartDefinition withWidth(int imageWidth)
   {
      this.imageWidth = imageWidth;
      return this;
   }

   public RDXSimulatedSensorPartDefinition withHeight(int imageHeight)
   {
      this.imageHeight = imageHeight;
      return this;
   }

   public RDXSimulatedSensorPartDefinition withVerticalFOV(float verticalFOV)
   {
      this.verticalFOV = verticalFOV;
      return this;
   }

   public RDXSimulatedSensorPartDefinition withMinRange(float minRange)
   {
      this.minRange = minRange;
      return this;
   }

   public RDXSimulatedSensorPartDefinition withColor(int colorImageKey)
   {
      hasColor = true;
      this.colorImageKey = colorImageKey;
      return this;
   }

   public RDXSimulatedSensorPartDefinition withDepth(int depthImageKey)
   {
      hasDepth = true;
      this.depthImageKey = depthImageKey;
      return this;
   }

   public RDXSimulatedSensorPartDefinition withTransformFromSensor(RigidBodyTransformReadOnly transformFromSensor)
   {
      sensorToPartTransform.set(transformFromSensor);
      return this;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public float getVerticalFOV()
   {
      return verticalFOV;
   }

   public float getMinRange()
   {
      return minRange;
   }

   public float getMaxRange()
   {
      return maxRange;
   }

   public boolean hasColor()
   {
      return hasColor;
   }

   public int getColorImageKey()
   {
      return colorImageKey;
   }

   public boolean hasDepth()
   {
      return hasDepth;
   }

   public int getDepthImageKey()
   {
      return depthImageKey;
   }

   public RigidBodyTransformReadOnly getSensorToPartTransform()
   {
      return sensorToPartTransform;
   }
}