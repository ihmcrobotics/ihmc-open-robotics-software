package us.ihmc.modelFileLoaders.SdfLoader;

import us.ihmc.robotics.sensors.ContactSensorType;

public class SDFContactSensor
{
   private final String name;
   private final String parentJointName;
   private final ContactSensorType type;

   public String getName()
   {
      return name;
   }

   public ContactSensorType getSensorType()
   {
      return type;
   }

   public String getParentJointName()
   {
      return parentJointName;
   }

   public SDFContactSensor(String name, String parentJointName, ContactSensorType type)
   {
      this.name = name;
      this.parentJointName = parentJointName;
      this.type = type;
   }
}
