package us.ihmc.robotics.sensors;

import us.ihmc.robotics.screwTheory.RigidBody;

public class RigidBodyContactSensor implements ContactSensor
{
   private final String sensorName;
   private final RigidBody rigidBody;
   private final ContactSensorType type;
   private boolean isInContact;
   
   public RigidBodyContactSensor(String sensorName, RigidBody rigidBody, ContactSensorType type)
   {
      this.sensorName = sensorName;
      this.rigidBody = rigidBody;
      this.type = type;
      this.isInContact = false;
   }

   @Override
   public boolean isInContact()
   {
      return isInContact;
   }

   @Override
   public void setIsInContact(boolean isInContact)
   {
      this.isInContact = isInContact;
   }

   @Override
   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   @Override
   public String getSensorName()
   {
      return sensorName;
   }

   @Override
   public ContactSensorType getType()
   {
      return type;
   }

   public void reset()
   {
      isInContact = false;
   }
}
