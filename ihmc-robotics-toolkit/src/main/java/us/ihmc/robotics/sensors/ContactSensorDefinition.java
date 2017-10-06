package us.ihmc.robotics.sensors;

import us.ihmc.robotics.screwTheory.RigidBody;

public class ContactSensorDefinition
{
   private final RigidBody contactingRigidBody;
   private final String contactSensorName;
   private final ContactSensorType type;
   
   public ContactSensorDefinition(String contactSensorName, RigidBody rigidBody, ContactSensorType type)
   {
      this.contactSensorName = contactSensorName;
      this.contactingRigidBody = rigidBody;
      this.type = type;
   }
   
   public String getSensorName()
   {
      return contactSensorName;
   }
   
   public RigidBody getRigidBody()
   {
      return contactingRigidBody;
   }
   
   public String getParentJointName()
   {
      return contactingRigidBody.getParentJoint().getName();
   }
   
   public ContactSensorType getSensorType()
   {
      return this.type;
   }
   
   @Override
   public String toString()
   {
      return "ContactSenserDefinition: " + contactSensorName + " attached to " + contactingRigidBody;
   }
   
   @Override
   public int hashCode()
   {
      return 17 + (31 * contactSensorName.hashCode()) + contactingRigidBody.getName().hashCode();
   }
   
   @Override
   public boolean equals(Object other)
   {
      if(other instanceof ContactSensorDefinition)
      {
         ContactSensorDefinition otherSensor = (ContactSensorDefinition) other;
         return otherSensor.getSensorName().equals(getSensorName()) && otherSensor.getParentJointName().equals(getParentJointName());
      }
      
      return false;
   }
}
