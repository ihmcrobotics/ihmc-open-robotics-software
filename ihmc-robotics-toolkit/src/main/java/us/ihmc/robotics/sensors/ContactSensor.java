package us.ihmc.robotics.sensors;

import us.ihmc.mecano.multiBodySystem.RigidBody;

public interface ContactSensor
{
   public boolean isInContact();
   
   public void setIsInContact(boolean isInContact);
   
   public RigidBody getRigidBody();
   
   public String getSensorName();
   
   public ContactSensorType getType();
   
   public void reset();
}
