package us.ihmc.robotics.sensors;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public interface ContactSensor
{
   public boolean isInContact();
   
   public void setIsInContact(boolean isInContact);
   
   public RigidBodyBasics getRigidBody();
   
   public String getSensorName();
   
   public ContactSensorType getType();
   
   public void reset();
}
