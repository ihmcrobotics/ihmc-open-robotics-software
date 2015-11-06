package us.ihmc.robotics.sensors;

public interface ContactBasedFootSwitch extends FootSwitchProvider
{
   public boolean isInContact();
   public void setIsInContact(boolean isInContact);
}
