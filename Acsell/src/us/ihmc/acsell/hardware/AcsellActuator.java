package us.ihmc.acsell.hardware;

public interface AcsellActuator
{
   public String getName();   
   public double getKt();
   public double getKm();
   public double getCurrentLimit();
   public int getBus();
   public int getIndex();
   public int getSensedCurrentToTorqueDirection();
   public double getMotorInertia();
}
