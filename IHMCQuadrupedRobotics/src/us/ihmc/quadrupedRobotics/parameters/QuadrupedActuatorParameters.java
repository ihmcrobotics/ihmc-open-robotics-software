package us.ihmc.quadrupedRobotics.parameters;

public interface QuadrupedActuatorParameters
{
   public double getLegKp();
   public double getLegKd();
   public double getLegSoftTorqueLimit();

   public double getNeckKp();
   public double getNeckKd();
   public double getNeckSoftTorqueLimit();
}
