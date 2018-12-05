package us.ihmc.systemIdentification.frictionId.simulators;

public interface CoulombViscousStribeckFrictionParameters
{
   double getViscousDamping();
   double getDynamicFriction();
   double getStribeckValue();
   double getCoulombFriction();
}
