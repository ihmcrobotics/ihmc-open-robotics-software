package us.ihmc.systemIdentification.frictionId.simulators;

public interface CoulombViscousStribeckFrictionParameters
{
   double getViscousDamping();
   double getDynamciFriction();
   double getStribeckValue();
   double getCoulombFriction();
}
