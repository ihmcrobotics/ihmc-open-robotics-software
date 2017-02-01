package us.ihmc.exampleSimulations.beetle.parameters;

import us.ihmc.quadrupedRobotics.simulation.GroundContactParameters;

public class RhinoBeetleGroundContactParameters implements GroundContactParameters
{
   @Override
   public double getZStiffness()
   {
      return 200.0;
   }

   @Override
   public double getZDamping()
   {
      return 250.0;
   }

   @Override
   public double getXYStiffness()
   {
      return 5000.0;
   }

   @Override
   public double getXYDamping()
   {
      return 100.0;
   }
}
