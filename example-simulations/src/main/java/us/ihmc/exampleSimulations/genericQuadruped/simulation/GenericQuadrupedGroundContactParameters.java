package us.ihmc.exampleSimulations.genericQuadruped.simulation;

import us.ihmc.quadrupedRobotics.simulation.GroundContactParameters;

public class GenericQuadrupedGroundContactParameters implements GroundContactParameters
{
   @Override
   public double getZStiffness()
   {
      return 5000.0;
   }
   
   @Override
   public double getZDamping()
   {
      return 600.0;
   }
   
   @Override
   public double getXYStiffness()
   {
      return 15000.0;
   }
   
   @Override
   public double getXYDamping()
   {
      return 700.0;
   }
}
