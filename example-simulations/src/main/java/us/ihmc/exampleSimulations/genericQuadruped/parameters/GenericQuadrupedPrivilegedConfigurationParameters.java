package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.quadrupedRobotics.parameters.QuadrupedPrivilegedConfigurationParameters;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class GenericQuadrupedPrivilegedConfigurationParameters extends QuadrupedPrivilegedConfigurationParameters
{
   public double getPrivilegedConfiguration(RobotQuadrant quadrant)
   {
      switch (quadrant)
      {
         default:
            return 0.0;
      }
   }

   public double getDefaultConfigurationGain()
   {
      return 40.0;
   }

   public double getDefaultVelocityGain()
   {
      return 6.0;
   }

   public double getDefaultWeight()
   {
      return 5.0;
   }

   public double getKneeConfigurationGain()
   {
      return 150.0;
   }

   public double getKneeConfigurationVelocityGain()
   {
      return 6.0;
   }

   public double getKneeWeight()
   {
      return 5.0;
   }
}
