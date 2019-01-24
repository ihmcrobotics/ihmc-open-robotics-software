package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.quadrupedRobotics.parameters.QuadrupedPrivilegedConfigurationParameters;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class GenericQuadrupedPrivilegedConfigurationParameters extends QuadrupedPrivilegedConfigurationParameters
{
   public double getPrivilegedConfiguration(LegJointName jointName, RobotQuadrant quadrant)
   {
      switch (jointName)
      {
      case HIP_ROLL:
         return 0.0;
      case HIP_PITCH:
         if (quadrant.isQuadrantInFront())
            return 0.9;
         else
            return -0.9;
      case KNEE_PITCH:
         if (quadrant.isQuadrantInFront())
            return -2.0;
         else
            return 2.0;
      }

      throw new IllegalArgumentException("Joint " + jointName + " is not implemented for privileged configurations.");
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
