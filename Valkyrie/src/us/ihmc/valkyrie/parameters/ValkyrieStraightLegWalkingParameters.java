package us.ihmc.valkyrie.parameters;

import us.ihmc.commonWalkingControlModules.configurations.StraightLegWalkingParameters;

public class ValkyrieStraightLegWalkingParameters extends StraightLegWalkingParameters
{
   private final boolean runningOnRealRobot;

   public ValkyrieStraightLegWalkingParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   /** {@inheritDoc} */
   public boolean attemptToStraightenLegs()
   {
      return false;
   }

   /** {@inheritDoc} */
   public double getBentLegPrivilegedConfigurationGain()
   {
      return runningOnRealRobot ? 40.0 : 100.0;
   }
}
