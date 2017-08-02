package us.ihmc.valkyrie.parameters;

import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;

public class ValkyrieLegConfigurationParameters extends LegConfigurationParameters
{
   private final boolean runningOnRealRobot;

   public ValkyrieLegConfigurationParameters(boolean runningOnRealRobot)
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
