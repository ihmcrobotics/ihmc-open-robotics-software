package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;

public class ConfigurationBuildOrder
{
   public enum ConfigurationSpaceName
   {
      X, Y, Z, ROLL, PITCH, YAW
   }

   private ArrayList<ConfigurationSpaceName> sequence = new ArrayList<ConfigurationSpaceName>();

   public ConfigurationBuildOrder()
   {
      sequence.clear();

      for (ConfigurationSpaceName currentConfigurationSpaceName : ConfigurationSpaceName.values())
      {
         sequence.add(currentConfigurationSpaceName);
      }
      if (sequence.size() != 6)
         PrintTools.warn("Dimension of the configuration build order is not six.");
   }

   public ConfigurationBuildOrder(ConfigurationSpaceName... configurationSpaceName)
   {
      sequence.clear();

      for (ConfigurationSpaceName currentConfigurationSpaceName : configurationSpaceName)
      {
         sequence.add(currentConfigurationSpaceName);
      }
      if (sequence.size() != 6)
         PrintTools.warn("Dimension of the configuration build order is not six.");
   }

   public ConfigurationSpaceName getConfigurationSpaceName(int sequence)
   {
      return this.sequence.get(sequence);
   }
}