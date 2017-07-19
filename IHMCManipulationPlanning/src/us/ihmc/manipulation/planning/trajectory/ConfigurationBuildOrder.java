package us.ihmc.manipulation.planning.trajectory;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;

public class ConfigurationBuildOrder
{
   public enum ConfigurationSpaceName
   {
      Translation_X, Translation_Y, Translation_Z, Rotation_Roll, Rotation_Pitch, Rotation_Yaw
   }
   
   private ArrayList<ConfigurationSpaceName> sequence = new ArrayList<ConfigurationSpaceName>();
   
   public ConfigurationBuildOrder()
   {
      sequence.clear();
      
      for (ConfigurationSpaceName currentConfigurationSpaceName : ConfigurationSpaceName.values())
      {
         sequence.add(currentConfigurationSpaceName);
      }
      if(sequence.size() != 6)
         PrintTools.warn("Dimension of the configuration build order is not six.");
   }
   
   public ConfigurationBuildOrder(ConfigurationSpaceName... configurationSpaceName)
   {
      sequence.clear();
      
      for (ConfigurationSpaceName currentConfigurationSpaceName : configurationSpaceName)
      {
         sequence.add(currentConfigurationSpaceName);
      }
      if(sequence.size() != 6)
         PrintTools.warn("Dimension of the configuration build order is not six.");
   }
   
   public ConfigurationSpaceName getConfigurationSpaceName(int sequence)
   {  
      return this.sequence.get(sequence);
   }
}
