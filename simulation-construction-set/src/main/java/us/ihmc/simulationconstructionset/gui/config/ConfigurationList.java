package us.ihmc.simulationconstructionset.gui.config;

import java.util.ArrayList;

public class ConfigurationList
{
   private ArrayList<Configuration> configs = new ArrayList<Configuration>();

   public ConfigurationList()
   {
   }

   public void addConfiguration(Configuration config)
   {
      boolean add= true;
	   for(Configuration configurations: configs){
    	  if(configurations.getName().equals(config.getName())){
    		  add=false;
    	  }
      }
	   if(add){
		   configs.add(config);
	   }

   }

   public String[] getConfigurationNames()
   {
      int n = configs.size();
      String[] ret = new String[n];

      for (int i = 0; i < n; i++)
      {
         ret[i] = (configs.get(i)).getName();
      }

      return ret;
   }

   public Configuration getConfiguration(String name)
   {
      int n = configs.size();

      for (int i = 0; i < n; i++)
      {
         Configuration config = (configs.get(i));

         if (config.getName().equals(name))
            return config;
      }

      return null;
   }
}
