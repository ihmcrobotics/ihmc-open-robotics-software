package us.ihmc.simulationconstructionset.gui.config;

import java.util.ArrayList;

public class GUIConfigFromFileList
{
   private ArrayList<GUIConfigFromFile> configs = new ArrayList<GUIConfigFromFile>();

   public GUIConfigFromFileList()
   {
   }

   public void addConfiguration(GUIConfigFromFile config)
   {
      boolean add= true;
	   for(GUIConfigFromFile configurations: configs){
    	  if(configurations.getFileName().equals(config.getFileName())){
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
         ret[i] = (configs.get(i)).getFileName();
      }

      return ret;
   }

   public GUIConfigFromFile getConfiguration(String name)
   {
      int n = configs.size();

      for (int i = 0; i < n; i++)
      {
    	  GUIConfigFromFile config = (configs.get(i));

         if (config.getFileName().equals(name))
            return config;
      }

      return null;
   }
}
