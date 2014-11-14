package us.ihmc.simulationconstructionset.gui.config;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.ViewportConfiguration;

public class ViewportConfigurationList
{
   private ArrayList<ViewportConfiguration> configs = new ArrayList<ViewportConfiguration>();

   public ViewportConfigurationList()
   {
   }

   public void addViewportConfiguration(ViewportConfiguration config)
   {
      configs.add(config);
   }

   public String[] getViewportConfigurationNames()
   {
      int n = configs.size();
      String[] ret = new String[n];

      for (int i = 0; i < n; i++)
      {
         ret[i] = (configs.get(i)).getName();
      }

      return ret;
   }

   public ViewportConfiguration getViewportConfiguration(String name)
   {
      int n = configs.size();

      for (int i = 0; i < n; i++)
      {
         ViewportConfiguration config = (configs.get(i));

         if (config.getName().equals(name))
            return config;
      }

      return null;
   }
}
