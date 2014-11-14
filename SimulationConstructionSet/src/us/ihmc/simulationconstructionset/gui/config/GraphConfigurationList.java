package us.ihmc.simulationconstructionset.gui.config;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.GraphConfiguration;

public class GraphConfigurationList
{
   private ArrayList<GraphConfiguration> configs = new ArrayList<GraphConfiguration>();

   public GraphConfigurationList()
   {
      addGraphConfiguration(GraphConfiguration.getStandardAutoScalingConfiguration());
   }

   public void addGraphConfiguration(GraphConfiguration config)
   {
      configs.add(config);
   }

   public String[] getGraphConfigurationNames()
   {
      int n = configs.size();
      String[] ret = new String[n];

      for (int i = 0; i < n; i++)
      {
         ret[i] = (configs.get(i)).getName();
      }

      return ret;
   }

   public GraphConfiguration getGraphConfiguration(String name)
   {
      int n = configs.size();

      for (int i = 0; i < n; i++)
      {
         GraphConfiguration config = (configs.get(i));

         if (config.getName().equals(name))
            return config;
      }

      return null;
   }
}
