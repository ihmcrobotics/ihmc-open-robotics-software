package us.ihmc.simulationconstructionset.gui.config;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.simulationconstructionset.ExtraPanelConfiguration;

public class ExtraPanelConfigurationList
{
   private ArrayList<ExtraPanelConfiguration> panels = new ArrayList<ExtraPanelConfiguration>();

   public ExtraPanelConfigurationList()
   {
   }

   public void addExtraPanelConfiguration(ExtraPanelConfiguration panel)
   {
      panels.add(panel);
   }

   public String[] getExtraPanelConfigurationNames()
   {
      int n = panels.size();
      String[] ret = new String[n];

      for (int i = 0; i < n; i++)
      {
         ret[i] = (panels.get(i)).getName();
      }

      return ret;
   }

   public ExtraPanelConfiguration getExtraPanelConfiguration(String name)
   {
      int n = panels.size();
      
      for (int i = 0; i < n; i++)
      {
         ExtraPanelConfiguration panel = panels.get(i);
         if (panel.getName().equals(name))
            return panel;
      }

      return null;
   }

   public int getindexOfExtraPanelConfiguration(String name)
   {
      int n = panels.size();
      for (int i = 0; i < n; i++)
      {
         ExtraPanelConfiguration panel = (panels.get(i));

         if (panel.getName().equals(name))
            return i;
      }

      return 0;
   }
   
   public List<ExtraPanelConfiguration> getConfigurationList()
   {
      return panels;
   }
}
