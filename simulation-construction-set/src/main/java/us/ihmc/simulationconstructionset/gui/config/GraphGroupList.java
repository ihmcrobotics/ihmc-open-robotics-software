package us.ihmc.simulationconstructionset.gui.config;

import java.util.ArrayList;

public class GraphGroupList
{
   private ArrayList<GraphGroup> groups = new ArrayList<GraphGroup>();

   public GraphGroupList()
   {
   }

   public void addGraphGroup(GraphGroup group)
   {
      groups.add(group);
   }

   public String[] getGraphGroupNames()
   {
      int n = groups.size();
      String[] ret = new String[n];

      for (int i = 0; i < n; i++)
      {
         ret[i] = (groups.get(i)).getName();
      }

      return ret;
   }

   public GraphGroup getGraphGroup(String name)
   {
      int n = groups.size();

      for (int i = 0; i < n; i++)
      {
         GraphGroup group = (groups.get(i));

         if (group.getName().equals(name))
            return group;
      }

      return null;
   }
}
