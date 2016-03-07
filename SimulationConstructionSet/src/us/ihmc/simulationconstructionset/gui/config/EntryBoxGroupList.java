package us.ihmc.simulationconstructionset.gui.config;

import java.util.ArrayList;

public class EntryBoxGroupList
{
   private final ArrayList<EntryBoxGroup> groups = new ArrayList<EntryBoxGroup>();

   public EntryBoxGroupList()
   {
   }

   public void addEntryBoxGroup(EntryBoxGroup group)
   {
      groups.add(group);
   }

   public void removeEntryBoxGroup(EntryBoxGroup group)
   {
      groups.remove(group);
   }
   
   public String[] getEntryBoxGroupNames()
   {
      int n = groups.size();
      String[] ret = new String[n];

      for (int i = 0; i < n; i++)
      {
         ret[i] = (groups.get(i)).getName();
      }

      return ret;
   }

   public EntryBoxGroup getEntryBoxGroup(String name)
   {
      int n = groups.size();

      for (int i = 0; i < n; i++)
      {
         EntryBoxGroup group = (groups.get(i));

         if (group.getName().equals(name))
            return group;
      }

      return null;
   }

   public String getNextGroupName(String currentGroupName)
   {
      for (int i = 0; i < groups.size(); i++)
      {
         EntryBoxGroup entryBoxGroup = groups.get(i);

         if (currentGroupName.equals(entryBoxGroup.getName()))
         {
            int index = (i + 1) % groups.size();

            return groups.get(index).getName();
         }

      }

      if (groups.isEmpty())
      {
         return "";
      }

      return groups.get(0).getName();
   }

   public String getPreviousGroupName(String currentGroupName)
   {
      for (int i = 0; i < groups.size(); i++)
      {
         EntryBoxGroup entryBoxGroup = groups.get(i);

         if (currentGroupName.equals(entryBoxGroup.getName()))
         {
            int index = (i - 1 + groups.size()) % groups.size();

            return groups.get(index).getName();
         }

      }

      if (groups.isEmpty())
      {
         return "";
      }

      return groups.get(0).getName();
   }

}
