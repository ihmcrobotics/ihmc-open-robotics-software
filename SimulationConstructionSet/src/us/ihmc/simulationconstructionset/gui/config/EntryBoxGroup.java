package us.ihmc.simulationconstructionset.gui.config;

import java.util.ArrayList;

public class EntryBoxGroup
{
   private String name;
   private ArrayList<String> variables = new ArrayList<String>();
   private ArrayList<String> regularExpressions = new ArrayList<String>();

   public EntryBoxGroup(String name)
   {
      this.name = name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   public void addVar(String varName)
   {
      variables.add(varName);
   }

   public void addEntryBoxVars(String[] varNames)
   {
      for (int i = 0; i < varNames.length; i++)
      {
         variables.add(varNames[i]);
      }
   }

   public void addEntryBoxRegularExpressions(String[] regularExpNames)
   {
      for (int i = 0; i < regularExpNames.length; i++)
      {
         regularExpressions.add(regularExpNames[i]);
      }
   }


   public void removeEntryBoxVar(String varName)
   {
      variables.remove(varName);
   }

   public String[] getEntryBoxVars()
   {
      String[] ret = new String[variables.size()];

      variables.toArray(ret);

      return ret;
   }

   public String[] getEntryBoxRegularExpressions()
   {
      String[] ret = new String[regularExpressions.size()];

      regularExpressions.toArray(ret);

      return ret;
   }


}
