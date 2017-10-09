package us.ihmc.simulationconstructionset.gui.config;

import java.util.ArrayList;

public class VarGroup
{
   private ArrayList<String> variables = new ArrayList<String>();
   private ArrayList<String> expressions = new ArrayList<String>();
   private String name;

   public VarGroup(String name)
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

   public void addVars(String[] varNames)
   {
      if (varNames == null)
      {
         return;
      }

      for (int i = 0; i < varNames.length; i++)
      {
         variables.add(varNames[i]);
      }
   }

   public void addRegularExpressions(String[] regularExpressions)
   {
      if (regularExpressions == null)
      {
         return;
      }

      for (int i = 0; i < regularExpressions.length; i++)
      {
         expressions.add(regularExpressions[i]);
      }
   }

   /*
    * public void addRegularExpression(String regularExpression)
    * {
    * regularExpressions.add(regularExpression);
    * }
    */
   public void removeVar(String varName)
   {
      variables.remove(varName);
   }

   public void removeRegularExpression(String regularExpressions)
   {
      expressions.remove(regularExpressions);
   }

   public String[] getVars()
   {
      String[] ret = new String[variables.size()];

      variables.toArray(ret);

      return ret;
   }

   public String[] getRegularExpressions()
   {
      String[] ret = new String[expressions.size()];

      expressions.toArray(ret);

      return ret;
   }
}
