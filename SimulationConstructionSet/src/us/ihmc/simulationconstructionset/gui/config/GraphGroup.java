package us.ihmc.simulationconstructionset.gui.config;

import java.util.ArrayList;

public class GraphGroup
{
   private String name;
   private final ArrayList<String[][]> graphLists = new ArrayList<String[][]>();
   private int numColumns = 1;

   public GraphGroup(String name)
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

   public void addGraphVars(ArrayList<String[][]> varNames)
   {
      graphLists.addAll(varNames);
   }

   public void addGraphVars(String[][] varNames)
   {
      for (int i = 0; i < varNames.length; i++)
      {
         graphLists.add(new String[][]
         {
            varNames[i], {""}
         });
      }
   }

   public void addGraphVars(String[][][] varNamesAndConfigs)
   {
      for (int i = 0; i < varNamesAndConfigs.length; i++)
      {
         graphLists.add(varNamesAndConfigs[i]);
      }
   }


   public ArrayList<String[][]> getGraphVars()
   {
      return graphLists;
   }

   public void setNumColumns(int numColumns)
   {
      this.numColumns = numColumns;
   }

   public int getNumColumns()
   {
      return numColumns;
   }

}
