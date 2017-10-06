package us.ihmc.simulationconstructionset.gui.config;


public class Configuration
{
   private String name;
   private String graphGroupName = "", entryBoxGroupName = "";

   public Configuration(String name)
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

   public void setGraphGroupName(String graphGroupName)
   {
      this.graphGroupName = graphGroupName;
   }

   public void setEntryBoxGroupName(String entryBoxGroupName)
   {
      this.entryBoxGroupName = entryBoxGroupName;
   }

   public String getGraphGroupName()
   {
      return graphGroupName;
   }

   public String getEntryBoxGroupName()
   {
      return entryBoxGroupName;
   }

}
