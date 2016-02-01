package us.ihmc.simulationconstructionset;

import java.awt.Component;



public class ExtraPanelConfiguration
{
   protected String name;
   protected Component panel = null;

   public ExtraPanelConfiguration(String name)
   {
      this.name = name;
   }

   public void setName(String name){
	   this.panel.setName(name);
   }

   public String getName()
   {
      return name;
   }

   public void setupPanel(Component panel)
   {
      this.panel = panel;
   }

   public Component getPanel()
   {
      return panel;
   }

}
