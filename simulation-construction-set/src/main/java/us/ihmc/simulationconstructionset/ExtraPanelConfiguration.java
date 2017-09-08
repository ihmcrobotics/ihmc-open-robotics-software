package us.ihmc.simulationconstructionset;

import java.awt.Component;

public class ExtraPanelConfiguration
{
   private String name;
   private Component panel = null;
   private boolean showOnStart;

   public ExtraPanelConfiguration(String name, Component panel, boolean showOnStart)
   {
      this.name = name;
      this.panel = panel;
      this.setShowOnStart(showOnStart);
   }

   public void setName(String name)
   {
	   this.panel.setName(name);
   }

   public String getName()
   {
      return name;
   }

   public void setPanel(Component panel)
   {
      this.panel = panel;
   }

   public Component getPanel()
   {
      return panel;
   }

   public boolean showOnStart()
   {
      return showOnStart;
   }

   public void setShowOnStart(boolean showOnStart)
   {
      this.showOnStart = showOnStart;
   }
}
