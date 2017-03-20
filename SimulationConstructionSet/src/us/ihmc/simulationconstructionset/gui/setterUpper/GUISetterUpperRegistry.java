package us.ihmc.simulationconstructionset.gui.setterUpper;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class GUISetterUpperRegistry
{

   private final ArrayList<GUISetterUpper> guiSetterUppers = new ArrayList<GUISetterUpper>();
   
   public void registerGUISetterUpper(GUISetterUpper guiSetterUpper)
   {
      this.guiSetterUppers.add(guiSetterUpper);
      
   }
   
   public void setupGUIs(SimulationConstructionSet scs)
   {
      for (GUISetterUpper guiSetterUpper : guiSetterUppers)
      {
         guiSetterUpper.setupGUI(scs);
      }
   }
}
