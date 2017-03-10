package us.ihmc.simulationconstructionset.gui;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.setterUpper.GUISetterUpper;

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
