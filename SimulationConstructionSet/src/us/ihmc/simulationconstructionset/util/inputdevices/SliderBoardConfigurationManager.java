package us.ihmc.simulationconstructionset.util.inputdevices;

import java.util.Hashtable;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.io.printing.PrintTools;

public class SliderBoardConfigurationManager extends MidiSliderBoard
{
   protected final Hashtable<String, Hashtable> configurations = new Hashtable<String, Hashtable>();

   public SliderBoardConfigurationManager(SimulationConstructionSet scs)
   {
      super(scs);
   }

   public void saveConfiguration(String name)
   {
      if (configurations.containsKey(name))
      {
         System.err.println("slider board configuration: " + name + " is already saved");
      }
      else
      {
         configurations.put(name, (Hashtable<Integer, MidiControl>) controlsHashTable.clone());
         clearControls();
      }
   }

   public void loadConfiguration(String name)
   {
      PrintTools.info(this, "LOADING SLIDERBOARD CONFIGURATION: " + name);

      if (configurations.containsKey(name))
      {
         clearControls();
         addListOfControlls(configurations.get(name).values());
      }
      else
         System.err.println("slider board ocnfiguration: " + name + " does not exist");

   }

}
