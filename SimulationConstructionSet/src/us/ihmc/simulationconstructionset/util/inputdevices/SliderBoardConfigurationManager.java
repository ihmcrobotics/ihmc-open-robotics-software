package us.ihmc.simulationconstructionset.util.inputdevices;

import java.util.Hashtable;

import us.ihmc.commons.PrintTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

@SuppressWarnings({"rawtypes", "unchecked"})
public class SliderBoardConfigurationManager extends MidiSliderBoard
{
   protected final Hashtable<String, Hashtable> configurations = new Hashtable<String, Hashtable>();

   public SliderBoardConfigurationManager(SimulationConstructionSet scs, boolean useVirtualSliderBoard, boolean showSliderboardVisualizer)
   {
      super(scs, useVirtualSliderBoard, showSliderboardVisualizer);
   }

   public SliderBoardConfigurationManager(SimulationConstructionSet scs, boolean useVirtualSliderBoard)
   {
      super(scs, useVirtualSliderBoard);
   }

   public SliderBoardConfigurationManager(SimulationConstructionSet scs)
   {
      super(scs);
   }

   public void saveConfiguration(String name)
   {
      if (configurations.containsKey(name))
      {
         System.err.println("SliderBoardConfigurationManager: " + name + " is already saved");
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
         addListOfControls(configurations.get(name).values());
         super.setTitle(name);
      }
      else
      {
         System.err.println("SliderBoardConfigurationManager: " + name + " does not exist");
      }
   }
}
