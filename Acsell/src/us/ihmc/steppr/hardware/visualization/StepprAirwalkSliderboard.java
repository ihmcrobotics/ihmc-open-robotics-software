package us.ihmc.steppr.hardware.visualization;

import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionsettools.util.inputdevices.MidiSliderBoard;
import us.ihmc.steppr.hardware.StepprDashboard;

public class StepprAirwalkSliderboard extends SCSVisualizer
{
   public StepprAirwalkSliderboard(int bufferSize)
   {
      super(bufferSize);
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)

   {
      StepprDashboard.createDashboard(scs, registry);
      MidiSliderBoard sliderBoard = new MidiSliderBoard(scs);

      YoVariable<?> hipAmplitude = registry.getVariable("StepprAirwalk", "hipAmplitude");
      YoVariable<?> hipFrequency = registry.getVariable("StepprAirwalk", "hipFrequency");
      YoVariable<?> kneeAmplitude = registry.getVariable("StepprAirwalk", "kneeAmplitude");
      YoVariable<?> kneeFrequency = registry.getVariable("StepprAirwalk", "kneeFrequency");
      YoVariable<?> ankleAmplitude = registry.getVariable("StepprAirwalk", "ankleAmplitude");
      YoVariable<?> ankleFrequency = registry.getVariable("StepprAirwalk", "ankleFrequency");
      
      sliderBoard.setSlider(1, hipAmplitude, 0, 1);
      sliderBoard.setSlider(2, hipFrequency, 0, 1);
      sliderBoard.setSlider(3, kneeAmplitude, 0, 1);
      sliderBoard.setSlider(4, kneeFrequency, 0, 1);
      sliderBoard.setSlider(5, ankleAmplitude, 0, 1);
      sliderBoard.setSlider(6, ankleFrequency, 0, 1);
      
   }
   
   public static void main(String[] args)
   {
      SCSVisualizer scsYoVariablesUpdatedListener = new StepprAirwalkSliderboard(16384);
      scsYoVariablesUpdatedListener.setShowOverheadView(false);
      
      YoVariableClient client = new YoVariableClient(scsYoVariablesUpdatedListener, "remote");
      client.start();
   }
}
