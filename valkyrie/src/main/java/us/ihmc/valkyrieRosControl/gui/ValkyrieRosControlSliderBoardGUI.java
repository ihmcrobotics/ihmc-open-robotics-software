package us.ihmc.valkyrieRosControl.gui;

import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.MidiSliderBoard;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class ValkyrieRosControlSliderBoardGUI extends SCSVisualizer
{
   public ValkyrieRosControlSliderBoardGUI(int bufferSize)
   {
      super(bufferSize);
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoRegistry registry)
   {
      MidiSliderBoard sliderBoard = new MidiSliderBoard(scs);

      YoEnum<?> selectedJoint = (YoEnum<?>) registry.findVariable("ValkyrieRosControlSliderBoard", "selectedJoint");
      
      YoVariable q_d = registry.findVariable("ValkyrieRosControlSliderBoard", "qDesiredSelected");
      YoVariable qd_d = registry.findVariable("ValkyrieRosControlSliderBoard", "qdDesiredSelected");

      YoVariable kp = registry.findVariable("ValkyrieRosControlSliderBoard", "kpSelected");
      YoVariable kd = registry.findVariable("ValkyrieRosControlSliderBoard", "kdSelected");

      YoVariable masterScaleFactor = registry.findVariable("ValkyrieRosControlSliderBoard", "masterScaleFactor");

      YoVariable functionGeneratorPhase = registry.findVariable("SelectedYoFunGen", "SelectedPhase");
      YoVariable functionGeneratorAmplitude = registry.findVariable("SelectedYoFunGen", "SelectedAmp");
      YoVariable functionGeneratorFrequency = registry.findVariable("SelectedYoFunGen", "SelectedFreq");

      YoVariable secondaryFunctionGeneratorPhase = registry.findVariable("SecondaryYoFunGen", "SecondaryPhase");
      YoVariable secondaryFunctionGeneratorAmplitude = registry.findVariable("SecondaryYoFunGen", "SecondaryAmp");
      YoVariable secondaryFunctionGeneratorFrequency = registry.findVariable("SecondaryYoFunGen", "SecondaryFreq");
      

      
      
      
      sliderBoard.setKnob(1, selectedJoint, 0, selectedJoint.getEnumSize());
      sliderBoard.setSlider(1, q_d, -Math.PI, Math.PI);
      sliderBoard.setSlider(2, qd_d, -Math.PI, Math.PI);
      sliderBoard.setSlider(3, kp, 0, 500);
      sliderBoard.setSlider(4, kd, 0, 10);
      sliderBoard.setSlider(8, masterScaleFactor, 0.0, 1.0);
      
      sliderBoard.setSlider(5, functionGeneratorPhase, 0.0, Math.PI);
      sliderBoard.setSlider(6, functionGeneratorAmplitude, 0.0, 10.0);
      sliderBoard.setSlider(7, functionGeneratorFrequency, 0.0, 20.0);

      sliderBoard.setKnob(5, secondaryFunctionGeneratorPhase, 0.0, Math.PI);
      sliderBoard.setKnob(6, secondaryFunctionGeneratorAmplitude, 0.0, 10.0);
      sliderBoard.setKnob(7, secondaryFunctionGeneratorFrequency, 0.0, 20.0);
   }
   
   public static void main(String[] args)
   {
      SCSVisualizer scsYoVariablesUpdatedListener = new ValkyrieRosControlSliderBoardGUI(16384);
      scsYoVariablesUpdatedListener.setShowOverheadView(false);
      
      YoVariableClient client = new YoVariableClient(scsYoVariablesUpdatedListener);
      client.startWithHostSelector();
   }
}
