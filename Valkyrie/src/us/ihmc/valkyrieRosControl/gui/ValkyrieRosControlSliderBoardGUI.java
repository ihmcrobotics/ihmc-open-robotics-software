package us.ihmc.valkyrieRosControl.gui;

import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.MidiSliderBoard;
import us.ihmc.valkyrie.visualizer.ValkyrieIpToNiceNameRemapper;

public class ValkyrieRosControlSliderBoardGUI extends SCSVisualizer
{
   public ValkyrieRosControlSliderBoardGUI(int bufferSize)
   {
      super(bufferSize);
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
      MidiSliderBoard sliderBoard = new MidiSliderBoard(scs);

      EnumYoVariable<?> selectedJoint = (EnumYoVariable<?>) registry.getVariable("ValkyrieRosControlSliderBoard", "selectedJoint");
      
      YoVariable<?> q_d = registry.getVariable("ValkyrieRosControlSliderBoard", "qDesiredSelected");
      YoVariable<?> qd_d = registry.getVariable("ValkyrieRosControlSliderBoard", "qdDesiredSelected");

      YoVariable<?> kp = registry.getVariable("ValkyrieRosControlSliderBoard", "kpSelected");
      YoVariable<?> kd = registry.getVariable("ValkyrieRosControlSliderBoard", "kdSelected");

      YoVariable<?> masterScaleFactor = registry.getVariable("ValkyrieRosControlSliderBoard", "masterScaleFactor");

      YoVariable<?> functionGeneratorPhase = registry.getVariable("SelectedYoFunGen", "SelectedPhase");
      YoVariable<?> functionGeneratorAmplitude = registry.getVariable("SelectedYoFunGen", "SelectedAmp");
      YoVariable<?> functionGeneratorFrequency = registry.getVariable("SelectedYoFunGen", "SelectedFreq");

      YoVariable<?> secondaryFunctionGeneratorPhase = registry.getVariable("SecondaryYoFunGen", "SecondaryPhase");
      YoVariable<?> secondaryFunctionGeneratorAmplitude = registry.getVariable("SecondaryYoFunGen", "SecondaryAmp");
      YoVariable<?> secondaryFunctionGeneratorFrequency = registry.getVariable("SecondaryYoFunGen", "SecondaryFreq");
      

      
      
      
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
      
      YoVariableClient client = new YoVariableClient(scsYoVariablesUpdatedListener, "remote", new ValkyrieIpToNiceNameRemapper());
      client.start();
   }
}
