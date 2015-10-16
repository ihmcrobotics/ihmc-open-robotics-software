package us.ihmc.valkyrieRosControl.gui;

import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.MidiSliderBoard;

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


      
      
      
      sliderBoard.setKnob(1, selectedJoint, 0, selectedJoint.getEnumSize());
      sliderBoard.setSlider(1, q_d, -Math.PI, Math.PI);
      sliderBoard.setSlider(2, qd_d, -Math.PI, Math.PI);
      sliderBoard.setSlider(3, kp, 0, 500);
      sliderBoard.setSlider(4, kd, 0, 10);
      
   }
   
   public static void main(String[] args)
   {
      SCSVisualizer scsYoVariablesUpdatedListener = new ValkyrieRosControlSliderBoardGUI(16384);
      scsYoVariablesUpdatedListener.setShowOverheadView(false);
      
      YoVariableClient client = new YoVariableClient(scsYoVariablesUpdatedListener, "remote");
      client.start();
   }
}
