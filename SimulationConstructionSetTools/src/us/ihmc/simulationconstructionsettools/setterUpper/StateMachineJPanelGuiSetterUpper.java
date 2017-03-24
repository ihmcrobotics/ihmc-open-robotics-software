package us.ihmc.simulationconstructionsettools.setterUpper;

import java.awt.Container;

import javax.swing.BoxLayout;
import javax.swing.JFrame;

import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachinesJPanel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class StateMachineJPanelGuiSetterUpper implements GUISetterUpper
{
   private final StateMachine stateMachine;
   private final boolean inJFrame;
   public StateMachineJPanelGuiSetterUpper(StateMachine stateMachine, boolean inJFrame)
   {
      this.stateMachine = stateMachine;
      this.inJFrame = inJFrame;
   }

   @Override
   public void setupGUI(SimulationConstructionSet scs)
   {
      StateMachinesJPanel walkingStatePanel = new StateMachinesJPanel(stateMachine);
      String name = "State Machine";
      if (inJFrame)
      {
         JFrame jFrame = new JFrame(name);
         Container contentPane = jFrame.getContentPane();
         contentPane.setLayout(new BoxLayout(contentPane, BoxLayout.X_AXIS));

         //ScrollPane pane = new ScrollPane();
         //pane.add(walkingStatePanel);
         jFrame.getContentPane().add(walkingStatePanel);
         jFrame.pack();
         jFrame.setSize(450, 300);
         jFrame.setAlwaysOnTop(true);
         jFrame.setVisible(true);
      }
      else
      {
         scs.addExtraJpanel(walkingStatePanel, name, false);         
      }

      // Doing the following will cause redraw when the state changes, but not
      // during replay or rewind:
      stateMachine.attachStateChangedListener(walkingStatePanel);
   }

}
