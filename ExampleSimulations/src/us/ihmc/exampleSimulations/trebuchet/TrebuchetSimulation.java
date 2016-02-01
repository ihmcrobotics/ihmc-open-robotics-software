package us.ihmc.exampleSimulations.trebuchet;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.ButtonGroup;
import javax.swing.JRadioButton;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;


public class TrebuchetSimulation
{
   private SimulationConstructionSet sim;

   public TrebuchetSimulation()
   {
      TrebuchetRobot trebuchet = new TrebuchetRobot();

      sim = new SimulationConstructionSet(trebuchet);

      // Trebuchet View:
      sim.setCameraFix(45.0, 0.0, 18.0);
      sim.setCameraPosition(-64.0, 100.0, 8.0);

      sim.setDT(0.001, 5);

      sim.setCameraTracking(false, true, true, true);
      sim.setCameraDolly(false, true, true, true);

      sim.setCameraTrackingVars("q_ball_x", "q_ball_y", "q_ball_z");
      sim.setCameraDollyVars("q_ball_x", "q_ball_y", "q_ball_z");

      sim.setCameraDollyOffsets(-30.0, -0.0, 10.0);

      // Add some buttons:

      JRadioButton trebView = new JRadioButton("Treb View");
      trebView.setSelected(true);
      trebView.setActionCommand("Treb View");
      trebView.setRequestFocusEnabled(false);

      JRadioButton castleView = new JRadioButton("Castle View");
      castleView.setActionCommand("Castle View");
      castleView.setRequestFocusEnabled(false);

      ButtonGroup group = new ButtonGroup();
      group.add(trebView);
      group.add(castleView);

      RadioListener myListener = new RadioListener();
      trebView.addActionListener(myListener);
      castleView.addActionListener(myListener);

      sim.addRadioButton(trebView);
      sim.addRadioButton(castleView);

      sim.setClipDistances(1.0, 1000.0);
      sim.setSimulateDuration(15.0);

      // Set up some graphs and entry boxes:

      sim.setupGraph("q_ball_x");
      sim.setupGraph("q_ball_z");
      sim.setupGraph(new String[] {"qd_ball_x", "qd_ball_z"});

      sim.setupEntryBox("q_x");

      // Start the simulation
      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new TrebuchetSimulation();
   }

   public class RadioListener implements ActionListener
   {
      public void actionPerformed(ActionEvent e)
      {
         if (e.getActionCommand() == "Treb View")
         {
            sim.setCameraFix(45.0, 0.0, 18.0);
            sim.setCameraPosition(-64.0, 100.0, 8.0);
         }
         else if (e.getActionCommand() == "Castle View")
         {
            sim.setCameraFix(395, 0.0, 8.0);
            sim.setCameraPosition(470.0, 0.0, 0.0);
         }
      }
   }
}
