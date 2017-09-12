package us.ihmc.exampleSimulations.trebuchet;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.ButtonGroup;
import javax.swing.JRadioButton;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * Example SimulationConstructionSet simulation. Trebuchet siege engine.  
 *
 */
public class TrebuchetSimulation
{
   private SimulationConstructionSet simulationConstructionSet;

   public TrebuchetSimulation()
   {
      TrebuchetRobot trebuchet = new TrebuchetRobot();
      TrebuchetController trebuchetController = new TrebuchetController(trebuchet);
      trebuchet.setController(trebuchetController);

      simulationConstructionSet = new SimulationConstructionSet(trebuchet);

      setupCameras();
      createTrebuchetAndCastleViewButtons();

      setupGraphsAndEntryBoxes();

      simulationConstructionSet.setDT(0.001, 5);
      simulationConstructionSet.setSimulateDuration(15.0);

      simulationConstructionSet.startOnAThread();
   }

   private void setupCameras()
   {
      simulationConstructionSet.setClipDistances(1.0, 1000.0);

      simulationConstructionSet.setCameraFix(45.0, 0.0, 18.0);
      simulationConstructionSet.setCameraPosition(-64.0, 100.0, 8.0);

      simulationConstructionSet.setCameraTracking(false, true, true, true);
      simulationConstructionSet.setCameraDolly(false, true, true, true);

      simulationConstructionSet.setCameraTrackingVars("q_ball_x", "q_ball_y", "q_ball_z");
      simulationConstructionSet.setCameraDollyVars("q_ball_x", "q_ball_y", "q_ball_z");

      simulationConstructionSet.setCameraDollyOffsets(-30.0, -0.0, 10.0);
   }

   private void setupGraphsAndEntryBoxes()
   {
      simulationConstructionSet.setupGraph("q_ball_x");
      simulationConstructionSet.setupGraph("q_ball_z");
      simulationConstructionSet.setupGraph(new String[] { "qd_ball_x", "qd_ball_z" });

      simulationConstructionSet.setupEntryBox("q_x");
   }

   private void createTrebuchetAndCastleViewButtons()
   {
      JRadioButton trebuchetViewButton = new JRadioButton("Trebuchet View");
      trebuchetViewButton.setSelected(true);
      trebuchetViewButton.setActionCommand("Trebuchet View");
      trebuchetViewButton.setRequestFocusEnabled(false);

      JRadioButton castleViewButton = new JRadioButton("Castle View");
      castleViewButton.setActionCommand("Castle View");
      castleViewButton.setRequestFocusEnabled(false);

      ButtonGroup group = new ButtonGroup();
      group.add(trebuchetViewButton);
      group.add(castleViewButton);

      RadioListener myListener = new RadioListener();
      trebuchetViewButton.addActionListener(myListener);
      castleViewButton.addActionListener(myListener);

      simulationConstructionSet.addRadioButton(trebuchetViewButton);
      simulationConstructionSet.addRadioButton(castleViewButton);
   }

   public class RadioListener implements ActionListener
   {
      public void actionPerformed(ActionEvent e)
      {
         if (e.getActionCommand() == "Trebuchet View")
         {
            simulationConstructionSet.setCameraFix(45.0, 0.0, 18.0);
            simulationConstructionSet.setCameraPosition(-64.0, 100.0, 8.0);
         }
         else if (e.getActionCommand() == "Castle View")
         {
            simulationConstructionSet.setCameraFix(395, 0.0, 8.0);
            simulationConstructionSet.setCameraPosition(470.0, 0.0, 0.0);
         }
      }
   }

   public static void main(String[] args)
   {
      new TrebuchetSimulation();
   }
}
