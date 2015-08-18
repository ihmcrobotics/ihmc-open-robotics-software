package us.ihmc.valkyrie.visualizer;
/**
 * A simple extension to vanilla SCS GUI with a button findCOM to pass the data between InOut to LinkCom optimizer.
 */


import javax.swing.JButton;

import us.ihmc.CoM.LinkComIDActionListener;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieSystemIdentificationSCSUI
{
   public static void main(String[] args)
   {
     
      final SimulationConstructionSet scs;

      // initialize SCS
     DRCRobotModel robotModel = new ValkyrieRobotModel(true,false);
      final SDFRobot robot =robotModel.createSdfRobot(false);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(65536);
      scs = new SimulationConstructionSet(robot, parameters);

      // add sysid button
      JButton button = new JButton("findCOM");
      button.addActionListener(new LinkComIDActionListener(scs.getDataBuffer(), robot));
      scs.addButton(button);
      scs.startOnAThread();

   }
}
