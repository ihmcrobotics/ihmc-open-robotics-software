package us.ihmc.valkyrie.visualizer;
/**
 * A simple extension to vanilla SCS GUI with a button findCOM to pass the data between InOut to LinkCom optimizer.
 */


import javax.swing.JButton;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.systemIdentification.com.LinkComIDActionListener;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieSystemIdentificationSCSUI
{
   public static void main(String[] args)
   {
     
      final SimulationConstructionSet scs;

      // initialize SCS
     DRCRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.REAL_ROBOT, false);
      final FloatingRootJointRobot robot =robotModel.createHumanoidFloatingRootJointRobot(false);
      
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
