package us.ihmc.exampleSimulations.jointLimits;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class JointLimitsSimulation
{
   private static final double simulateDT = 0.0001;
   private static final int recordFrequency = 10;

   public JointLimitsSimulation()
   {
      JointLimitsRobot robot = new JointLimitsRobot();
      robot.setController(new JointLimitsController(robot, simulateDT));

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(64000);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setDT(simulateDT, recordFrequency);
      scs.setGroundVisible(true);

      Thread simThread = new Thread(scs);
      simThread.start();
   }

   public static void main(String[] args)
   {
      new JointLimitsSimulation();
   }
}
