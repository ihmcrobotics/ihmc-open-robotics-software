package us.ihmc.exampleSimulations.doubleMassSpring;

import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;

public class DoubleMassSpringSimulation
{

   public DoubleMassSpringSimulation()
   {
      DoubleMassSpringRobotDefinition robotDefinition = new DoubleMassSpringRobotDefinition();

      SimulationSession simulationSession = new SimulationSession();
      simulationSession.addRobot(robotDefinition);

      SimulationSessionControls simulationSessionControls = simulationSession.getSimulationSessionControls();
      simulationSessionControls.setDT(0.0001);
      simulationSessionControls.setBufferRecordTickPeriod(100);

      SessionVisualizer.startSessionVisualizer(simulationSession);
   }

   public static void main(String[] args)
   {
      new DoubleMassSpringSimulation();
   }
}
