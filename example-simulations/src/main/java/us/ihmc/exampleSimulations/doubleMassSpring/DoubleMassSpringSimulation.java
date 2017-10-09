package us.ihmc.exampleSimulations.doubleMassSpring;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class DoubleMassSpringSimulation
{

   public DoubleMassSpringSimulation()
   {
      DoubleMassSpringRobot robot = new DoubleMassSpringRobot();
      DoubleMassSpringController controller = new DoubleMassSpringController(robot);

      robot.setController(controller);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 100);
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new DoubleMassSpringSimulation();
   }
}
