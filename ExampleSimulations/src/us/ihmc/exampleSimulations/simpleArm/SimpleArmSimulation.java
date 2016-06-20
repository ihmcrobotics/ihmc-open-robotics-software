package us.ihmc.exampleSimulations.simpleArm;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SimpleArmSimulation
{
   private SimulationConstructionSet scs;

   public SimpleArmSimulation()
   {
      SimpleArmRobot simpleArmRobot = new SimpleArmRobot();
      simpleArmRobot.setController(new SimpleArmController(simpleArmRobot, "ArmController"));

      scs = new SimulationConstructionSet(simpleArmRobot);
      scs.setDT(0.001, 20);

      Thread myThread = new Thread(scs);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new SimpleArmSimulation();
   }
}
