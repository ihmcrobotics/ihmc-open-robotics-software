package us.ihmc.exampleSimulations.unicycle;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class UnicycleSimulation
{
   private SimulationConstructionSet scs;

   public UnicycleSimulation()
   {
      UnicycleRobot unicycleRobot = new UnicycleRobot();
      unicycleRobot.setController(new UnicycleController(unicycleRobot, "UnicycleController"));

      scs = new SimulationConstructionSet(unicycleRobot);
      scs.setDT(0.001, 10);
      scs.setMaxBufferSize(16384*4);

      Thread myThread = new Thread(scs);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new UnicycleSimulation();
   }
}
