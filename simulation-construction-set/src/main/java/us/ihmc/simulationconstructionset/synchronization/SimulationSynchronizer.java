package us.ihmc.simulationconstructionset.synchronization;

public class SimulationSynchronizer
{
   private static boolean createdOne = false;

   public SimulationSynchronizer()
   {
      if (createdOne)
      {
//         String errorMessage = "Should only ever create one SimulationSynchronizer in order for everything to stay synchronized!";
//         System.err.println(errorMessage);

         // throw new RuntimeException(errorMessage);
      }

      createdOne = true;

   }
}
