package us.ihmc.valkyrie.simulation;

import us.ihmc.tools.processManagement.JavaProcessSpawner;
import us.ihmc.tools.processManagement.RestartableProcess;

public class ValkyrieExperimentalPhysicsSimulationProcess implements RestartableProcess
{
   private final JavaProcessSpawner javaProcessSpawner = new JavaProcessSpawner(true);
   private boolean exited = false;
   private Process spawnedProcess;

   @Override
   public void start()
   {
      exited = false;
      String[] jvmArgs = new String[0];
      String[] programArgs = new String[] {"headless"};
      spawnedProcess = javaProcessSpawner.spawn(ValkyriePlanarRegionPositionControlSimulation.class, jvmArgs, programArgs, statusValue ->
      {
         exited = true;
      });
   }

   @Override
   public void stop()
   {
      try
      {
         spawnedProcess.destroyForcibly().waitFor();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public String getName()
   {
      return "Experimental physics simulation";
   }
}
