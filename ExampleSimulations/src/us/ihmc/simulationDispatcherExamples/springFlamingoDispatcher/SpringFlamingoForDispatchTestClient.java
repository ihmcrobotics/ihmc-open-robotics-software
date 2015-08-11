package us.ihmc.simulationDispatcherExamples.springFlamingoDispatcher;

import java.rmi.Naming;
import java.rmi.RMISecurityManager;

import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.DispatchedSimulationDescription;
import us.ihmc.simulationconstructionset.simulationDispatcher.interfaces.RemoteSimulationRunnerInterface;

public class SpringFlamingoForDispatchTestClient
{
   private String pwd = "***REMOVED***";

   public SpringFlamingoForDispatchTestClient()
   {
   }


   public void test()
   {
      SpringFlamingoForDispatchRobot flam = new SpringFlamingoForDispatchRobot();
      Simulation sim = new Simulation(flam, 2);

      DispatchedSimulationDescription description = new DispatchedSimulationDescription(new SpringFlamingoForDispatchSimulationConstructor(), new String[] {"t", "v1"},
                                                       new String[] {"q_x",
              "q_z"});    // flam));

      System.out.println("Dispatching Simulation. ");

      RemoteSimulationRunnerInterface remoteSim = null;

      System.setSecurityManager(new RMISecurityManager());

      // description.createSimulation();description.startSimulation();if(true)return;

      try
      {
         // remoteSim = (RemoteSimulationRunnerInterface) Naming.lookup("//robotica/RemoteSimulationRunner");

         System.out.println("Looking up RemoteSimulationRunner.");
         System.out.flush();
         remoteSim = (RemoteSimulationRunnerInterface) Naming.lookup("//robotica/RemoteSimulationRunner");

         System.out.println("Creating the Simulation.");
         System.out.flush();
         remoteSim.createSimulation(description, null, null, pwd);

         System.out.println("Setting the Simulation State.");
         System.out.flush();
         remoteSim.setSimulationState(new double[] {10.2, 1.3}, pwd);

         System.out.println("Running the Simulation.");
         System.out.flush();
         remoteSim.startSimulation(pwd);


         System.out.println("Waiting for the Simulation to Finish.");
         System.out.flush();

         while (!remoteSim.isSimulationDone(pwd))
         {
            try
            {
               Thread.sleep(1000);
            }
            catch (InterruptedException e)
            {
            }
         }

         System.out.println("Done Running the Simulation.");
         System.out.flush();

         System.out.println("Getting the Simulation State.");
         System.out.flush();
         double[] finalState = (double[]) remoteSim.getSimulationState(pwd);

         System.out.println("Final State: " + finalState);
         System.out.flush();

         // System.out.println("Looking up RemoteSimulationRunner again.");
         // remoteSim = (RemoteSimulationRunnerInterface) Naming.lookup("//robotica/RemoteSimulationRunner");

         System.out.println("Creating another Simulation.");
         System.out.flush();
         remoteSim.createSimulation(description, null, null, pwd);

         System.out.println("Setting the other Simulation State.");
         System.out.flush();
         remoteSim.setSimulationState(new double[] {10.4, 1.4}, pwd);

         System.out.println("Running the other Simulation.");
         System.out.flush();
         remoteSim.startSimulation(pwd);

         System.out.println("Waiting for the Simulation to Finish.");
         System.out.flush();

         while (!remoteSim.isSimulationDone(pwd))
         {
            try
            {
               Thread.sleep(1000);
            }
            catch (InterruptedException e)
            {
            }
         }

         System.out.println("Done Running the Second Simulation.");
         System.out.flush();


         System.out.println("Getting the Simulation State.");
         System.out.flush();
         finalState = (double[]) remoteSim.getSimulationState(pwd);

         System.out.println("Final State: " + finalState);
         System.out.flush();
      }
      catch (Exception e)
      {
         System.out.println("Exception in TestClient:  " + e.getMessage());
         e.printStackTrace();
      }

   }

   public static void main(String[] args)
   {
      SpringFlamingoForDispatchTestClient test = new SpringFlamingoForDispatchTestClient();

      test.test();


   }

}
