package us.ihmc.simulationDispatcherExamples.springFlamingoDispatcher;

import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.SimulationConstructor;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.DispatchDoneListener;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationDispatcher;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationToDispatch;
import us.ihmc.utilities.io.files.FileTools;

public class FlamingoDispatcher
{
   private static final boolean DISPATCH_FLAMINGO_SIMS = true;
   private static final boolean DISPATCH_M2_SIMS = true;
   
   private static boolean RUN_SIMULATIONS_LOCALLY = false;
   private static boolean DISPATCH_SIMULATIONS = true;

   public FlamingoDispatcher()
   {
   }

   public static void main(String args[])
   {
      // Note: The codeBase is always the trickiest part. Sometimes using bengal svn works well, but sometimes due to 
      // Certificates or what not, it doesn't. If not, upload the .jar file onto an ihmc groups page and use that link.
      
      // String myCodeBase = "file:///D:/JerryPrattDocuments/SoftwareTools/SimulationDispatcher/FlamingoDispatcher/SpringFlamingo/springflamingodispatcher11.jar";
      
      // Access at: http://ihmc.us/groups/jpratt/wiki/415b6/test.html
//      String myCodeBase = "http://ihmc.us/groups/jpratt/wiki/415b6/attachments/1e894/FlamingoDispatcher.jar";
//      String myCodeBase = "https://confluence.ihmc.us/download/attachments/38469830/FlamingoDispatcher.jar";
      String myCodeBase = "https://sharebox.ihmc.us/public.php?service=files&t=ad151648d89b032bfd19271830e84b2e&download";

      String[] hostNames = new String[]
            {
//            "unknownpw-PC:8", 
//            "gazelle:2", 
            "Cheetah:2"
            };

      SimulationDispatcher dispatcher = new SimulationDispatcher(hostNames, myCodeBase);

      // Creating M2 Simulation
      M2SimulationConstructor m2SimulationConstructor1 = new M2SimulationConstructor();

      // Create Flamingo Simulation
      SpringFlamingoForDispatchSimulationConstructor flamingoSimulationConstructor1 = new SpringFlamingoForDispatchSimulationConstructor();
      if (!FileTools.checkIfSerializable(flamingoSimulationConstructor1)) 
      {
         System.exit(-1);
      }
         
      Simulation flamSim = flamingoSimulationConstructor1.constructSimulation(null, null);

      if (RUN_SIMULATIONS_LOCALLY)
      {
         m2SimulationConstructor1.setupAndLaunchSCS();
      }

      else if (DISPATCH_SIMULATIONS)
      {
         Simulation m2Sim = m2SimulationConstructor1.constructSimulation(null, null);
         m2SimulationConstructor1.setupGraphics(m2Sim);

         String[] flamInputStateVars = new String[] {"t", "q_x"};
         String[] flamOutputStateVars = new String[]
         {
            "t", "q_x", "q_z", "q_pitch", "q_lh", "q_lk", "q_la", "q_rh", "q_rk", "q_ra"
         };


         String[] m2InputStateVars = new String[] {"t", "q_x"};
         String[] m2OutputStateVars = new String[]
         {
            "t", "q_x", "q_y", "q_z", "q_qs", "q_qx", "q_qy", "q_qz", "q_left_hip_pitch", "q_left_knee", "q_left_ankle_pitch", "q_right_hip_pitch",
            "q_right_knee", "q_right_ankle_pitch"
         };


         DispatchDoneListener flamListener = new DispatchDoneListener()
         {
            public void dispatchDone(SimulationToDispatch dispatchSim, double[] finalState)
            {
               // System.out.println("Another Sim Done1");
               System.out.println("Flamingo Final state: t=" + finalState[0] + " q_x=" + finalState[1] + " q_z=" + finalState[2] + " q_pitch=" + finalState[3]);
            }
         };

         DispatchDoneListener m2Listener = new DispatchDoneListener()
         {
            public void dispatchDone(SimulationToDispatch dispatchSim, double[] finalState)
            {
               // System.out.println("Another Sim Done1");
               System.out.println("M2 Final state: t=" + finalState[0] + " q_x=" + finalState[1] + " q_z=" + finalState[2] + " q_pitch=" + finalState[3]);
            }
         };


         int j = 0;

         SimulationConstructor flamingoSimulationConstructor = new SpringFlamingoForDispatchSimulationConstructor();
         SimulationConstructor m2SimulationConstructor = new M2SimulationConstructor();

         while (true)
         {
            for (int i = 0; i < 10; i++)
            {
               if (DISPATCH_FLAMINGO_SIMS)
               {
                  double[] newFlamVals = new double[] {10.0 + Math.random(), 0.9, 3.3};
                  SimulationToDispatch dispatchFlamSim = new SimulationToDispatch(flamSim, "flamingo " + j + "_" + i, flamingoSimulationConstructor,
                        null, null,
                        flamInputStateVars, newFlamVals, flamOutputStateVars, flamListener);
                  dispatcher.addSimulation(dispatchFlamSim);
               }

               if (DISPATCH_M2_SIMS)
               {
                  double[] newM2Vals = new double[] {0.0, 0.0, 3.3};
                  SimulationToDispatch dispatchM2Sim = new SimulationToDispatch(m2Sim, "M2 " + j + "_" + i, m2SimulationConstructor, 
                        null, null,
                        m2InputStateVars, newM2Vals, m2OutputStateVars, m2Listener);
                  dispatcher.addSimulation(dispatchM2Sim);
               }
            }

            j++;

            // dispatcher.dispatchSimulations();

            while (dispatcher.getNumberSimulationsToDispatch() > 10)
            {
               try
               {
                  Thread.sleep(30000);
               }
               catch (InterruptedException e)
               {
               }
            }
         }
      }

   }

}
