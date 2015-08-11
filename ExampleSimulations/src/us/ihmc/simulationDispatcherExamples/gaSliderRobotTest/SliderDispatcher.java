package us.ihmc.simulationDispatcherExamples.gaSliderRobotTest;

import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.SimulationConstructor;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.DispatchDoneListener;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationDispatcher;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationToDispatch;

public class SliderDispatcher
{
   public SliderDispatcher()
   {
   }

   public static void main(String args[])
   {      
      // Note: The codeBase is always the trickiest part. Sometimes using bengal svn works well, but sometimes due to 
      // Certificates or what not, it doesn't. If not, upload the .jar file onto an ihmc groups page and use that link.
//      String myCodeBase = "https://bengal.ihmc.us/svn/SimulationDispatcher/GASliderRobotTest/GASliderRobotDispatcher05.jar";
      String myCodeBase = "http://ihmc.us/groups/jpratt/wiki/415b6/attachments/26ff7/GASliderRobotDispatcher05.jar";
      
      String[] hostNames = new String[] {"gazelle.ihmc.us", "cheetah", "dalmatian", "unknownpw-PC"};

      SimulationDispatcher dispatcher = new SimulationDispatcher(hostNames, myCodeBase);

      // Create Flamingo Simulation
      SliderRobot sliderRobot = new SliderRobot(null, null);
      SliderController controller = new SliderController(sliderRobot);
      sliderRobot.setController(controller);

      // SimulationConstructionSet sim = null;
      Simulation sliderSim = new Simulation(sliderRobot, 1);

      SimulationConstructor sliderConstructor = new SliderSimulationConstructor();
      String[] sliderInputStateVars = new String[]
      {
         "k1", "k2", "k3", "k4", "q_joint1", "q_joint2"
      };
      String[] sliderOutputStateVars = new String[]
      {
         "t", "q_joint1", "q_joint2", "qd_joint1", "qd_joint2", "k1", "k2", "k3", "k4"
      };

      DispatchDoneListener sliderListener = new DispatchDoneListener()
      {
         public void dispatchDone(SimulationToDispatch dispatchSim, double[] finalState)
         {
            // System.out.println("Another Sim Done1");
            System.out.print("Slider Final state: t=" + finalState[0] + " q_joint1=" + finalState[1] + " q_joint2=" + finalState[2]);
            System.out.print(" qd_joint1=" + finalState[3] + " qd_joint2=" + finalState[4]);
            System.out.println(" k1=" + finalState[5] + " k2=" + finalState[6] + " k3=" + finalState[7] + " k4=" + finalState[8]);
         }
      };


      SimulationToDispatch dispatchSliderSim;

      int j = 0;

      while (true)
      {
         for (int i = 0; i < 40; i++)
         {
            double[] newSliderVals = new double[]
            {
               -200 * Math.random(), -200 * Math.random(), -200 * Math.random(), -200 * Math.random(), 0.25 * (-1.0 + 2.0 * Math.random()),
               0.25 * (-1.0 + 2.0 * Math.random())
            };

            dispatchSliderSim = new SimulationToDispatch(sliderSim, "slider " + j + "_" + i, sliderConstructor, 
                  null, null,
                  sliderInputStateVars, newSliderVals,
                  sliderOutputStateVars, sliderListener);
            dispatcher.addSimulation(dispatchSliderSim);
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
