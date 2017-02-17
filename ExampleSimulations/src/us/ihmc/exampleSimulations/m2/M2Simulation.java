package us.ihmc.exampleSimulations.m2;

import java.io.File;
import java.util.ArrayList;

import javax.swing.JOptionPane;

import us.ihmc.exampleSimulations.m2.Output.PerfectProcessedOutputs;
import us.ihmc.exampleSimulations.m2.Sensors.PerfectSensorProcessing;
import us.ihmc.exampleSimulations.m2.Sensors.ProcessedSensors;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationDoneListener;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.StateFileComparer;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;

public class M2Simulation implements SimulationDoneListener
{
   public static final double DT = 0.0002;

   public static boolean USE_HEAVY_M2 = false;
   boolean AUTOMATICALLY_RUN_AND_COMPARE = false;

   private SimulationConstructionSet sim;
   private M2Robot m2;

   public M2Simulation() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      M2Parameters littleM2Parameters = new LittleM2Parameters();
      M2Parameters bigM2Parameters = new BigM2Parameters();

      double ratio = 0.0;
      if (USE_HEAVY_M2)
         ratio = 1.0;

      M2Parameters m2Parameters = M2Parameters.mergeParameters(littleM2Parameters, bigM2Parameters, ratio);

      System.out.println("body mass = " + m2Parameters.BODY_MASS.value);


      m2 = new M2Robot(m2Parameters);

      // m2.createControllerBase(System.out, "M2");

//      System.out.println("total robot mass = " + m2.computeCenterOfMass(new Point3D()));

      double groundKxy;
      double groundBxy;
      double groundKz;
      double groundBz;
      if (USE_HEAVY_M2)
      {
         groundKxy = 40000.0;
         groundBxy = 250.0;
         groundKz = 500.0;
         groundBz = 100.0;
      }
      else
      {
         groundKxy = 40000.0;
         groundBxy = 100.0;
         groundKz = 40.0;
         groundBz = 500.0;
      }

      GroundContactModel linearGroundModel = new LinearGroundContactModel(m2, groundKxy, groundBxy, groundKz, groundBz, m2.getRobotsYoVariableRegistry());

      // HurosotGroundProfile profile = new HurosotGroundProfile(30.0, 1.5);
//    HurosotTerrain profile = new HurosotTerrain(30.0, 1.5);
//    linearGroundModel.setGroundProfile(profile);

      m2.setGroundContactModel(linearGroundModel);

      ProcessedSensors processedSensors = new ProcessedSensors();

      PerfectSensorProcessing perfectSensorProcessing = new PerfectSensorProcessing(m2, processedSensors);
      PerfectProcessedOutputs perfectProcessedOutputs = new PerfectProcessedOutputs(m2);

      M2ProcessedSensorsControllerWithStateTransitions controller = new M2ProcessedSensorsControllerWithStateTransitions(m2Parameters, processedSensors,
                                                                       perfectSensorProcessing, perfectProcessedOutputs, "m2ProcessedSensorsControllerWithStateTransitions");
      m2.setController(controller);

      PowerAndEnergyCalculator powerAndEnergyCalculator = new PowerAndEnergyCalculator(m2, "powerAndEnergyCalculator");
      m2.setController(powerAndEnergyCalculator);


      sim = new SimulationConstructionSet(m2);    // , false);

//      sim.addVarList(registry.createVarList());


//    sim.addStaticLink(profile.getLink());
      sim.setClipDistances(1.0, 100.0);

      // sim.setClipDistances(0.15, 20.0);
//    sim.setGroundVisible(false);

//    sim.setCameraFix(1.3, -21.7, 1.3);
//    sim.setCameraPosition(13.6, -36.4, 4.85);

      sim.setCameraFix(-0.5, -13.7, 0.7);
      sim.setCameraPosition(-0.5, -20.8, 0.7);

      sim.setCameraTracking(false, true, true, false);
      sim.setCameraDolly(false, true, true, false);

//    sim.setDT(0.0002, 100);
      sim.setDT(DT, 10);

      sim.setupVarGroup("kinematics", null, new String[] {"q_.*"});

      sim.setupGraphGroup("graph1", new String[][]
      {
         {"left_state", "right_state"}, {"q_pitch"}, {"qd_x", "xd"}, {"qd_y", "yd"}
      });

      sim.setupEntryBoxGroup("entry1", new String[] {"t_damp", "swing_time"});

      sim.setupConfiguration("config1", "kinematics", "graph1", "entry1");

      sim.selectConfiguration("config1");

      sim.setFastSimulate(true);

      powerAndEnergyCalculator.setUpGUI(sim);

      // sim.addSimulateDoneListener(this);
      // sim.setSimulateDoneCriterion(controller);

      Thread myThread = new Thread(sim);
      myThread.start();

      sleepForSeconds(2.0);


      if (AUTOMATICALLY_RUN_AND_COMPARE)
      {
         File dir = new File("StateComparison");
         if (!dir.exists())
         {
            dir.mkdir();
         }

         String baseFilename = "StateComparison/priorSimStateToMatch.state";
         String newFilename = "StateComparison/newSimStateToMatch.state";

         File file = new File(baseFilename);

         boolean writeBase = false;
         if (!file.exists())
         {
            JOptionPane.showMessageDialog(null, "Warning! " + baseFilename + " doesn't exist. Creating it. Sims will be the same. Run again to compare!");
            writeBase = true;
         }

         BlockingSimulationRunner runner = new BlockingSimulationRunner(sim, 600.0);

         // YoVariable t = sim.getVar("t");

         double runTime = 6.0;
         long startTime = System.currentTimeMillis();

         runner.simulateAndBlock(runTime);
         double elapsedTime = ((double) (System.currentTimeMillis() - startTime)) / 1000.0;
         double realTimeRate = runTime / elapsedTime;
         System.out.println("runTime = " + runTime + "   elapsed_time = " + elapsedTime + "   RealTimeRate = " + realTimeRate);

         if (writeBase)
         {
            sim.writeState(baseFilename);
         }

         sim.writeState(newFilename);

         ArrayList<VariableDifference> variableDifferences = StateFileComparer.absoluteCompareStateFiles(baseFilename, newFilename, 0.001, null);

         if (variableDifferences.isEmpty())
         {
            System.err.println("Sims are the same!");

            if (realTimeRate < 0.6)
            {
               System.err.println("Real Time Rate < 0.6!! Should be about 0.75. What happened?");
               JOptionPane.showMessageDialog(null, "Real Time Rate < 0.6!! It is " + realTimeRate + "! Should be about 0.75. What happened?");
            }
         }

         else
         {
            boolean xGotToSameValue = Math.abs(((DoubleYoVariable)sim.getVariable("q_x")).getDoubleValue() - 4.32420) < 1e-4;

            String message = "Sims are different. xGotToSameValue = " + xGotToSameValue + "\n" + VariableDifference.allVariableDifferencesToString(variableDifferences);
            System.err.println(message);
            JOptionPane.showMessageDialog(null, message);
         }
      }

      else
//         sim.simulate(1.646);
         sim.simulate(1.8);
   }

   public void sleepForSeconds(double sleepInSeconds)
   {
      try
      {
         Thread.sleep((long) (sleepInSeconds * 1000));
      }
      catch (InterruptedException e)
      {
      }
   }

   public void simulationDone()
   {
      System.out.println("Simulate Done");
   }

   public static void main(String[] args) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
	  new M2Simulation();
   }

   public void simulationDoneWithException(Throwable throwable)
   {
      simulationDone();
   }
}
