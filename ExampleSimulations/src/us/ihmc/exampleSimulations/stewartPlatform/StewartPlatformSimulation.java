package us.ihmc.exampleSimulations.stewartPlatform;

//import javax.swing.*;
//import java.awt.event.*;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;


public class StewartPlatformSimulation
{
   private SimulationConstructionSet sim;

   public StewartPlatformSimulation()
   {
      StewartPlatformRobot StewartPlatform = new StewartPlatformRobot("StewartPlatform");
      StewartPlatform.setController(new StewartPlatformController(StewartPlatform, "stewartPlatformSimulation"));
      sim = new SimulationConstructionSet(StewartPlatform);

      StewartPlatform.setGravity(0.0);    // -9.81);

      // StewartPlatform View:
      // sim.setCameraFix(45.0,0.0,18.0);
      // sim.setCameraPosition(-64.0,100.0,8.0);

      sim.setDT(0.0004, 50);

      sim.setCameraTracking(false, true, true, false);
      sim.setCameraDolly(false, true, true, false);


      // Set up some graphs and entry boxes:

      sim.setupGraph(new String[] {"q_platform_x", "q_d_x"});
      sim.setupGraph(new String[] {"q_platform_y", "q_d_y"});
      sim.setupGraph(new String[] {"q_platform_z", "q_d_z"});

      sim.setupGraph(new String[] {"q_yaw", "q_d_yaw"});
      sim.setupGraph(new String[] {"q_roll", "q_d_roll"});
      sim.setupGraph(new String[] {"q_pitch", "q_d_pitch"});

      // sim.setupGraph("q_ball_z");
      // sim.setupGraph(new String[]{"qd_ball_x", "qd_ball_z"} );

      // sim.setupEntryBox("k_loop");
      // sim.setupEntryBox("b_loop");
      // sim.setupEntryBox("Fx");
      // sim.setupEntryBox("Fy");
      // sim.setupEntryBox("Fz");

      // sim.setupEntryBox("q_d_x");sim.setupEntryBox("q_d_y");sim.setupEntryBox("q_d_z");
      // sim.setupEntryBox("k_x");sim.setupEntryBox("k_y");sim.setupEntryBox("k_z");
      // sim.setupEntryBox("b_x");sim.setupEntryBox("b_y");sim.setupEntryBox("b_z");

      sim.setupEntryBox("x_amp");
      sim.setupEntryBox("y_amp");
      sim.setupEntryBox("z_amp");
      sim.setupEntryBox("x_freq");
      sim.setupEntryBox("y_freq");
      sim.setupEntryBox("z_freq");
      sim.setupEntryBox("y_phase");

      // Start the simulation
      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new StewartPlatformSimulation();
   }

}
