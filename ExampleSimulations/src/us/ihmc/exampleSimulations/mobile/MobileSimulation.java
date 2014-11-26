package us.ihmc.exampleSimulations.mobile;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * A simulation of a child's mobile toy that uses a tree structure of 21 gimbal
 * joints (63 degrees of freedom total).
 */
public class MobileSimulation
{
   private SimulationConstructionSet sim;

   public MobileSimulation()
   {
	  // Create an instance of MobileRobot
      MobileRobot mobile = new MobileRobot();

      // Instantiate a SCS object using the MobileRobot object reference
      sim = new SimulationConstructionSet(mobile);
      sim.setGroundVisible(false);

      sim.setCameraTracking(false, false, false, false);
      sim.setCameraDolly(false, false, false, false);

      // set camera to a convenient viewing angle
      sim.setCameraPosition(1.0, 1.0, 0.5);
      sim.setCameraFix(0.0, 0.0, 0.8);

      sim.setCameraTrackingVars("ef_track00_x", "ef_track00_y", "ef_track00_z");

      sim.setDT(0.02, 1);
      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new MobileSimulation();
   }
}
