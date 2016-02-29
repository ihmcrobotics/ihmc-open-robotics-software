package us.ihmc.exampleSimulations.mobile;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * A simulation of a child's mobile toy that uses a tree structure of 21 gimbal
 * joints (63 degrees of freedom total).
 */
public class MobileSimulation
{
   private SimulationConstructionSet simulationConstructionSet;

   public MobileSimulation()
   {
      // Create an instance of MobileRobot
      MobileRobot mobile = new MobileRobot();

      // Instantiate a SCS object using the MobileRobot object reference
      simulationConstructionSet = new SimulationConstructionSet(mobile);
      simulationConstructionSet.setGroundVisible(false);

      simulationConstructionSet.setCameraTracking(false, false, false, false);
      simulationConstructionSet.setCameraDolly(false, false, false, false);

      // set camera to a convenient viewing angle
      simulationConstructionSet.setCameraPosition(1.0, 1.0, 0.5);
      simulationConstructionSet.setCameraFix(0.0, 0.0, 0.8);

      simulationConstructionSet.setCameraTrackingVars("ef_track00_x", "ef_track00_y", "ef_track00_z");

      simulationConstructionSet.setDT(0.02, 1);
      simulationConstructionSet.startOnAThread();
   }

   public static void main(String[] args)
   {
      new MobileSimulation();
   }
}
