package us.ihmc.exampleSimulations.fallingSphere;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class FallingSphereSimulation
{
   SimulationConstructionSet sim;

   public FallingSphereSimulation()
   {
      FallingSphereRobot fallingSphereImpulse = new FallingSphereRobot("FallingSphereImpulse", true);
      FallingSphereRobot fallingSphereSpringDamper = new FallingSphereRobot("FallingSphereSpringDamper", false);

      FallingSphereController controller = new FallingSphereController(fallingSphereImpulse);
      fallingSphereImpulse.setController(controller);

      controller = new FallingSphereController(fallingSphereSpringDamper);
      fallingSphereSpringDamper.setController(controller);

      Robot[] robots = new Robot[] {fallingSphereImpulse, fallingSphereSpringDamper};
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(16000);
      sim = new SimulationConstructionSet(robots, parameters);

      sim.setDT(0.001, 1);

//    sim.setDT(0.001, 20);

      sim.setCameraPosition(-1.5, -2.5, 0.5);
      sim.setCameraFix(0.0, 0.0, 0.4);

      sim.setCameraTracking(false, true, true, false);
      sim.setCameraDolly(false, true, true, false);

      // Set up some graphs:

      // sim.setupGraph("q_z");
      sim.setupGraph("qd_x");
      sim.setupGraph("qd_y");
      sim.setupGraph("qd_z");

      sim.setupGraph("qd_wx");
      sim.setupGraph("qd_wy");
      sim.setupGraph("qd_wz");

      sim.setupGraph("energy");


      sim.setupEntryBox("qd_x");
      sim.setupEntryBox("qd_y");
      sim.setupEntryBox("qd_z");

      sim.setupEntryBox("qd_wx");
      sim.setupEntryBox("qd_wy");
      sim.setupEntryBox("qd_wz");



      // sim.setGroundAppearance(YoAppearance.EarthTexture(null));
      // sim.setGroundAppearance(YoAppearance.StoneTexture(null));

      // sim.setGroundAppearance(YoAppearance.AluminumMaterial());
      // sim.setGroundVisible(false);

      sim.startOnAThread();

   }

   public static void main(String[] args)
   {
      new FallingSphereSimulation();
   }
}
