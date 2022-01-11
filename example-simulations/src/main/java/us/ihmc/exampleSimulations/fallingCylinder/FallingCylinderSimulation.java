package us.ihmc.exampleSimulations.fallingCylinder;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class FallingCylinderSimulation
{
   private static final double DT = 0.001;

   private enum GROUND_APPEARANCE
   {
      EARTH, STONE, ALUMINUM
   };

   SimulationConstructionSet sim;

   public FallingCylinderSimulation()
   {
      GROUND_APPEARANCE appearance = GROUND_APPEARANCE.EARTH;
      FallingCylinderRobot fallingCylinder = new FallingCylinderRobot();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(16342);
      sim = new SimulationConstructionSet(fallingCylinder, parameters);

      sim.setDT(DT, 20);

      sim.setCameraPosition(-1.5, -2.5, 0.5);
      sim.setCameraFix(0.0, 0.0, 0.4);

      sim.setCameraTracking(false, true, true, false);
      sim.setCameraDolly(false, true, true, false);

      // Set up some graphs:
      sim.setupGraph("q_z");

      sim.setupEntryBox("qd_x");
      sim.setupEntryBox("qd_y");
      sim.setupEntryBox("qd_z");

      sim.setupEntryBox("qd_wx");
      sim.setupEntryBox("qd_wy");
      sim.setupEntryBox("qd_wz");

      switch (appearance)
      {
         case EARTH:
            sim.setGroundAppearance(YoAppearance.EarthTexture());
            break;
         case STONE:
            sim.setGroundAppearance(YoAppearance.StoneTexture());
            break;
         case ALUMINUM:
            sim.setGroundAppearance(YoAppearance.AluminumMaterial());
            break;
      }
      Thread myThread = new Thread(sim);
      myThread.start();
   }
   
   public static void main(String[] args)
   {
      new FallingCylinderSimulation();
   }

}
