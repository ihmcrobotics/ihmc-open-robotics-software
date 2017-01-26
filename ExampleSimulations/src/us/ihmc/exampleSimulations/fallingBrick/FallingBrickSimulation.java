package us.ihmc.exampleSimulations.fallingBrick;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.examples.FallingBrickRobot;

public class FallingBrickSimulation
{
   private enum GROUND_APPEARANCE
   {
      EARTH, STONE, ALUMINUM
   };

   SimulationConstructionSet sim;

   public FallingBrickSimulation()
   {
      GROUND_APPEARANCE appearance = GROUND_APPEARANCE.EARTH;
      FallingBrickRobot FallingBrick = new FallingBrickRobot();
      //      sim = new SimulationConstructionSet(FallingBrick, new JMEGraphics3dAdapter(), 16342);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(16342);
      sim = new SimulationConstructionSet(FallingBrick, parameters);

      sim.setDT(0.001, 20);

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
      new FallingBrickSimulation();
   }
}