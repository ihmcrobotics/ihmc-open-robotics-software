package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class TwoBollardEnvironment extends PlanarRegionEnvironmentInterface
{
   public TwoBollardEnvironment(double barrelSeparation)
   {
      // ground plane
      generator.addRectangle(1.0, barrelSeparation + 0.5);
      generator.translate(2.0, 0.0, 0.0);
      generator.addRectangle(3.0, 3.0);
      generator.translate(-4.0, 0.0, 0.0);
      generator.addRectangle(3.0, 3.0);
      addPlanarRegionsToTerrain(YoAppearance.RGBColor(110 / 256.0, 121 / 256.0, 121 / 256.0));

      // first bollard
      generator.identity();
      generator.translate(0.0, 0.5 * barrelSeparation, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.2, 0.2, 0.75);
      addPlanarRegionsToTerrain(YoAppearance.Yellow());

      // second bollard
      generator.identity();
      generator.translate(0.0, -0.5 * barrelSeparation, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.2, 0.2, 0.75);
      addPlanarRegionsToTerrain(YoAppearance.Yellow());
   }

   public static void main(String[] args)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("robot"));
      scs.addStaticLinkGraphics(new TwoBollardEnvironment(0.75).getTerrainObject3D().getLinkGraphics());
      scs.setGroundVisible(false);
      scs.startOnAThread();
   }
}
