package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.euclid.Axis3D;
import us.ihmc.commons.MathTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.PlanarRegionEnvironmentInterface;

public class AlternatingSlopesEnvironment extends PlanarRegionEnvironmentInterface
{
   private final double rampWidth;

   public AlternatingSlopesEnvironment(double rampWidth, double landingLength)
   {
      super();
      this.rampWidth = rampWidth;

      generator.addRectangle(landingLength, rampWidth);
      generator.translate(landingLength / 2.0, 0.0, 0.0);
   }

   public void addRamp(double length, double deltaZ)
   {
      checkHasNotBeenGenerated();

      generator.translate(length / 2.0, 0.0, deltaZ / 2.0);
      generator.rotate(-Math.atan2(deltaZ, length), Axis3D.Y);

      generator.addRectangle(Math.sqrt(MathTools.square(length) + MathTools.square(deltaZ)), rampWidth);

      generator.rotate(Math.atan2(deltaZ, length), Axis3D.Y);
      generator.translate(length / 2.0, 0.0, deltaZ / 2.0);
   }

   @Override
   public void generateEnvironment()
   {
      addPlanarRegionsToTerrain(YoAppearance.Grey());
      super.generateEnvironment();
   }
}
