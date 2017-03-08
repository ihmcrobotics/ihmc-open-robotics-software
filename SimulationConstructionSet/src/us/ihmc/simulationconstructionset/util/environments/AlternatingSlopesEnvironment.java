package us.ihmc.simulationconstructionset.util.environments;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationconstructionset.util.environments.planarRegionEnvironments.PlanarRegionEnvironmentInterface;

public class AlternatingSlopesEnvironment extends PlanarRegionEnvironmentInterface
{
   private final double rampWidth;
   
   public AlternatingSlopesEnvironment(double rampWidth)
   {
      this.rampWidth = rampWidth;
   }
   
   @Override
   public void generateEnvironment()
   {
      super.generateEnvironment();
   }

   @Override
   protected void buildGenerator(PlanarRegionsListGenerator generator)
   {
   }

   public void addRamp(double length, double deltaZ)
   {
      generator.translate(length / 2.0, 0.0, deltaZ / 2.0);
      generator.rotate(-Math.atan2(deltaZ, length), Axis.Y);
      
      generator.addRectangle(Math.sqrt(MathTools.square(length) + MathTools.square(deltaZ)), rampWidth);
      
      generator.rotate(Math.atan2(deltaZ, length), Axis.Y);
      generator.translate(length / 2.0, 0.0, deltaZ / 2.0);
   }
}
