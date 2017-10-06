package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.MathTools;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.PlanarRegionEnvironmentInterface;

public class AlternatingSlopesEnvironment extends PlanarRegionEnvironmentInterface
{
   private final double rampWidth;
   
   public AlternatingSlopesEnvironment(double rampWidth, double landingLength)
   {
      super();
      this.rampWidth = rampWidth;
      
      generator.addRectangle(Math.sqrt(MathTools.square(landingLength)), rampWidth);
      generator.translate(landingLength / 2.0, 0.0, 0.0);
      
   }

   public void addRamp(double length, double deltaZ)
   {
      checkHasNotBeenGenerated();
      
      generator.translate(length / 2.0, 0.0, deltaZ / 2.0);
      generator.rotate(-Math.atan2(deltaZ, length), Axis.Y);
      
      generator.addRectangle(Math.sqrt(MathTools.square(length) + MathTools.square(deltaZ)), rampWidth);
      
      generator.rotate(Math.atan2(deltaZ, length), Axis.Y);
      generator.translate(length / 2.0, 0.0, deltaZ / 2.0);
   }
}
