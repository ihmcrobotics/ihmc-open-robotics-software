package us.ihmc.sensorProcessing.signalCorruption;

import java.util.Random;

import org.apache.commons.math3.util.FastMath;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class GaussianOrientationCorruptor implements SignalCorruptor<RotationMatrix>
{
   private final YoVariableRegistry registry;
   private final AxisAngle noiseAxisAngle = new AxisAngle();
   private final RotationMatrix noiseRotationMatrix = new RotationMatrix();
   private final Random random;
   private final DoubleYoVariable standardDeviation;

   public GaussianOrientationCorruptor(String namePrefix, long seed, YoVariableRegistry parentRegistry)
   {
      random = new Random(seed);
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      standardDeviation = new DoubleYoVariable(namePrefix + "StdDev", registry);
      parentRegistry.addChild(registry);
   }

   public void setStandardDeviation(double standardDeviation)
   {
      this.standardDeviation.set(standardDeviation);
   }

   public void corrupt(RotationMatrix signal)
   {
      generateGaussianRotation(noiseAxisAngle, random, standardDeviation.getDoubleValue());
      noiseRotationMatrix.set(noiseAxisAngle);
      signal.multiply(noiseRotationMatrix);
   }

   private static void generateGaussianRotation(AxisAngle axisAngleToPack, Random random, double standardDeviation)
   {
      /*
       * random direction obtained from
       * from http://mathworld.wolfram.com/SpherePointPicking.html, equation (16)
       *
       * not completely sure about the right way to do Gaussian distribution on SO(3), but this should be OK for now.
       */

      double x = random.nextGaussian();
      double y = random.nextGaussian();
      double z = random.nextGaussian();

      double inverseSquareRoot = 1.0 / FastMath.sqrt(MathTools.square(x) + MathTools.square(y) + MathTools.square(z));
      double angle = random.nextGaussian() * standardDeviation;

      axisAngleToPack.set(x * inverseSquareRoot, y * inverseSquareRoot, z * inverseSquareRoot, angle);
   }
}
