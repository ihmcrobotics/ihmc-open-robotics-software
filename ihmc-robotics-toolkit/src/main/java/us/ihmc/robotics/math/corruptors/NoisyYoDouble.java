package us.ihmc.robotics.math.corruptors;

import java.util.Random;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

/*
* A NoisyYoDouble is a noisy version of a YoDouble.  Both the
* perfect and noisy values are retained over time and are accessible.
*
* If using NoisyYoDouble as a modifier for an existing
* YoDouble, the passed in YoDouble will be assigned to the
* .perfect field, where the non-noisy values will be stored.  Otherwise, a
* new YoDouble is created as a field named .perfect. The noisy version
* of the perfect data is stored in the .val field of the
* NoisyYoDouble itself (which extends YoDouble).
*
* The update() method generates noise and modifies .val given the current
* value of .perfect.  The update(double) method generates noise and modifies
* .val given the passed in double value.  Noise is determined by using Random
* values and a scalar of standard deviation.
*
* Boolean flags enable different functionality. IS_NOISY adjusts the update()
* method to assign noisy values to .val or not.  (.randomBound determines the
* the half amplitude of noise that is added.)  USE_BIAS decides whether to bias
* the noise.  (.bias determines the bias amount. .biasMax, .biasMin and
* .biasDelta determine walking bias behavior.)  USE_GAUSSIAN_NOISE determines
* which kind of noise to use.  (.standardDeviation determines the standard
* deviation of the Gaussian noise.)
*
* USE_GAUSSIAN_NOISE overrides .randomBound and uses .standardDeviation as the
* basis for noise calculations.
*
* To access the noisy data (assuming IS_NOISY is true), use the
* .getDoubleValue() method from the superclass. To access the perfect
* data, use the .getPerfectDoubleValue() method from this class. DON'T CHEAT.
*
* The following are default values.  They can be changed with set*() methods.
*   * IS_NOISY - true
*   * randomBound - 1.0
*   * USE_BIAS - false
*   * bias - 0.0
*   * biasMax - 1.0
*   * biasMin - -1.0
*   * biasDelta - 0.001
*   * USE_GAUSSIAN_NOISE - false
*   * standardDeviation - 1.0
 */

public class NoisyYoDouble extends YoDouble
{
   private static final long serialVersionUID = 8152020075993223818L;

   private final YoBoolean isNoisy;
   private final YoBoolean useBias;
   private final YoEnum<NoiseType> noiseType;

   private final long randomSeed = System.nanoTime();
   private final Random rand = new Random(randomSeed);

   private final YoDouble randomBound;
   private final YoDouble bias;
   private final YoDouble biasMin;
   private final YoDouble biasMax;
   private final YoDouble biasDelta;
   private final YoDouble standardDeviation;

   private final YoDouble perfect;

// Simple constructor
   public NoisyYoDouble(String name, YoVariableRegistry registry)
   {
      super(name, registry);
      this.isNoisy = new YoBoolean(name + "_IsNoisy", registry);
      this.isNoisy.set(false);
      this.randomBound = new YoDouble(name + "_RandomBound", registry);
      this.randomBound.set(0.0);
      this.useBias = new YoBoolean(name + "_UseBias", registry);
      this.useBias.set(false);
      this.bias = new YoDouble(name + "_Bias", registry);
      this.bias.set(0.0);
      this.biasMax = new YoDouble(name + "_BiasMax", registry);
      this.biasMax.set(0.0);
      this.biasMin = new YoDouble(name + "_BiasMin", registry);
      this.biasMin.set(0.0);
      this.biasDelta = new YoDouble(name + "_BiasDelta", registry);
      this.biasDelta.set(0.0);
      this.noiseType = new YoEnum<NoiseType>(name + "_NoiseType", registry, NoiseType.class);
      this.noiseType.set(NoiseType.UNIFORM);
      this.standardDeviation = new YoDouble(name + "_StandardDeviation", registry);
      this.standardDeviation.set(0.0);

      this.perfect = new YoDouble(name + "_Perfect", registry);
      this.update(0.0);
   }

// Simple constructor, given existing YoDouble
   public NoisyYoDouble(String name, YoVariableRegistry registry, YoDouble perfect)
   {
      super(name, registry);
      this.isNoisy = new YoBoolean(name + "_IsNoisy", registry);
      this.isNoisy.set(false);
      this.randomBound = new YoDouble(name + "_RandomBound", registry);
      this.randomBound.set(0.0);
      this.useBias = new YoBoolean(name + "_UseBias", registry);
      this.useBias.set(false);
      this.bias = new YoDouble(name + "_Bias", registry);
      this.bias.set(0.0);
      this.biasMax = new YoDouble(name + "_BiasMax", registry);
      this.biasMax.set(0.0);
      this.biasMin = new YoDouble(name + "_BiasMin", registry);
      this.biasMin.set(0.0);
      this.biasDelta = new YoDouble(name + "_BiasDelta", registry);
      this.biasDelta.set(0.0);
      this.noiseType = new YoEnum<NoiseType>(name + "_NoiseType", registry, NoiseType.class);
      this.noiseType.set(NoiseType.UNIFORM);
      this.standardDeviation = new YoDouble(name + "_StandardDeviation", registry);
      this.standardDeviation.set(0.0);

      this.perfect = perfect;
      this.update();
   }

// Full constructor
   public NoisyYoDouble(String name, YoVariableRegistry registry, boolean isNoisy, double randomBound, boolean useBias, double bias, double biasMax,
                                double biasMin, double biasDelta, NoiseType noiseType, double standardDeviation)
   {
      super(name, registry);
      this.isNoisy = new YoBoolean(name + "_IsNoisy", registry);
      this.isNoisy.set(isNoisy);
      this.randomBound = new YoDouble(name + "_RandomBound", registry);
      this.randomBound.set(randomBound);
      this.useBias = new YoBoolean(name + "_UseBias", registry);
      this.useBias.set(useBias);
      this.bias = new YoDouble(name + "_Bias", registry);
      this.bias.set(bias);
      this.biasMax = new YoDouble(name + "_BiasMax", registry);
      this.biasMax.set(biasMax);
      this.biasMin = new YoDouble(name + "_BiasMin", registry);
      this.biasMin.set(biasMin);
      this.biasDelta = new YoDouble(name + "_BiasDelta", registry);
      this.biasDelta.set(biasDelta);
      this.noiseType = new YoEnum<NoiseType>(name + "_NoiseType", registry, NoiseType.class);
      this.noiseType.set(noiseType);
      this.standardDeviation = new YoDouble(name + "_StandardDeviation", registry);
      this.standardDeviation.set(standardDeviation);

      this.perfect = new YoDouble(name + "_Perfect", registry);
      this.update(0.0);
   }

// Full constructor, given existing YoDouble
   public NoisyYoDouble(String name, YoVariableRegistry registry, YoDouble perfect, boolean isNoisy, double randomBound, boolean useBias,
                                double bias, double biasMax, double biasMin, double biasDelta, NoiseType noiseType, double standardDeviation)
   {
      super(name, registry);
      this.isNoisy = new YoBoolean(name + "_IsNoisy", registry);
      this.isNoisy.set(isNoisy);
      this.randomBound = new YoDouble(name + "_RandomBound", registry);
      this.randomBound.set(randomBound);
      this.useBias = new YoBoolean(name + "_UseBias", registry);
      this.useBias.set(useBias);
      this.bias = new YoDouble(name + "_Bias", registry);
      this.bias.set(bias);
      this.biasMax = new YoDouble(name + "_BiasMax", registry);
      this.biasMax.set(biasMax);
      this.biasMin = new YoDouble(name + "_BiasMin", registry);
      this.biasMin.set(biasMin);
      this.biasDelta = new YoDouble(name + "_BiasDelta", registry);
      this.biasDelta.set(biasDelta);
      this.noiseType = new YoEnum<NoiseType>(name + "_NoiseType", registry, NoiseType.class);
      this.noiseType.set(noiseType);
      this.standardDeviation = new YoDouble(name + "_StandardDeviation", registry);
      this.standardDeviation.set(standardDeviation);

      this.perfect = perfect;
      this.update();
   }



   public void update()
   {
      update(perfect.getDoubleValue());
   }

   public void update(double perfectValue)
   {
      perfect.set(perfectValue);

      if (isNoisy.getBooleanValue())
      {
         double noise = getBias() + getRandomNoise();
         super.set(perfect.getDoubleValue() + noise);
      }
      else
         super.set(perfect.getDoubleValue());

//    System.out.println("NoisyYoDouble Diff: (" + this.getName() + ")" + (super.getDoubleValue() - perfect.getDoubleValue()));
   }

// public void set(double value)
// {
//    throw new NullPointerException("Don't use set(double).  Use update(double).");
// }




   public double getPerfectDoubleValue()
   {
      return perfect.getDoubleValue();
   }

   public void setIsNoisy(boolean choice)
   {
      isNoisy.set(choice);
   }

   public void setBias(boolean choice)
   {
      useBias.set(choice);
   }

   public void setBias(double bias)
   {
      useBias.set(true);
      this.bias.set(bias);
      if (bias > biasMax.getDoubleValue())
         biasMax.set(bias);
      if (bias < biasMin.getDoubleValue())
         biasMin.set(bias);
      biasDelta.set(0.0);
   }

   public void setBias(double bias, double biasMax, double biasMin, double biasDelta)
   {
      useBias.set(true);
      this.bias.set(bias);
      this.biasMax.set(biasMax);
      this.biasMin.set(biasMin);
      this.biasDelta.set(biasDelta);
   }
   
   public void setBiasRandomlyBetweenMinAndMax()
   {
      bias.set((biasMax.getDoubleValue() - biasMin.getDoubleValue()) * rand.nextDouble() + biasMin.getDoubleValue());
   }

   public void setNoiseType(NoiseType noiseType)
   {
      this.noiseType.set(noiseType);
   }

   public void setRandomBound(double randomBound)
   {
      this.noiseType.set(NoiseType.UNIFORM);
      this.randomBound.set(randomBound);
   }

   public void setGaussianNoise(double standardDeviation)
   {
      this.noiseType.set(NoiseType.GAUSSIAN);
      this.standardDeviation.set(standardDeviation);
   }

   private double getBias()
   {
      if (useBias.getBooleanValue())
      {
         double biasWalk = biasDelta.getDoubleValue() * ((2.0 * rand.nextDouble()) - 1.0);
         bias.set(MathTools.clamp(bias.getDoubleValue() + biasWalk, biasMin.getDoubleValue(), biasMax.getDoubleValue()));

         return bias.getDoubleValue();
      }
      else
         return 0.0;
   }

   private double getRandomNoise()
   {
      switch (noiseType.getEnumValue())
      {
         case UNIFORM :
            return randomBound.getDoubleValue() * (2.0 * rand.nextDouble() - 1.0);

         case GAUSSIAN :
            return standardDeviation.getDoubleValue() * rand.nextGaussian();

         default :
            throw new RuntimeException("Noise type not recognized");
      }
   }
}
