package us.ihmc.robotics.math.corruptors;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class NoisyYoRotationMatrix
{
   public final static double DEFAULT_NOISE_ROTATION_ANGLE_STANDARD_DEVIATION = 10.0 / 180.0 * Math.PI;
   public final static double DEFAULT_NOISE_DIRECTION_HEIGHT_BOUND = 1.0;
   public final static double DEFAULT_NOISE_DIRECTION_ANGLE_BOUND = Math.PI;
   public final static double DEFAULT_BIAS_ROTATION_ANGLE= 0.0;
   public final static double DEFAULT_BIAS_ROTATION_ANGLE_MAX = Math.PI;
   public final static double DEFAULT_BIAS_ROTATION_ANGLE_MIN = -Math.PI;
   public final static double DEFAULT_BIAS_ROTATION_ANGLE_DELTA = 0.0;
   public final static double DEFAULT_BIAS_DIRECTION_HEIGHT = 0.0;
   public final static double DEFAULT_BIAS_DIRECTION_HEIGHT_MAX = 1.0;
   public final static double DEFAULT_BIAS_DIRECTION_HEIGHT_MIN = -1.0;
   public final static double DEFAULT_BIAS_DIRECTION_HEIGHT_DELTA = 0.0;
   public final static double DEFAULT_BIAS_DIRECTION_ANGLE= 0.0;
   public final static double DEFAULT_BIAS_DIRECTION_ANGLE_MAX = Math.PI;
   public final static double DEFAULT_BIAS_DIRECTION_ANGLE_MIN = -Math.PI;
   public final static double DEFAULT_BIAS_DIRECTION_ANGLE_DELTA = 0.0;
   
   private final RotationMatrix perfectRotationMatrix = new RotationMatrix();
   private final RotationMatrix noisyRotationMatrix = new RotationMatrix();
   private final RotationMatrix biasMatrix = new RotationMatrix();
   private final RotationMatrix noiseMatrix = new RotationMatrix();
   private final NoisyDoubleYoVariable noiseRotationAngle, noiseDirectionHeight, noiseDirectionAngle;
   private final NoisyDoubleYoVariable biasRotationAngle, biasDirectionHeight, biasDirectionAngle;
   private final DoubleYoVariable noiseDirectionX, noiseDirectionY, noiseDirectionZ;
   private final DoubleYoVariable biasDirectionX, biasDirectionY, biasDirectionZ;

   public NoisyYoRotationMatrix(String name, YoVariableRegistry registry)
   {
      noiseRotationAngle = new NoisyDoubleYoVariable(name + "_noise_rot_ang", registry);
      noiseDirectionHeight = new NoisyDoubleYoVariable(name + "_noise_dir_height", registry);
      noiseDirectionAngle = new NoisyDoubleYoVariable(name + "_noise_dir_ang", registry);
      biasRotationAngle = new NoisyDoubleYoVariable(name + "_bias_rot_ang", registry);
      biasDirectionHeight = new NoisyDoubleYoVariable(name + "_bias_dir_height", registry);
      biasDirectionAngle = new NoisyDoubleYoVariable(name + "_bias_dir_ang", registry);
      noiseDirectionX = new DoubleYoVariable(name + "_noise_dir_x", registry);
      noiseDirectionY = new DoubleYoVariable(name + "_noise_dir_y", registry);
      noiseDirectionZ = new DoubleYoVariable(name + "_noise_dir_z", registry);
      biasDirectionX = new DoubleYoVariable(name + "_bias_dir_x", registry);
      biasDirectionY = new DoubleYoVariable(name + "_bias_dir_y", registry);
      biasDirectionZ = new DoubleYoVariable(name + "_bias_dir_z", registry);
      
      noiseRotationAngle.setGaussianNoise(DEFAULT_NOISE_ROTATION_ANGLE_STANDARD_DEVIATION);
      noiseDirectionHeight.setRandomBound(DEFAULT_NOISE_DIRECTION_HEIGHT_BOUND);
      noiseDirectionAngle.setRandomBound(DEFAULT_NOISE_DIRECTION_ANGLE_BOUND);
      noiseRotationAngle.setBias(false);
      noiseDirectionHeight.setBias(false);
      noiseDirectionAngle.setBias(false);
      
      biasRotationAngle.setRandomBound(0.0);
      biasDirectionHeight.setRandomBound(0.0);
      biasDirectionAngle.setRandomBound(0.0);
      biasRotationAngle.setBias(DEFAULT_BIAS_ROTATION_ANGLE, DEFAULT_BIAS_ROTATION_ANGLE_MAX, DEFAULT_BIAS_ROTATION_ANGLE_MIN, DEFAULT_BIAS_ROTATION_ANGLE_DELTA);
      biasDirectionHeight.setBias(DEFAULT_BIAS_DIRECTION_HEIGHT, DEFAULT_BIAS_DIRECTION_HEIGHT_MAX, DEFAULT_BIAS_DIRECTION_HEIGHT_MIN, DEFAULT_BIAS_DIRECTION_HEIGHT_DELTA);
      biasDirectionAngle.setBias(DEFAULT_BIAS_DIRECTION_ANGLE, DEFAULT_BIAS_DIRECTION_ANGLE_MAX, DEFAULT_BIAS_DIRECTION_ANGLE_MIN, DEFAULT_BIAS_DIRECTION_ANGLE_DELTA);
      biasRotationAngle.setIsNoisy(false);
      biasDirectionHeight.setIsNoisy(false);
      biasDirectionAngle.setIsNoisy(false);
   }


   public void update(RotationMatrix m)
   {
      update(m.getM00(), m.getM01(), m.getM02(), m.getM10(), m.getM11(), m.getM12(), m.getM20(), m.getM21(), m.getM22());
   }
   
   public void update(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      perfectRotationMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      
      generateNoise();
      generateBias();
      
      noisyRotationMatrix.set(perfectRotationMatrix); // NoiseMatrix is transformation from IMU measurement to perfect body orientation
      noisyRotationMatrix.multiply(biasMatrix); // NoiseMatrix is transformation from IMU measurement to perfect body orientation
      noisyRotationMatrix.multiply(noiseMatrix); // NoiseMatrix is transformation from IMU measurement to perfect body orientation
   
      if (((Double)(noisyRotationMatrix.getM00())).isNaN())
      {
         noisyRotationMatrix.set(perfectRotationMatrix);
      }
   }

   private void generateNoise()
   {
      noiseDirectionHeight.update(0.0);
      noiseDirectionAngle.update(0.0);
      noiseRotationAngle.update(0.0);
      
      // Generate uniformly random point on unit sphere (based on http://mathworld.wolfram.com/SpherePointPicking.html)
      double height = noiseDirectionHeight.getDoubleValue();
      double angle = noiseDirectionAngle.getDoubleValue();
      double radius = Math.sqrt(1.0 - height * height);
      
      noiseDirectionX.set(radius * Math.cos(angle));
      noiseDirectionY.set(radius * Math.sin(angle));
      noiseDirectionZ.set(height);
      
      noiseMatrix.set(new AxisAngle(noiseDirectionX.getDoubleValue(), noiseDirectionY.getDoubleValue(), noiseDirectionZ.getDoubleValue(), noiseRotationAngle.getDoubleValue()));
   }
   
   private void generateBias()
   {
      biasDirectionHeight.update(0.0);
      biasDirectionAngle.update(0.0);
      biasRotationAngle.update(0.0);
      
      double height = biasDirectionHeight.getDoubleValue();
      double angle = biasDirectionAngle.getDoubleValue();
      double radius = Math.sqrt(1.0 - height * height);
      
      biasDirectionX.set(radius * Math.cos(angle));
      biasDirectionY.set(radius * Math.sin(angle));
      biasDirectionZ.set(height);
      
      biasMatrix.set(new AxisAngle(biasDirectionX.getDoubleValue(), biasDirectionY.getDoubleValue(), biasDirectionZ.getDoubleValue(), biasRotationAngle.getDoubleValue()));
   }

   public RotationMatrix getMatrix3d()
   {
      return noisyRotationMatrix;
   }

   public void setIsNoisy(boolean choice)
   {
      noiseRotationAngle.setIsNoisy(choice);
      noiseDirectionHeight.setIsNoisy(choice);
      noiseDirectionAngle.setIsNoisy(choice);
      biasRotationAngle.setIsNoisy(choice);
      biasDirectionHeight.setIsNoisy(choice);
      biasDirectionAngle.setIsNoisy(choice);
   }

   public void setBias(boolean choice)
   {
      biasRotationAngle.setIsNoisy(choice);
      biasDirectionHeight.setIsNoisy(choice);
      biasDirectionAngle.setIsNoisy(choice);
   }

   public void setBias(double biasOfRotationAngle)
   {
      biasRotationAngle.setBias(biasOfRotationAngle);
   }
   
   public void setBias(double biasOfRotationAngle, double biasOfDirectionHeight, double biasOfDirectionAngle)
   {
      biasRotationAngle.setBias(biasOfRotationAngle);
      biasDirectionHeight.setBias(biasOfDirectionHeight);
      biasDirectionAngle.setBias(biasOfDirectionAngle);
   }

   public void setBiasOfRotationAngle(double bias, double biasMax, double biasMin, double biasDelta)
   {
      biasRotationAngle.setBias(bias, biasMax, biasMin, biasDelta);
   }
   
   public void setBiasOfDirectionHeight(double bias, double biasMax, double biasMin, double biasDelta)
   {
      biasDirectionHeight.setBias(bias, biasMax, biasMin, biasDelta);
   }
   
   public void setBiasOfDirectionAngle(double bias, double biasMax, double biasMin, double biasDelta)
   {
      biasDirectionAngle.setBias(bias, biasMax, biasMin, biasDelta);
   }
   
   public void setBiasRandomlyBetweenMinAndMax()
   {
      biasRotationAngle.setBiasRandomlyBetweenMinAndMax();
      biasDirectionHeight.setBiasRandomlyBetweenMinAndMax();
      biasDirectionAngle.setBiasRandomlyBetweenMinAndMax();
   }

   public void setNoiseType(NoiseType noiseType)
   {
      noiseRotationAngle.setNoiseType(noiseType);
   }

   public void setRandomBound(double randomBound)
   {
      noiseRotationAngle.setRandomBound(randomBound);
   }

   public void setGaussianNoise(double standardDeviation)
   {
      noiseRotationAngle.setGaussianNoise(standardDeviation);
   }
}