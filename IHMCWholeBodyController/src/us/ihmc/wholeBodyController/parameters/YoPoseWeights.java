package us.ihmc.wholeBodyController.parameters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

import javax.vecmath.Matrix3d;

public class YoPoseWeights
{
   private final YoPositionWeights yoPositionWeights;
   private final YoOrientationWeights yoOrientationWeights;

   public YoPoseWeights(String prefix, YoVariableRegistry registry)
   {
      yoPositionWeights = new YoPositionWeights(prefix, registry);
      yoOrientationWeights = new YoOrientationWeights(prefix, registry);
   }

   public void reset()
   {
      yoPositionWeights.reset();
      yoOrientationWeights.reset();
   }

   public void resetPosition()
   {
      yoPositionWeights.reset();
   }

   public void resetOrientation()
   {
      yoOrientationWeights.reset();
   }

   public Matrix3d createOrientationWeightMatrix()
   {
      return yoOrientationWeights.createOrientationWeightMatrix();
   }

   public Matrix3d createPositionWeightMatrix()
   {
      return yoPositionWeights.createPositionWeightMatrix();
   }

   public void setOrientationWeights(double yawWeight, double pitchWeight, double rollWeight)
   {
      yoOrientationWeights.setOrientationWeights(yawWeight, pitchWeight, rollWeight);
   }

   public void setOrientationWeights(double weight)
   {
      yoOrientationWeights.setOrientationWeights(weight);
   }

   public void setOrientationWeights(double[] weights)
   {
      yoOrientationWeights.setOrientationWeights(weights);
   }

   public void setYawWeights(double weight)
   {
      yoOrientationWeights.setYawWeights(weight);
   }

   public void setPositionWeights(double xWeight, double yWeight, double zWeight)
   {
      yoPositionWeights.setPositionWeights(xWeight, yWeight, zWeight);
   }

   public void setPositionWeights(double xyWeights, double zWeight)
   {
      yoPositionWeights.setPositionWeights(xyWeights, zWeight);
   }

   public void setPositionWeights(double[] weights)
   {
      yoPositionWeights.setPositionWeights(weights);
   }

   public void setPositionWeights(double weight)
   {
      yoPositionWeights.setPositionWeights(weight);
   }

   public void setPitchWeights(double weight)
   {
      yoOrientationWeights.setPitchWeights(weight);
   }

   public void setRollWeights(double weight)
   {
      yoOrientationWeights.setRollWeights(weight);
   }

   public void setXYWeights(double xWeight, double yWeight)
   {
      yoPositionWeights.setXYWeights(Math.max(xWeight, yWeight));
   }

   public void setXYWeights(double xyWeights)
   {
      yoPositionWeights.setXYWeights(xyWeights);
   }

   public void setZWeight(double zWeight)
   {
      yoPositionWeights.setZWeight(zWeight);
   }

   public double getYawWeight()
   {
      return yoOrientationWeights.getYawWeight();
   }

   public double getPitchWeight()
   {
      return yoOrientationWeights.getPitchWeight();
   }

   public double getRollWeight()
   {
      return yoOrientationWeights.getRollWeight();
   }

   public double[] getOrientationWeights()
   {
      return yoOrientationWeights.getOrientationWeights();
   }

   public double getXWeight()
   {
      return yoPositionWeights.getXWeight();
   }

   public double getYWeight()
   {
      return yoPositionWeights.getYWeight();
   }

   public double getXYWeights()
   {
      return yoPositionWeights.getXYWeights();
   }

   public double getZWeight()
   {
      return yoPositionWeights.getZWeight();
   }

   public double[] getPositionWeights()
   {
      return yoPositionWeights.getPositionWeights();
   }
}
