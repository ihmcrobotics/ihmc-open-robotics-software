package us.ihmc.wholeBodyController.parameters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

import javax.vecmath.Matrix3d;

public class YoAccelerationWeights
{
   private final YoLinearAccelerationWeights yoLinearAccelerationWeights;
   private final YoAngularAccelerationWeights yoAngularAccelerationWeights;

   public YoAccelerationWeights(String prefix, YoVariableRegistry registry)
   {
      yoLinearAccelerationWeights = new YoLinearAccelerationWeights(prefix, registry);
      yoAngularAccelerationWeights = new YoAngularAccelerationWeights(prefix, registry);
   }

   public void reset()
   {
      yoLinearAccelerationWeights.reset();
      yoAngularAccelerationWeights.reset();
   }

   public void resetLinear()
   {
      yoLinearAccelerationWeights.reset();
   }

   public void resetAngular()
   {
      yoAngularAccelerationWeights.reset();
   }

   public Matrix3d createAngularAccelerationWeightMatrix()
   {
      return yoAngularAccelerationWeights.createAngularAccelerationWeightMatrix();
   }

   public Matrix3d createLinearAccelerationWeightMatrix()
   {
      return yoLinearAccelerationWeights.createLinearAccelerationWeightMatrix();
   }

   public void setAngularAccelerationWeights(double yawWeight, double pitchWeight, double rollWeight)
   {
      yoAngularAccelerationWeights.setAngularAccelerationWeights(yawWeight, pitchWeight, rollWeight);
   }

   public void setAngularAccelerationWeights(double weight)
   {
      yoAngularAccelerationWeights.setAngularAccelerationWeights(weight);
   }

   public void setAngularAccelerationWeights(double[] weights)
   {
      yoAngularAccelerationWeights.setAngularAccelerationWeights(weights);
   }

   public void setYawAccelerationWeight(double weight)
   {
      yoAngularAccelerationWeights.setYawAccelerationWeight(weight);
   }

   public void setPitchAccelerationWeight(double weight)
   {
      yoAngularAccelerationWeights.setPitchAccelerationWeight(weight);
   }

   public void setRollAccelerationWeight(double weight)
   {
      yoAngularAccelerationWeights.setRollAccelerationWeight(weight);
   }

   public void setLinearAccelerationWeights(double xWeight, double yWeight, double zWeight)
   {
      yoLinearAccelerationWeights.setLinearAccelerationWeights(xWeight, yWeight, zWeight);
   }

   public void setLinearAccelerationWeights(double xyWeights, double zWeight)
   {
      yoLinearAccelerationWeights.setLinearAccelerationWeights(xyWeights, zWeight);
   }

   public void setLinearAccelerationWeights(double[] weights)
   {
      yoLinearAccelerationWeights.setLinearAccelerationWeights(weights);
   }

   public void setLinearAccelerationWeights(double weight)
   {
      yoLinearAccelerationWeights.setLinearAccelerationWeights(weight);
   }

   public void setXAccelerationWeight(double xWeight)
   {
      yoLinearAccelerationWeights.setXAccelerationWeight(xWeight);
   }

   public void setYAccelerationWeight(double yWeight)
   {
      yoLinearAccelerationWeights.setYAccelerationWeight(yWeight);
   }

   public void setXYAccelerationWeights(double xWeight, double yWeight)
   {
      yoLinearAccelerationWeights.setXYAccelerationWeights(xWeight, yWeight);
   }

   public void setXYAccelerationWeights(double xyWeights)
   {
      yoLinearAccelerationWeights.setXYAccelerationWeights(xyWeights);
   }

   public void setZAccelerationWeight(double zWeight)
   {
      yoLinearAccelerationWeights.setZAccelerationWeight(zWeight);
   }

   public double getYawAccelerationWeight()
   {
      return yoAngularAccelerationWeights.getYawAccelerationWeight();
   }

   public double getPitchAccelerationWeight()
   {
      return yoAngularAccelerationWeights.getPitchAccelerationWeight();
   }

   public double getRollAccelerationWeight()
   {
      return yoAngularAccelerationWeights.getRollAccelerationWeight();
   }

   public double[] getAngularAccelerationWeights()
   {
      return yoAngularAccelerationWeights.getAngularAccelerationWeights();
   }

   public double getXAccelerationWeight()
   {
      return yoLinearAccelerationWeights.getXAccelerationWeight();
   }

   public double getYAccelerationWeight()
   {
      return yoLinearAccelerationWeights.getYAccelerationWeight();
   }

   public double getZAccelerationWeight()
   {
      return yoLinearAccelerationWeights.getZAccelerationWeight();
   }

   public double[] getLinearAccelerationWeights()
   {
      return yoLinearAccelerationWeights.getLinearAccelerationWeights();
   }
}
