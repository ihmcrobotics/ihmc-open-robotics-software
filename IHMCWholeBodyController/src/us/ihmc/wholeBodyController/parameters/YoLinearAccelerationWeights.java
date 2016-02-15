package us.ihmc.wholeBodyController.parameters;

import us.ihmc.robotics.controllers.MatrixUpdater;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import javax.vecmath.Matrix3d;

public class YoLinearAccelerationWeights
{
   private final DoubleYoVariable xyAccelerationWeights, zAccelerationWeight;

   public YoLinearAccelerationWeights(String prefix, YoVariableRegistry registry)
   {
      xyAccelerationWeights = new DoubleYoVariable(prefix + "_XYAccelerationWeights", registry);
      zAccelerationWeight = new DoubleYoVariable(prefix + "_ZAccelerationWeight", registry);
   }

   public void reset()
   {
      xyAccelerationWeights.set(0);
      zAccelerationWeight.set(0);
   }

   public Matrix3d createLinearAccelerationWeightMatrix()
   {
      Matrix3d weightMatrix = new Matrix3d();

      xyAccelerationWeights.addVariableChangedListener(new MatrixUpdater(0, 0, weightMatrix));
      xyAccelerationWeights.addVariableChangedListener(new MatrixUpdater(1, 1, weightMatrix));
      zAccelerationWeight.addVariableChangedListener(new MatrixUpdater(2, 2, weightMatrix));

      xyAccelerationWeights.notifyVariableChangedListeners();
      zAccelerationWeight.notifyVariableChangedListeners();

      return weightMatrix;
   }

   public void setLinearAccelerationWeights(double xWeight, double yWeight, double zAccelerationWeight)
   {
      xyAccelerationWeights.set(Math.max(xWeight, yWeight));
      this.zAccelerationWeight.set(zAccelerationWeight);
   }

   public void setLinearAccelerationWeights(double xyAccelerationWeights, double zAccelerationWeight)
   {
      this.xyAccelerationWeights.set(xyAccelerationWeights);
      this.zAccelerationWeight.set(zAccelerationWeight);
   }

   public void setLinearAccelerationWeights(double[] weights)
   {
      xyAccelerationWeights.set(Math.max(weights[0], weights[1]));
      zAccelerationWeight.set(weights[2]);
   }

   public void setLinearAccelerationWeights(double weight)
   {
      xyAccelerationWeights.set(weight);
      zAccelerationWeight.set(weight);
   }

   public void setXYAccelerationWeights(double xWeight, double yWeight)
   {
      xyAccelerationWeights.set(Math.max(xWeight, yWeight));
   }

   public void setXYAccelerationWeights(double xyAccelerationWeights)
   {
      this.xyAccelerationWeights.set(xyAccelerationWeights);
   }

   public void setZAccelerationWeight(double zAccelerationWeight)
   {
      this.zAccelerationWeight.set(zAccelerationWeight);
   }

   public double getXAccelerationWeight()
   {
      return xyAccelerationWeights.getDoubleValue();
   }

   public double getYAccelerationWeight()
   {
      return xyAccelerationWeights.getDoubleValue();
   }

   public double getXYAccelerationWeights()
   {
      return xyAccelerationWeights.getDoubleValue();
   }

   public double getZAccelerationWeight()
   {
      return zAccelerationWeight.getDoubleValue();
   }

   public double[] getLinearAccelerationWeights()
   {
      double[] linearAccelerationWeights = new double[3];
      linearAccelerationWeights[0] = xyAccelerationWeights.getDoubleValue();
      linearAccelerationWeights[1] = xyAccelerationWeights.getDoubleValue();
      linearAccelerationWeights[2] = zAccelerationWeight.getDoubleValue();

      return linearAccelerationWeights;
   }
}
