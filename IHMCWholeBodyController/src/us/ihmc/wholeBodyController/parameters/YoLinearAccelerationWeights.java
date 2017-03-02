package us.ihmc.wholeBodyController.parameters;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.robotics.controllers.MatrixUpdater;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class YoLinearAccelerationWeights
{
   private final DoubleYoVariable xAccelerationWeights, yAccelerationWeights, zAccelerationWeight;

   public YoLinearAccelerationWeights(String prefix, YoVariableRegistry registry)
   {
      xAccelerationWeights = new DoubleYoVariable(prefix + "_XAccelerationWeights", registry);
      yAccelerationWeights = new DoubleYoVariable(prefix + "_YAccelerationWeights", registry);
      zAccelerationWeight = new DoubleYoVariable(prefix + "_ZAccelerationWeight", registry);
   }

   public void reset()
   {
      xAccelerationWeights.set(0);
      yAccelerationWeights.set(0);
      zAccelerationWeight.set(0);
   }

   public Matrix3D createLinearAccelerationWeightMatrix()
   {
      Matrix3D weightMatrix = new Matrix3D();

      xAccelerationWeights.addVariableChangedListener(new MatrixUpdater(0, 0, weightMatrix));
      yAccelerationWeights.addVariableChangedListener(new MatrixUpdater(1, 1, weightMatrix));
      zAccelerationWeight.addVariableChangedListener(new MatrixUpdater(2, 2, weightMatrix));

      xAccelerationWeights.notifyVariableChangedListeners();
      yAccelerationWeights.notifyVariableChangedListeners();
      zAccelerationWeight.notifyVariableChangedListeners();

      return weightMatrix;
   }

   public void setLinearAccelerationWeights(double xWeight, double yWeight, double zAccelerationWeight)
   {
      xAccelerationWeights.set(xWeight);
      yAccelerationWeights.set(yWeight);
      this.zAccelerationWeight.set(zAccelerationWeight);
   }

   public void setLinearAccelerationWeights(double xyAccelerationWeights, double zAccelerationWeight)
   {
      xAccelerationWeights.set(xyAccelerationWeights);
      yAccelerationWeights.set(xyAccelerationWeights);
      this.zAccelerationWeight.set(zAccelerationWeight);
   }

   public void setLinearAccelerationWeights(double[] weights)
   {
      xAccelerationWeights.set(weights[0]);
      yAccelerationWeights.set(weights[1]);
      zAccelerationWeight.set(weights[2]);
   }

   public void setLinearAccelerationWeights(double weight)
   {
      xAccelerationWeights.set(weight);
      yAccelerationWeights.set(weight);
      zAccelerationWeight.set(weight);
   }

   public void setXAccelerationWeight(double xAccelerationWeight)
   {
      xAccelerationWeights.set(xAccelerationWeight);
   }

   public void setYAccelerationWeight(double yAccelerationWeight)
   {
      yAccelerationWeights.set(yAccelerationWeight);
   }

   public void setXYAccelerationWeights(double xWeight, double yWeight)
   {
      xAccelerationWeights.set(xWeight);
      yAccelerationWeights.set(yWeight);
   }

   public void setXYAccelerationWeights(double xyAccelerationWeights)
   {
      xAccelerationWeights.set(xyAccelerationWeights);
      yAccelerationWeights.set(xyAccelerationWeights);
   }

   public void setZAccelerationWeight(double zAccelerationWeight)
   {
      this.zAccelerationWeight.set(zAccelerationWeight);
   }

   public double getXAccelerationWeight()
   {
      return xAccelerationWeights.getDoubleValue();
   }

   public double getYAccelerationWeight()
   {
      return yAccelerationWeights.getDoubleValue();
   }

   public double getZAccelerationWeight()
   {
      return zAccelerationWeight.getDoubleValue();
   }

   public void getLinearAccelerationWeights(double[] weightsToPack)
   {
      weightsToPack[0] = xAccelerationWeights.getDoubleValue();
      weightsToPack[1] = yAccelerationWeights.getDoubleValue();
      weightsToPack[2] = zAccelerationWeight.getDoubleValue();
   }
}
