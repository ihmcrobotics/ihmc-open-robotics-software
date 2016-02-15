package us.ihmc.wholeBodyController.parameters;

import us.ihmc.robotics.controllers.MatrixUpdater;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import javax.vecmath.Matrix3d;

public class YoPositionWeights
{
   private final DoubleYoVariable xyWeights, zWeight;

   public YoPositionWeights(String prefix, YoVariableRegistry registry)
   {
      xyWeights = new DoubleYoVariable(prefix + "_XYWeights", registry);
      zWeight = new DoubleYoVariable(prefix + "_ZWeight", registry);
   }

   public void reset()
   {
      xyWeights.set(0);
      zWeight.set(0);
   }

   public Matrix3d createPositionWeightMatrix()
   {
      Matrix3d weightMatrix = new Matrix3d();

      xyWeights.addVariableChangedListener(new MatrixUpdater(0, 0, weightMatrix));
      xyWeights.addVariableChangedListener(new MatrixUpdater(1, 1, weightMatrix));
      zWeight.addVariableChangedListener(new MatrixUpdater(2, 2, weightMatrix));

      xyWeights.notifyVariableChangedListeners();
      zWeight.notifyVariableChangedListeners();

      return weightMatrix;
   }

   public void setPositionWeights(double xWeight, double yWeight, double zWeight)
   {
      xyWeights.set(Math.max(xWeight, yWeight));
      this.zWeight.set(zWeight);
   }

   public void setPositionWeights(double xyWeights, double zWeight)
   {
      this.xyWeights.set(xyWeights);
      this.zWeight.set(zWeight);
   }

   public void setPositionWeights(double[] weights)
   {
      xyWeights.set(Math.max(weights[0], weights[1]));
      zWeight.set(weights[2]);
   }

   public void setPositionWeights(double weight)
   {
      xyWeights.set(weight);
      zWeight.set(weight);
   }

   public void setXYWeights(double xWeight, double yWeight)
   {
      xyWeights.set(Math.max(xWeight, yWeight));
   }

   public void setXYWeights(double xyWeights)
   {
      this.xyWeights.set(xyWeights);
   }

   public void setZWeight(double zWeight)
   {
      this.zWeight.set(zWeight);
   }

   public double getXWeight()
   {
      return xyWeights.getDoubleValue();
   }

   public double getYWeight()
   {
      return xyWeights.getDoubleValue();
   }

   public double getXYWeights()
   {
      return xyWeights.getDoubleValue();
   }

   public double getZWeight()
   {
      return zWeight.getDoubleValue();
   }

   public double[] getPositionWeights()
   {
      double[] positionWeights = new double[3];
      positionWeights[0] = xyWeights.getDoubleValue();
      positionWeights[1] = xyWeights.getDoubleValue();
      positionWeights[2] = zWeight.getDoubleValue();

      return positionWeights;
   }
}
