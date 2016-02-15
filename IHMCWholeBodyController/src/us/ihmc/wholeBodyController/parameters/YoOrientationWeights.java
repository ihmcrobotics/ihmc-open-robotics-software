package us.ihmc.wholeBodyController.parameters;

import us.ihmc.robotics.controllers.MatrixUpdater;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import javax.vecmath.Matrix3d;

public class YoOrientationWeights
{
   private final DoubleYoVariable yawWeight, pitchWeight, rollWeight;

   public YoOrientationWeights(String prefix, YoVariableRegistry registry)
   {
      yawWeight = new DoubleYoVariable(prefix + "_YawWeight", registry);
      pitchWeight = new DoubleYoVariable(prefix + "_PitchWeight", registry);
      rollWeight = new DoubleYoVariable(prefix + "_PitchWeight", registry);
   }

   public void reset()
   {
      yawWeight.set(0);
      pitchWeight.set(0);
      rollWeight.set(0);
   }

   public Matrix3d createOrientationWeightMatrix()
   {
      Matrix3d weightMatrix = new Matrix3d();

      yawWeight.addVariableChangedListener(new MatrixUpdater(0, 0, weightMatrix));
      pitchWeight.addVariableChangedListener(new MatrixUpdater(1, 1, weightMatrix));
      rollWeight.addVariableChangedListener(new MatrixUpdater(2, 2, weightMatrix));

      yawWeight.notifyVariableChangedListeners();
      pitchWeight.notifyVariableChangedListeners();
      rollWeight.notifyVariableChangedListeners();

      return weightMatrix;
   }

   public void setOrientationWeights(double yawWeight, double pitchWeight, double rollWeight)
   {
      this.yawWeight.set(yawWeight);
      this.pitchWeight.set(pitchWeight);
      this.rollWeight.set(rollWeight);
   }

   public void setOrientationWeights(double weight)
   {
      this.yawWeight.set(weight);
      this.pitchWeight.set(weight);
      this.rollWeight.set(weight);
   }

   public void setOrientationWeights(double[] weights)
   {
      yawWeight.set(weights[0]);
      pitchWeight.set(weights[1]);
      rollWeight.set(weights[2]);
   }

   public void setYawWeights(double weight)
   {
      yawWeight.set(weight);
   }

   public void setPitchWeights(double weight)
   {
      pitchWeight.set(weight);
   }

   public void setRollWeights(double weight)
   {
      rollWeight.set(weight);
   }

   public double getYawWeight()
   {
      return yawWeight.getDoubleValue();
   }

   public double getPitchWeight()
   {
      return pitchWeight.getDoubleValue();
   }

   public double getRollWeight()
   {
      return rollWeight.getDoubleValue();
   }

   public double[] getOrientationWeights()
   {
      double[] orientationWeights = new double[3];
      orientationWeights[0] = yawWeight.getDoubleValue();
      orientationWeights[1] = pitchWeight.getDoubleValue();
      orientationWeights[2] = rollWeight.getDoubleValue();

      return orientationWeights;
   }
}
