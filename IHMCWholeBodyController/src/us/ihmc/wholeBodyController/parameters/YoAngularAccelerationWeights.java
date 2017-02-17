package us.ihmc.wholeBodyController.parameters;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.robotics.controllers.MatrixUpdater;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class YoAngularAccelerationWeights
{
   private final DoubleYoVariable yawAccelerationWeight, pitchAccelerationWeight, rollAccelerationWeight;

   public YoAngularAccelerationWeights(String prefix, YoVariableRegistry registry)
   {
      yawAccelerationWeight = new DoubleYoVariable(prefix + "_YawAccelerationWeight", registry);
      pitchAccelerationWeight = new DoubleYoVariable(prefix + "_PitchAccelerationWeight", registry);
      rollAccelerationWeight = new DoubleYoVariable(prefix + "_RollAccelerationWeight", registry);
   }

   public void reset()
   {
      yawAccelerationWeight.set(0);
      pitchAccelerationWeight.set(0);
      rollAccelerationWeight.set(0);
   }

   public Matrix3D createAngularAccelerationWeightMatrix()
   {
      Matrix3D weightMatrix = new Matrix3D();

      yawAccelerationWeight.addVariableChangedListener(new MatrixUpdater(0, 0, weightMatrix));
      pitchAccelerationWeight.addVariableChangedListener(new MatrixUpdater(1, 1, weightMatrix));
      rollAccelerationWeight.addVariableChangedListener(new MatrixUpdater(2, 2, weightMatrix));

      yawAccelerationWeight.notifyVariableChangedListeners();
      pitchAccelerationWeight.notifyVariableChangedListeners();
      rollAccelerationWeight.notifyVariableChangedListeners();

      return weightMatrix;
   }

   public void setAngularAccelerationWeights(double yawAccelerationWeight, double pitchAccelerationWeight, double rollAccelerationWeight)
   {
      this.yawAccelerationWeight.set(yawAccelerationWeight);
      this.pitchAccelerationWeight.set(pitchAccelerationWeight);
      this.rollAccelerationWeight.set(rollAccelerationWeight);
   }

   public void setAngularAccelerationWeights(double weight)
   {
      this.yawAccelerationWeight.set(weight);
      this.pitchAccelerationWeight.set(weight);
      this.rollAccelerationWeight.set(weight);
   }

   public void setAngularAccelerationWeights(double[] weights)
   {
      yawAccelerationWeight.set(weights[0]);
      pitchAccelerationWeight.set(weights[1]);
      rollAccelerationWeight.set(weights[2]);
   }

   public void setYawAccelerationWeight(double weight)
   {
      yawAccelerationWeight.set(weight);
   }

   public void setPitchAccelerationWeight(double weight)
   {
      pitchAccelerationWeight.set(weight);
   }

   public void setRollAccelerationWeight(double weight)
   {
      rollAccelerationWeight.set(weight);
   }

   public double getYawAccelerationWeight()
   {
      return yawAccelerationWeight.getDoubleValue();
   }

   public double getPitchAccelerationWeight()
   {
      return pitchAccelerationWeight.getDoubleValue();
   }

   public double getRollAccelerationWeight()

   {
      return rollAccelerationWeight.getDoubleValue();
   }

   public void getAngularAccelerationWeights(double[] weightsToPack)
   {
      weightsToPack[0] = yawAccelerationWeight.getDoubleValue();
      weightsToPack[1] = pitchAccelerationWeight.getDoubleValue();
      weightsToPack[2] = rollAccelerationWeight.getDoubleValue();
   }
}
