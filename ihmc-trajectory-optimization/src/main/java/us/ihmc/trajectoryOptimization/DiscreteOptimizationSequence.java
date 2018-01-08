package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;

public class DiscreteOptimizationSequence implements DiscreteOptimizationData
{
   private final DiscreteSequence stateSequence;
   private final DiscreteSequence controlSequence;

   public DiscreteOptimizationSequence(int stateSize, int controlSize)
   {
      this.stateSequence = new DiscreteSequence(stateSize);
      this.controlSequence = new DiscreteSequence(controlSize);
   }

   public void setLength(int length)
   {
      stateSequence.setLength(length);
      controlSequence.setLength(length);
   }

   @Override
   public void set(DiscreteOptimizationData other)
   {
      this.stateSequence.set(other.getStateSequence());
      this.controlSequence.set(other.getControlSequence());
   }

   @Override
   public DiscreteSequence getControlSequence()
   {
      return controlSequence;
   }

   @Override
   public DiscreteSequence getStateSequence()
   {
      return stateSequence;
   }

   @Override
   public DenseMatrix64F getState(int index)
   {
      return stateSequence.get(index);
   }

   @Override
   public DenseMatrix64F getControl(int index)
   {
      return controlSequence.get(index);
   }

   @Override
   public void setState(int index, DenseMatrix64F state)
   {
      stateSequence.get(index).set(state);
   }

   @Override
   public void setControl(int index, DenseMatrix64F control)
   {
      controlSequence.get(index).set(control);
   }

   @Override
   public int size()
   {
      return controlSequence.size();
   }

   @Override
   public void setZero(DiscreteOptimizationData other)
   {
      stateSequence.setZero(other.size());
      controlSequence.setZero(other.size());
   }
}
