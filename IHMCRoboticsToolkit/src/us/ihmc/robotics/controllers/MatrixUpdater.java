package us.ihmc.robotics.controllers;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public final class MatrixUpdater implements VariableChangedListener
{
   private final int i;
   private final int j;
   private final Matrix3d matrix;

   public MatrixUpdater(int i, int j, Matrix3d matrix)
   {
      this.i = i;
      this.j = j;
      this.matrix = matrix;
   }

   public void variableChanged(YoVariable<?> v)
   {
      matrix.setElement(i, j, v.getValueAsDouble());
   }
}