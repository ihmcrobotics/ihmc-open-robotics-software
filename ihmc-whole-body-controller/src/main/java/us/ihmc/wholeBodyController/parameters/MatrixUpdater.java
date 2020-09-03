package us.ihmc.wholeBodyController.parameters;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoVariable;

public final class MatrixUpdater implements YoVariableChangedListener
{
   private final int i;
   private final int j;
   private final Matrix3D matrix;

   public MatrixUpdater(int i, int j, Matrix3D matrix)
   {
      this.i = i;
      this.j = j;
      this.matrix = matrix;
   }

   public void changed(YoVariable v)
   {
      matrix.setElement(i, j, v.getValueAsDouble());
   }
}