package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import us.ihmc.matrixlib.NativeMatrix;

public class RowMajorNativeMatrixGrower
{
   private final NativeMatrix tempMatrix = new NativeMatrix(0, 0);

   public void appendRows(NativeMatrix matrixToAppendTo, NativeMatrix matrixToAppend)
   {
      appendRows(matrixToAppendTo, 0, matrixToAppend);
   }

   public void appendRows(NativeMatrix matrixToAppendTo, int colOffset, NativeMatrix matrixToAppend)
   {
      int dimension = matrixToAppendTo.getNumCols();

      if (dimension < matrixToAppend.getNumCols() + colOffset)
         throw new IllegalArgumentException("Invalid matrix size.");

      int previousSize = matrixToAppendTo.getNumRows();
      int taskSize = matrixToAppend.getNumRows();


      tempMatrix.set(matrixToAppendTo);
      matrixToAppendTo.reshape(previousSize + taskSize, dimension);
      matrixToAppendTo.insert(tempMatrix, 0, 0);
      matrixToAppendTo.fillBlock(previousSize, 0, taskSize, dimension, 0.0);
      matrixToAppendTo.insert(matrixToAppend, previousSize, colOffset);
   }

}
