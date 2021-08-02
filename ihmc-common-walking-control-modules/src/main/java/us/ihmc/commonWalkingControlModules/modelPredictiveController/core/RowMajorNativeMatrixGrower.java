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
      int colsToInsert = matrixToAppend.getNumCols();
      int endCol = colOffset + colsToInsert;
      int leftOverCols = dimension - endCol;

      if (leftOverCols < 0)
         throw new IllegalArgumentException("Invalid matrix size.");

      int previousSize = matrixToAppendTo.getNumRows();
      int taskSize = matrixToAppend.getNumRows();

      tempMatrix.set(matrixToAppendTo);
      matrixToAppendTo.reshape(previousSize + taskSize, dimension);
      matrixToAppendTo.insert(tempMatrix, 0, 0);

      // zero left side
      if (colOffset > 0)
         matrixToAppendTo.fillBlock(previousSize, 0, taskSize, colOffset, 0.0);
      // insert new values
      matrixToAppendTo.insert(matrixToAppend, previousSize, colOffset);
      // zero right side
      if (leftOverCols > 0)
         matrixToAppendTo.fillBlock(previousSize, endCol, taskSize, leftOverCols, 0.0);
   }

   public void appendRows(NativeMatrix matrixToAppendTo, double sign, NativeMatrix matrixToAppend)
   {
      appendRows(matrixToAppendTo, 0, sign, matrixToAppend);
   }

   public void appendRows(NativeMatrix matrixToAppendTo, int colOffset, double sign, NativeMatrix matrixToAppend)
   {
      int dimension = matrixToAppendTo.getNumCols();
      int colsToInsert = matrixToAppend.getNumCols();
      int endCol = colOffset + colsToInsert;
      int leftOverCols = dimension - endCol;

      if (leftOverCols < 0)
         throw new IllegalArgumentException("Invalid matrix size.");

      int previousSize = matrixToAppendTo.getNumRows();
      int taskSize = matrixToAppend.getNumRows();

      tempMatrix.set(matrixToAppendTo);
      matrixToAppendTo.reshape(previousSize + taskSize, dimension);
      matrixToAppendTo.insert(tempMatrix, 0, 0);

      // zero left side
      if (colOffset > 0)
         matrixToAppendTo.fillBlock(previousSize, 0, taskSize, colOffset, 0.0);
      // insert new values
      matrixToAppendTo.insertScaled(matrixToAppend, previousSize, colOffset, sign);
      // zero right side
      if (leftOverCols > 0)
         matrixToAppendTo.fillBlock(previousSize, endCol, taskSize, leftOverCols, 0.0);
   }

}
