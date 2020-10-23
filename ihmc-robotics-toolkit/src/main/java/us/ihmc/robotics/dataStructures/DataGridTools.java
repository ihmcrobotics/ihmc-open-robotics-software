package us.ihmc.robotics.dataStructures;

import org.ejml.data.DMatrixRMaj;

public class DataGridTools
{
   public static void fillMapWithMatrix(HeightMapWithPoints map, DMatrixRMaj matrix, double gridSize)
   {
      for (int i = 0; i < matrix.getNumRows(); i++)
      {
         for (int j = 0; j < matrix.getNumCols(); j++)
         {
            map.addPoint(i*gridSize, j*gridSize, matrix.get(i, j));
         }
      }
   }
   
   public static void fillMapWithMatrixCentered(HeightMapWithPoints map, DMatrixRMaj matrix, double gridSize)
   {
      for (int i = 0; i < matrix.getNumRows(); i++)
      {
         for (int j = 0; j < matrix.getNumRows(); j++)
         {
            map.addPoint((i-matrix.getNumRows()/2)* gridSize, (j-matrix.getNumRows()/2)* gridSize, matrix.get(i, j));
         }
      }
   }
}
