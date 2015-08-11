package us.ihmc.robotics.dataStructures;

import javax.vecmath.GMatrix;

public class DataGridTools
{
   public static void fillMapWithMatrix(HeightMapWithPoints map, GMatrix matrix, double gridSize)
   {
      for (int i = 0; i < matrix.getNumRow(); i++)
      {
         for (int j = 0; j < matrix.getNumRow(); j++)
         {
            map.addPoint(i*gridSize, j*gridSize, matrix.getElement(i, j));
         }
      }
   }
   
   public static void fillMapWithMatrixCentered(HeightMapWithPoints map, GMatrix matrix, double gridSize)
   {
      for (int i = 0; i < matrix.getNumRow(); i++)
      {
         for (int j = 0; j < matrix.getNumRow(); j++)
         {
            map.addPoint((i-matrix.getNumRow()/2)* gridSize, (j-matrix.getNumRow()/2)* gridSize, matrix.getElement(i, j));
         }
      }
   }
}
