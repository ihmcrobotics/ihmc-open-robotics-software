package us.ihmc.robotics.math;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixFeatures;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class MatrixYoVariableConversionTools
{
   public static void populateYoVariables(DoubleYoVariable[][] yoVariableArray, String prefix, YoVariableRegistry registry)
   {
      for (int i = 0; i < yoVariableArray.length; i++)
      {
         for (int j = 0; j < yoVariableArray[0].length; j++)
         {
            yoVariableArray[i][j] = new DoubleYoVariable(prefix + Integer.toString(i) + "_" + Integer.toString(j), registry);
         }
      }
   }

   public static void populateYoVariablesSymmetric(DoubleYoVariable[] yoVariableArray, String prefix, int size, YoVariableRegistry registry)
   {
      int n = 0;
      for (int i = 0; i < size; i++)
      {
         for (int j = i; j < size; j++)
         {
            yoVariableArray[n] = new DoubleYoVariable(prefix + Integer.toString(i) + "_" + Integer.toString(j), registry);
            n++;
         }
      }
   }

   public static void populateYoVariables(DoubleYoVariable[] yoVariableArray, String prefix, YoVariableRegistry registry)
   {
      for (int i = 0; i < yoVariableArray.length; i++)
      {
         yoVariableArray[i] = new DoubleYoVariable(prefix + Integer.toString(i), registry);
      }
   }

   public static void storeInYoVariables(DenseMatrix64F m, DoubleYoVariable[][] yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         for (int j = 0; j < m.getNumCols(); j++)
         {
            yoM[i][j].set(m.get(i, j));
         }
      }
   }

   public static void storeInYoVariablesSymmetric(DenseMatrix64F m, DoubleYoVariable[] yoM)
   {
      int size = m.getNumRows();

      int n = 0;
      for (int i = 0; i < size; i++)
      {
         for (int j = i; j < size; j++)
         {
            yoM[n].set(m.get(i, j));
            n++;
         }
      }
   }

   public static void storeInYoVariables(DenseMatrix64F v, DoubleYoVariable[] yoV)
   {
      for (int i = 0; i < v.getNumRows(); i++)
      {
         yoV[i].set(v.get(i));
      }
   }

   public static void getFromYoVariables(DenseMatrix64F m, DoubleYoVariable[][] yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         for (int j = 0; j < m.getNumCols(); j++)
         {
            m.set(i, j, yoM[i][j].getDoubleValue());
         }
      }
   }

   public static void getFromYoVariablesSymmetric(DenseMatrix64F m, DoubleYoVariable[] yoM)
   {
      int size = m.getNumRows();

      int n = 0;
      for (int i = 0; i < size; i++)
      {
         for (int j = i; j < size; j++)
         {
            m.set(i, j, yoM[n].getDoubleValue());
            m.set(j, i, yoM[n].getDoubleValue());
            n++;
         }
      }
   }

   public static void getFromYoVariables(DenseMatrix64F m, DoubleYoVariable[] yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         m.set(i, 0, yoM[i].getDoubleValue());
      }
   }

   public static void populateYoVariablesMatrix(List<List<DoubleYoVariable>> yoVariableArray, int nRows, int nColumns, String prefix,
         YoVariableRegistry registry)
   {
      for (int i = 0; i < nRows; i++)
      {
         List<DoubleYoVariable> row = getOrAddRow(yoVariableArray, i);
         for (int j = row.size(); j < nColumns; j++)
         {
            row.add(new DoubleYoVariable(prefix + Integer.toString(i) + "_" + Integer.toString(j), registry));
         }
      }
   }

   public static void populateYoVariablesSymmetricMatrix(List<List<DoubleYoVariable>> yoVariableArray, int size, String prefix, YoVariableRegistry registry)
   {
      for (int i = 0; i < size; i++)
      {
         List<DoubleYoVariable> row = getOrAddRow(yoVariableArray, i);

         for (int j = row.size(); j <= i; j++)
         {
            row.add(new DoubleYoVariable(prefix + Integer.toString(i) + "_" + Integer.toString(j), registry));
         }
      }
   }

   public static void populateYoVariablesVector(List<DoubleYoVariable> yoVariableArray, int size, String prefix, YoVariableRegistry registry)
   {
      for (int i = yoVariableArray.size(); i < size; i++)
      {
         yoVariableArray.add(new DoubleYoVariable(prefix + Integer.toString(i), registry));
      }
   }

   public static void storeInYoVariablesMatrix(DenseMatrix64F m, List<List<DoubleYoVariable>> yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         List<DoubleYoVariable> row = yoM.get(i);
         for (int j = 0; j < m.getNumCols(); j++)
         {
            row.get(j).set(m.get(i, j));
         }
      }
   }

   public static void storeInYoVariablesSymmetric(DenseMatrix64F m, List<List<DoubleYoVariable>> yoM)
   {
      int size = m.getNumRows();

      for (int i = 0; i < size; i++)
      {
         List<DoubleYoVariable> row = yoM.get(i);
         for (int j = 0; j <= i; j++)
         {
            row.get(j).set(m.get(i, j));
         }
      }
   }

   public static void storeInYoVariablesVector(DenseMatrix64F v, List<DoubleYoVariable> yoV)
   {
      for (int i = 0; i < v.getNumRows(); i++)
      {
         yoV.get(i).set(v.get(i));
      }
   }

   public static void getFromYoVariablesMatrix(DenseMatrix64F m, List<List<DoubleYoVariable>> yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         List<DoubleYoVariable> row = yoM.get(i);
         for (int j = 0; j < m.getNumCols(); j++)
         {
            m.set(i, j, row.get(j).getDoubleValue());
         }
      }
   }

   public static void getFromYoVariablesSymmetric(DenseMatrix64F m, List<List<DoubleYoVariable>> yoM)
   {
      int size = m.getNumRows();

      for (int i = 0; i < size; i++)
      {
         List<DoubleYoVariable> row = yoM.get(i);
         for (int j = 0; j <= i; j++)
         {
            double value = row.get(j).getDoubleValue();
            m.set(i, j, value);
            m.set(j, i, value);
         }
      }
   }

   public static void getFromYoVariablesVector(DenseMatrix64F m, List<DoubleYoVariable> yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         m.set(i, 0, yoM.get(i).getDoubleValue());
      }
   }

   private static List<DoubleYoVariable> getOrAddRow(List<List<DoubleYoVariable>> yoVariableArray, int i)
   {
      List<DoubleYoVariable> row;
      if (i < yoVariableArray.size())
      {
         row = yoVariableArray.get(i);
      }
      else
      {
         row = new ArrayList<DoubleYoVariable>();
         yoVariableArray.add(row);
      }
      return row;
   }

   public static int getNumberOfElementsForSymmetricMatrix(int size)
   {
      return size * (size + 1) / 2;
   }

   public static void checkPositiveSemiDefinite(DenseMatrix64F m)
   {
      if (!MatrixFeatures.isPositiveSemidefinite(m))
         throw new RuntimeException("Matrix is not positive semidefinite: " + m);
   }

   public static void checkSquare(DenseMatrix64F m)
   {
      if (!MatrixFeatures.isSquare(m))
         throw new RuntimeException("Matrix is not square: " + m);
   }

   public static void checkSize(DenseMatrix64F m1, DenseMatrix64F m2)
   {
      boolean nRowsNotEqual = m1.getNumRows() != m2.getNumRows();
      boolean nColsNotEqual = m1.getNumCols() != m2.getNumCols();

      if (nRowsNotEqual || nColsNotEqual)
      {
         throw new RuntimeException("Matrix sizes not equal: " + m1 + "\n\n" + m2);
      }
   }
}
