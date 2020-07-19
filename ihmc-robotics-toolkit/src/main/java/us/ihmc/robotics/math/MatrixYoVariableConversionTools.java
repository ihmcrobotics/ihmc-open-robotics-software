package us.ihmc.robotics.math;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MatrixYoVariableConversionTools
{
   public static void populateYoVariables(YoDouble[][] yoVariableArray, String prefix, YoRegistry registry)
   {
      for (int i = 0; i < yoVariableArray.length; i++)
      {
         for (int j = 0; j < yoVariableArray[0].length; j++)
         {
            yoVariableArray[i][j] = new YoDouble(prefix + Integer.toString(i) + "_" + Integer.toString(j), registry);
         }
      }
   }

   public static void populateYoVariablesSymmetric(YoDouble[] yoVariableArray, String prefix, int size, YoRegistry registry)
   {
      int n = 0;
      for (int i = 0; i < size; i++)
      {
         for (int j = i; j < size; j++)
         {
            yoVariableArray[n] = new YoDouble(prefix + Integer.toString(i) + "_" + Integer.toString(j), registry);
            n++;
         }
      }
   }

   public static void populateYoVariables(YoDouble[] yoVariableArray, String prefix, YoRegistry registry)
   {
      for (int i = 0; i < yoVariableArray.length; i++)
      {
         yoVariableArray[i] = new YoDouble(prefix + Integer.toString(i), registry);
      }
   }

   public static void storeInYoVariables(DMatrixRMaj m, YoDouble[][] yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         for (int j = 0; j < m.getNumCols(); j++)
         {
            yoM[i][j].set(m.get(i, j));
         }
      }
   }

   public static void storeInYoVariablesSymmetric(DMatrixRMaj m, YoDouble[] yoM)
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

   public static void storeInYoVariables(DMatrixRMaj v, YoDouble[] yoV)
   {
      for (int i = 0; i < v.getNumRows(); i++)
      {
         yoV[i].set(v.get(i));
      }
   }

   public static void getFromYoVariables(DMatrixRMaj m, YoDouble[][] yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         for (int j = 0; j < m.getNumCols(); j++)
         {
            m.set(i, j, yoM[i][j].getDoubleValue());
         }
      }
   }

   public static void getFromYoVariablesSymmetric(DMatrixRMaj m, YoDouble[] yoM)
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

   public static void getFromYoVariables(DMatrixRMaj m, YoDouble[] yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         m.set(i, 0, yoM[i].getDoubleValue());
      }
   }

   public static void populateYoVariablesMatrix(List<List<YoDouble>> yoVariableArray, int nRows, int nColumns, String prefix,
         YoRegistry registry)
   {
      for (int i = 0; i < nRows; i++)
      {
         List<YoDouble> row = getOrAddRow(yoVariableArray, i);
         for (int j = row.size(); j < nColumns; j++)
         {
            row.add(new YoDouble(prefix + Integer.toString(i) + "_" + Integer.toString(j), registry));
         }
      }
   }

   public static void populateYoVariablesSymmetricMatrix(List<List<YoDouble>> yoVariableArray, int size, String prefix, YoRegistry registry)
   {
      for (int i = 0; i < size; i++)
      {
         List<YoDouble> row = getOrAddRow(yoVariableArray, i);

         for (int j = row.size(); j <= i; j++)
         {
            row.add(new YoDouble(prefix + Integer.toString(i) + "_" + Integer.toString(j), registry));
         }
      }
   }

   public static void populateYoVariablesVector(List<YoDouble> yoVariableArray, int size, String prefix, YoRegistry registry)
   {
      for (int i = yoVariableArray.size(); i < size; i++)
      {
         yoVariableArray.add(new YoDouble(prefix + Integer.toString(i), registry));
      }
   }

   public static void storeInYoVariablesMatrix(DMatrixRMaj m, List<List<YoDouble>> yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         List<YoDouble> row = yoM.get(i);
         for (int j = 0; j < m.getNumCols(); j++)
         {
            row.get(j).set(m.get(i, j));
         }
      }
   }

   public static void storeInYoVariablesSymmetric(DMatrixRMaj m, List<List<YoDouble>> yoM)
   {
      int size = m.getNumRows();

      for (int i = 0; i < size; i++)
      {
         List<YoDouble> row = yoM.get(i);
         for (int j = 0; j <= i; j++)
         {
            row.get(j).set(m.get(i, j));
         }
      }
   }

   public static void storeInYoVariablesVector(DMatrixRMaj v, List<YoDouble> yoV)
   {
      for (int i = 0; i < v.getNumRows(); i++)
      {
         yoV.get(i).set(v.get(i));
      }
   }

   public static void getFromYoVariablesMatrix(DMatrixRMaj m, List<List<YoDouble>> yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         List<YoDouble> row = yoM.get(i);
         for (int j = 0; j < m.getNumCols(); j++)
         {
            m.set(i, j, row.get(j).getDoubleValue());
         }
      }
   }

   public static void getFromYoVariablesSymmetric(DMatrixRMaj m, List<List<YoDouble>> yoM)
   {
      int size = m.getNumRows();

      for (int i = 0; i < size; i++)
      {
         List<YoDouble> row = yoM.get(i);
         for (int j = 0; j <= i; j++)
         {
            double value = row.get(j).getDoubleValue();
            m.set(i, j, value);
            m.set(j, i, value);
         }
      }
   }

   public static void getFromYoVariablesVector(DMatrixRMaj m, List<YoDouble> yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         m.set(i, 0, yoM.get(i).getDoubleValue());
      }
   }

   private static List<YoDouble> getOrAddRow(List<List<YoDouble>> yoVariableArray, int i)
   {
      List<YoDouble> row;
      if (i < yoVariableArray.size())
      {
         row = yoVariableArray.get(i);
      }
      else
      {
         row = new ArrayList<YoDouble>();
         yoVariableArray.add(row);
      }
      return row;
   }

   public static int getNumberOfElementsForSymmetricMatrix(int size)
   {
      return size * (size + 1) / 2;
   }

   public static void checkPositiveSemiDefinite(DMatrixRMaj m)
   {
      if (!MatrixFeatures_DDRM.isPositiveSemidefinite(m))
         throw new RuntimeException("Matrix is not positive semidefinite: " + m);
   }

   public static void checkSquare(DMatrixRMaj m)
   {
      if (!MatrixFeatures_DDRM.isSquare(m))
         throw new RuntimeException("Matrix is not square: " + m);
   }

   public static void checkSize(DMatrixRMaj m1, DMatrixRMaj m2)
   {
      boolean nRowsNotEqual = m1.getNumRows() != m2.getNumRows();
      boolean nColsNotEqual = m1.getNumCols() != m2.getNumCols();

      if (nRowsNotEqual || nColsNotEqual)
      {
         throw new RuntimeException("Matrix sizes not equal: " + m1 + "\n\n" + m2);
      }
   }
}
