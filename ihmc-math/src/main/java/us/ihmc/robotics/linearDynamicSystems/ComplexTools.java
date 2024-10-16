package us.ihmc.robotics.linearDynamicSystems;

import org.ejml.data.Complex_F64;
import org.ejml.data.ZMatrixRMaj;

import us.ihmc.robotics.dataStructures.ComplexNumber;

public class ComplexTools
{
   public static ZMatrixRMaj ihmcComplexToEjmlComplex(ComplexMatrix ihmc)
   {
      ZMatrixRMaj ret = new ZMatrixRMaj(ihmc.getRowDimension(), ihmc.getColumnDimension());

      for (int i = 0; i < ret.numRows; i++)
      {
         for (int j = 0; j < ret.numCols; j++)
         {
            ComplexNumber c = ihmc.get(i, j);
            ret.set(i, j, c.real(), c.imag());
         }
      }

      return ret;
   }

   public static ComplexMatrix ejmlToIhmComplex(ZMatrixRMaj ejml)
   {
      ComplexMatrix ihmc = new ComplexMatrix(ejml.numRows, ejml.numCols);

      Complex_F64 c = new Complex_F64();

      for (int i = 0; i < ejml.numRows; i++)
      {
         for (int j = 0; j < ejml.numCols; j++)
         {
            ejml.get(i, j, c);
            ihmc.set(i, j, new ComplexNumber(c.getReal(), c.getImaginary()));
         }
      }

      return ihmc;
   }

   public static ComplexNumber[][] copyEjmlComplexIntoIhmcComplexNumber2dArray(ZMatrixRMaj ejml)
   {
      ComplexNumber[][] ihmcComplexNumber2dArray = new ComplexNumber[ejml.getNumRows()][ejml.getNumCols()];

      Complex_F64 c = new Complex_F64();
      for (int i = 0; i < ejml.getNumRows(); i++)
      {
         for (int j = 0; j < ejml.getNumCols(); j++)
         {
            ejml.get(i, j, c);

            ihmcComplexNumber2dArray[i][j] = new ComplexNumber(c.getReal(), c.getImaginary());
         }
      }

      return ihmcComplexNumber2dArray;
   }

// public static flanagan.complex.ComplexMatrix ihmcComplexMatrixToFlanaganComplexMatrix(ComplexMatrix ihmcComplexMatrix)
// {
//    flanagan.complex.ComplexMatrix flanaganMatrix = new flanagan.complex.ComplexMatrix(ihmcComplexMatrix.getRowDimension(), ihmcComplexMatrix.getColumnDimension());
//    
//    ComplexNumber complexNumber;
//    for (int i = 0; i < ihmcComplexMatrix.getRowDimension(); i++)
//    {
//       for (int j = 0; j < ihmcComplexMatrix.getColumnDimension(); j++)
//       {
//          complexNumber = ihmcComplexMatrix.get(i, j);
//          flanaganMatrix.setElement(i, j, new flanagan.complex.Complex(complexNumber.real(), complexNumber.imag()));
//       }
//    }
//    
//    return flanaganMatrix;
// }
// 
// public static ComplexMatrix flanaganComplexMatrixToIhmcComplexMatrix(flanagan.complex.ComplexMatrix flanaganComplexMatrix)
// {
//    ComplexMatrix ihmcComplexMatrix = new ComplexMatrix(flanaganComplexMatrix.getNrow(), flanaganComplexMatrix.getNcol());
//    
//    flanagan.complex.Complex flanaganComplex;
//    for (int i = 0; i < flanaganComplexMatrix.getNrow(); i++)
//    {
//       for (int j = 0; j < flanaganComplexMatrix.getNcol(); j++)
//       {
//          flanaganComplex = flanaganComplexMatrix.getElementReference(i, j);
//          ihmcComplexMatrix.set(i, j, new ComplexNumber(flanaganComplex.getReal(), flanaganComplex.getImag()));
//       }
//    }
//    
//    return ihmcComplexMatrix;
// }
// 
//   public static ComplexNumber[][] copyFlaniganComplexMatrixIntoIhmcComplexNumber2dArray(flanagan.complex.ComplexMatrix flanaganComplexMatrix)
//   {
//      ComplexNumber[][] ihmcComplexNumber2dArray = new ComplexNumber[flanaganComplexMatrix.getNrow()][flanaganComplexMatrix.getNcol()];
//
//      flanagan.complex.Complex flanaganComplex;
//      for (int i = 0; i < flanaganComplexMatrix.getNrow(); i++)
//      {
//         for (int j = 0; j < flanaganComplexMatrix.getNcol(); j++)
//         {
//            flanaganComplex = flanaganComplexMatrix.getElementCopy(i, j);
//
//            ihmcComplexNumber2dArray[i][j] = new ComplexNumber(flanaganComplex.getReal(), flanaganComplex.getImag());
//         }
//      }
//
//      return ihmcComplexNumber2dArray;
//   }
}
