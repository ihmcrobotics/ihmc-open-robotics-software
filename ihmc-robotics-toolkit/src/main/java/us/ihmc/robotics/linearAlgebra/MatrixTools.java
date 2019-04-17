package us.ihmc.robotics.linearAlgebra;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import org.ejml.alg.dense.misc.TransposeAlgs;
import org.ejml.data.DenseMatrix64F;
import org.ejml.data.RowD1Matrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixDimensionException;
import org.ejml.ops.MatrixIO;

import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;

public class MatrixTools
{
   public final static Matrix3D IDENTITY = new Matrix3D();

   static
   {
      IDENTITY.setIdentity();
   }

   /**
    * Sets all the entries of a matrix to NaN
    *
    * @param matrix
    */
   public static void setToNaN(DenseMatrix64F matrix)
   {
      setToValue(matrix, Double.NaN);
   }

   /**
    * Sets all entries of a matrix to zero
    *
    * @param matrix
    */
   public static void setToZero(DenseMatrix64F matrix)
   {
      setToValue(matrix, 0.0);
   }

   private static void setToValue(DenseMatrix64F matrix, double val)
   {
      for (int i = 0; i < matrix.numRows; i++)
      {
         for (int j = 0; j < matrix.numCols; j++)
         {
            matrix.unsafe_set(i, j, val);
         }
      }
   }

   public static boolean containsNaN(DenseMatrix64F matrix)
   {
      for (int i = 0; i < matrix.numRows; i++)
      {
         for (int j = 0; j < matrix.numCols; j++)
         {
            if (Double.isNaN(matrix.unsafe_get(i, j)))
               return true;
         }
      }
      return false;
   }

   /**
    * This method tries to be smart about converting the various yaml fields to DenseMatrix64F
    * 
    * @param val
    * @param fieldName
    * @param object
    * 
    *           Map<String, Object> object = (Map<String, Object>) yaml.load(input);
    *           yamlFieldToMatrix(beq,"beq",object);
    */
   public static DenseMatrix64F yamlFieldToMatrix(DenseMatrix64F val, String fieldName, Map<String, Object> object)
   {
      if (val == null)
         val = new DenseMatrix64F(1, 1);
      if (object.get(fieldName) instanceof ArrayList<?>)
      {
         ArrayList<?> arrayList = (ArrayList<?>) object.get(fieldName);
         try
         {
            if (arrayList.get(0) instanceof ArrayList)
            {
               @SuppressWarnings("unchecked")
               ArrayList<ArrayList<Double>> tmp2DArrayList = (ArrayList<ArrayList<Double>>) arrayList;
               // 2D
               val.reshape(tmp2DArrayList.size(), tmp2DArrayList.get(0).size());
               for (int i = 0; i < tmp2DArrayList.size(); i++)
               {
                  for (int j = 0; j < tmp2DArrayList.get(0).size(); j++)
                  {
                     val.set(i, j, tmp2DArrayList.get(i).get(j));
                  }
               }
            }
            else
            {
               @SuppressWarnings("unchecked")
               ArrayList<Double> tmp1DArrayList = (ArrayList<Double>) arrayList;
               // 1D
               val.reshape(1, arrayList.size());
               for (int i = 0; i < tmp1DArrayList.size(); i++)
               {
                  val.set(0, i, tmp1DArrayList.get(i));
               }
            }
         }
         catch (Exception e)
         {
            //Field is empty
            val = null;
         }
      }
      else if (object.get(fieldName) instanceof Double)
      {
         val.reshape(1, 1);
         Double tmpDouble = (Double) object.get(fieldName);

         val.set(0, 0, tmpDouble);
      }
      else
      {
         throw new RuntimeException("Unsupported data type:" + object.get(fieldName).getClass());
      }
      return val;
   }

   public static boolean isEmptyMatrix(DenseMatrix64F m)
   {
      if (m == null)
         throw new RuntimeException("Matrix is null");
      if (m.numCols == 0 && m.numRows == 0)
         return true;
      else
         return false;
   }

   /**
    * Same as CommonOps.mult but allow a,b,c to be zero rol/col matrices c = a * b;
    */

   public static void multAllowEmptyMatrix(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c)
   {
      if (a.numRows != c.numRows || b.numCols != c.numCols)
         throw new RuntimeException("matrix c dimension should be " + a.numRows + " x " + b.numCols);

      if (a.numCols != b.numRows)
         throw new RuntimeException("a, b matrices are not compatible, check their dimensions");

      if (a.numCols == 0 && b.numRows == 0)
         c.zero();
      else
         CommonOps.mult(a, b, c);
   }

   /**
    * same as CommonOps.multAdd but allow a,b,c to be empty matrices
    */

   public static void multAddAllowEmptyMatrix(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c)
   {
      if (a.numRows != c.numRows || b.numCols != c.numCols)
         throw new RuntimeException("matrix c dimension should be " + a.numRows + " x " + b.numCols);

      if (a.numCols != b.numRows)
         throw new RuntimeException("a, b matrices are not compatible, check their dimensions");

      if (a.numRows != 0 && b.numCols != 0)
         CommonOps.multAdd(a, b, c);
   }

   /**
    * fill column col of matrix m with value val
    */
   public static void fillColumn(DenseMatrix64F m, int col, double val)
   {
      for (int i = 0; i < m.numRows; i++)
         m.set(i, col, val);
   }

   /**
    * set matrixToSet from vectorElements
    */

   public static void setVector(DenseMatrix64F vectorToSet, double[] vectorElements)
   {
      if (vectorToSet.numCols != 1)
         throw new RuntimeException("first argument has to be column vector");
      for (int i = 0; i < vectorElements.length; i++)
      {
         vectorToSet.set(i, 0, vectorElements[i]);
      }
   }

   /**
    * return a newVector based on vectorElements
    */
   public static DenseMatrix64F createVector(double[] vectorElements)
   {
      DenseMatrix64F ret = new DenseMatrix64F(vectorElements.length, 1);
      setVector(ret, vectorElements);

      return ret;
   }

   /**
    * find the index of the first largest element in a vector in the range of [startIndex,
    * endIndex), not including the endIndex
    */
   public static int findMaxElementIndex(double m[], int startIndex, int endIndex)
   {
      double maxVal = Double.NEGATIVE_INFINITY;
      int maxIndex = -1;
      for (int i = startIndex; i < endIndex; i++)
      {
         if (m[i] > maxVal)
         {
            maxVal = m[i];
            maxIndex = i;
         }
      }
      return maxIndex;
   }

   /**
    * AtGA = A' * G * A
    * 
    * if AtGA is null, a new Matrix will be allocated and returned
    */
   public static DenseMatrix64F multQuad(DenseMatrix64F A, DenseMatrix64F G, DenseMatrix64F AtGA)
   {
      DenseMatrix64F out;
      if (AtGA == null)
      {
         out = new DenseMatrix64F(A.numCols, A.numCols);
      }
      else
      {
         out = AtGA;
      }

      DenseMatrix64F tmp = new DenseMatrix64F(A.numCols, G.numCols);
      CommonOps.multTransA(A, G, tmp);
      CommonOps.mult(tmp, A, out);
      return out;
   }

   public static double multMatrixRowVector(DenseMatrix64F a, int row, DenseMatrix64F b)
   {
      if (a.numCols != b.numRows)
         throw new RuntimeException("a.numCols should be equal to b.numRows");
      if (b.numCols != 1)
         throw new RuntimeException("b should be a column vector");

      int rowHeadIndex = a.getIndex(row, 0);
      double total = 0;
      for (int i = 0; i < a.numCols; i++)
      {
         total += a.get(rowHeadIndex + i) * b.get(i);
      }

      return total;

   }

   static enum Relation
   {
      LESSTHAN, EQUAL, GREATERTHAN
   };

   public static DenseMatrix64F compare(DenseMatrix64F a, Relation r, DenseMatrix64F b, double eps)
   {
      assert (a.numCols == b.numCols && a.numRows == b.numRows);
      DenseMatrix64F result = new DenseMatrix64F(a.numRows, a.numCols);
      for (int i = 0; i < a.numCols; i++)
         for (int j = 0; j < a.numRows; j++)
         {

            double testVal = a.get(i, j) - b.get(i, j);
            switch (r)
            {
            case LESSTHAN:
               result.unsafe_set(i, j, testVal < 0 ? 1 : 0);
               break;
            case EQUAL:
               result.unsafe_set(i, j, Math.abs(testVal) < eps ? 1 : 0);
               break;
            case GREATERTHAN:
               result.unsafe_set(i, j, testVal > 0 ? 1 : 0);
               break;
            default:
               throw new RuntimeException("shoun't be here");
            }
         }
      return result;
   }

   /**
    * Set a column of a Matrix to an Array
    *
    * @param matrix Matrix to set
    * @param column Column
    * @param columnValues
    */
   public static void setMatrixColumnFromArray(DenseMatrix64F matrix, int column, double[] columnValues)
   {
      for (int i = 0; i < matrix.numRows; i++)
      {
         matrix.unsafe_set(i, column, columnValues[i]);
      }
   }

   /**
    * Set a column of a Matrix to an Array
    *
    * @param matrix Matrix to set
    * @param column Column
    * @param columnValues
    */
   public static void setMatrixColumnFromArray(DenseMatrix64F matrix, int column, double[] columnValues, int startRow)
   {
      for (int i = startRow; i < columnValues.length; i++)
      {
         matrix.unsafe_set(i, column, columnValues[i]);
      }
   }

   public static void setMatrixFromOneBasedArray(DenseMatrix64F ret, double[] oneBasedArray)
   {
      for (int i = 0; i < oneBasedArray.length - 1; i++)
      {
         ret.set(i, 0, oneBasedArray[i + 1]);
      }
   }

   public static void setOneBasedArrayFromMatrix(double[] oneBasedArrayToPack, DenseMatrix64F matrix)
   {
      int index = 1;
      for (int i = 0; i < matrix.getNumRows(); i++)
      {
         oneBasedArrayToPack[index++] = matrix.get(i, 0);
      }
   }

   /**
    *
    * Differentiates a row vector by subtracting the previous element from the current element
    *
    * @param vectorToDiff Row vector
    * @param startRow Row to start at
    * @param numberOfRows Rows to differentiate
    * @param vectorToPack Result row vector
    */
   public static void diff(DenseMatrix64F vectorToDiff, int startRow, int numberOfRows, DenseMatrix64F vectorToPack)
   {
      for (int i = 1; i < numberOfRows; i++)
      {
         vectorToPack.unsafe_set(i - 1, 0, vectorToDiff.unsafe_get(startRow + i, 0) - vectorToDiff.unsafe_get(startRow + i - 1, 0));
      }
   }

   /**
    * Differentiates an array by subtracting the previous element from the current element
    *
    * @param vectorToDiff
    * @param vectorToPack Result row vector
    */
   public static void diff(double[] vectorToDiff, DenseMatrix64F vectorToPack)
   {
      for (int i = 1; i < vectorToDiff.length; i++)
      {
         vectorToPack.unsafe_set(i - 1, 0, vectorToDiff[i] - vectorToDiff[i - 1]);
      }
   }

   public static void numericallyDifferentiate(DenseMatrix64F derivativeToPack, DenseMatrix64F previousMatrixToUpdate, DenseMatrix64F newMatrix, double dt)
   {
      derivativeToPack.set(newMatrix);
      CommonOps.subtractEquals(derivativeToPack, previousMatrixToUpdate);
      CommonOps.scale(1.0 / dt, derivativeToPack);
      previousMatrixToUpdate.set(newMatrix);
   }

   /**
    *
    * Sets a block of a matrix
    *
    * @param dest Set a block of this matrix
    * @param destStartRow Row index of the top left corner of the block to set
    * @param destStartColumn Column index of the top left corner of the block to set
    * @param src Get a block of this matrix
    * @param srcStartRow Row index of the top left corner of the block to use from otherMatrix
    * @param srcStartColumn Column index of the top left corner of the block to use from otherMatrix
    * @param numberOfRows Row size of the block
    * @param numberOfColumns Column size of the block
    * @param scale Scale the block from otherMatrix by this value
    *
    */
   public static void setMatrixBlock(DenseMatrix64F dest, int destStartRow, int destStartColumn, DenseMatrix64F src, int srcStartRow, int srcStartColumn,
                                     int numberOfRows, int numberOfColumns, double scale)
   {
      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            dest.unsafe_set(destStartRow + i, destStartColumn + j, scale * src.unsafe_get(srcStartRow + i, srcStartColumn + j));
         }
      }
   }

   /**
    * Sets matrixToPack to the entries of input in the specified rows and columns
    *
    * @param matrixToPack matrix to pack
    * @param input input matrix
    * @param rows rows of input matrix to use in setting matrix to pack
    * @param columns columns of input matrix to use in setting matrix to pack
    */
   public static void getMatrixBlock(DenseMatrix64F matrixToPack, DenseMatrix64F input, int[] rows, int[] columns)
   {
      if ((rows.length != matrixToPack.getNumRows()) || (columns.length != matrixToPack.getNumCols()))
      {
         throw new RuntimeException("The size of matrixToPack is not rows.length * columns.length");
      }

      int newI = 0;
      for (int i : rows)
      {
         int newJ = 0;
         for (int j : columns)
         {
            matrixToPack.set(newI, newJ, input.get(i, j));
            newJ++;
         }

         newI++;
      }
   }

   /**
    *
    * Adds to a block of a matrix
    *
    * @param dest Add to a block of this matrix
    * @param destStartRow Row index of the top left corner of the block to set
    * @param destStartColumn Column index of the top left corner of the block to set
    * @param src Get a block of this matrix
    * @param srcStartRow Row index of the top left corner of the block to use from otherMatrix
    * @param srcStartColumn Column index of the top left corner of the block to use from otherMatrix
    * @param numberOfRows Row size of the block
    * @param numberOfColumns Column size of the block
    * @param scale Scale the block from otherMatrix by this value
    *
    */
   public static void addMatrixBlock(DenseMatrix64F dest, int destStartRow, int destStartColumn, DenseMatrix64F src, int srcStartRow, int srcStartColumn,
                                     int numberOfRows, int numberOfColumns, double scale)
   {
      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            dest.unsafe_set(destStartRow + i, destStartColumn + j,
                            dest.unsafe_get(destStartRow + i, destStartColumn + j) + scale * src.unsafe_get(srcStartRow + i, srcStartColumn + j));
         }
      }
   }

   public static void extractColumns(DenseMatrix64F source, int[] srcColumns, DenseMatrix64F dest, int destStartColumn)
   {
      for (int i : srcColumns)
      {
         CommonOps.extract(source, 0, source.getNumRows(), i, i + 1, dest, 0, destStartColumn);
         destStartColumn++;
      }
   }

   public static void extractRows(DenseMatrix64F source, int[] srcRows, DenseMatrix64F dest, int destStartRow)
   {
      for (int i : srcRows)
      {
         CommonOps.extract(source, i, i + 1, 0, source.getNumCols(), dest, destStartRow, 0);
         destStartRow++;
      }
   }

   public static void extractColumns(DenseMatrix64F source, TIntArrayList srcColumns, DenseMatrix64F dest, int destStartColumn)
   {
      for (int i = 0; i < srcColumns.size(); i++)
      {
         int index = srcColumns.get(i);
         CommonOps.extract(source, 0, source.getNumRows(), index, index + 1, dest, 0, destStartColumn);
         destStartColumn++;
      }
   }

   public static void linspace(double[] vector, double d1, double d2, int n)
   {
      // double[] vector = new double[n];

      for (int i = 0; i < n; i++)
      {
         vector[i] = d1 + i * ((d2 - d1) / ((double) (n - 1)));
      }

      // return vector;

   }

   public static double[] linspace(double d1, double d2, int n)
   {
      double[] vector = new double[n];
      linspace(vector, d1, d2, n);

      return vector;

   }

   public static DenseMatrix64F flipUpDown(DenseMatrix64F matrix)
   {
      DenseMatrix64F ret = matrix.copy();
      int nRows = ret.getNumRows();
      int nColumns = ret.getNumCols();

      for (int i = 0; i < nRows; i++)
      {
         int newRowNumber = nRows - i - 1;
         for (int j = 0; j < nColumns; j++)
         {
            ret.set(newRowNumber, j, matrix.get(i, j));
         }
      }

      return ret;
   }

   public static DenseMatrix64F mult(DenseMatrix64F A, DenseMatrix64F B)
   {
      DenseMatrix64F C = new DenseMatrix64F(A.getNumRows(), B.getNumCols());
      CommonOps.mult(A, B, C);

      return C;
   }

   public static void addDiagonal(DenseMatrix64F matrix, double scalar)
   {
      int n = Math.max(matrix.getNumRows(), matrix.getNumCols());
      for (int i = 0; i < n; i++)
      {
         matrix.add(i, i, scalar);
      }
   }

   /**
    * Set diagonal elements of matrix to diagValues
    *
    * @param matrix
    * @param diagValues
    */
   public static void setMatrixDiag(Matrix3D matrix, double[] diagValues)
   {
      for (int i = 0; i < 3; i++)
      {
         matrix.setElement(i, i, diagValues[i]);
      }
   }

   /**
    * Set diagonal elements of matrix to diagValues
    *
    * @param matrix
    * @param diagValue
    */
   public static void setMatrixDiag(Matrix3D matrix, double diagValue)
   {
      matrix.setM00(diagValue);
      matrix.setM11(diagValue);
      matrix.setM22(diagValue);
   }

   /**
    * Sets all the diagonal elements equal to one and everything else equal to zero.
    * If this is a square matrix then it will be an identity matrix.
    *
    * @param mat A square matrix.
    */
   public static void setDiagonal( RowD1Matrix64F mat , double diagonalValue)
   {
      int width = mat.numRows < mat.numCols ? mat.numRows : mat.numCols;

      Arrays.fill(mat.data, 0, mat.getNumElements(), 0);

      int index = 0;
      for( int i = 0; i < width; i++ , index += mat.numCols + 1) {
         mat.data[index] = diagonalValue;
      }
   }

   /**
    * Returns the resulting matrix from vector1*transpose(vector2)
    *
    * @param vector1
    * @param vector2
    */
   public static Matrix3D vectorTimesVectorTranspose(Vector3DReadOnly vector1, Vector3DReadOnly vector2)
   {
      Matrix3D matrix = new Matrix3D();

      double[] array1 = new double[3];
      double[] array2 = new double[3];

      vector1.get(array1);
      vector2.get(array2);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            matrix.setElement(row, column, array1[row] * array2[column]);
         }
      }

      return matrix;
   }

   /**
    * Returns the resulting matrix from vector*transpose(vector)
    *
    * @param vector
    */
   public static Matrix3D vectorTimesVectorTranspose(Vector3DReadOnly vector)
   {
      return vectorTimesVectorTranspose(vector, vector);
   }

   public static void vectorToSkewSymmetricMatrix(DenseMatrix64F matrixToPack, Tuple3DReadOnly tuple)
   {
      matrixToPack.set(0, 0, 0.0);
      matrixToPack.set(0, 1, -tuple.getZ());
      matrixToPack.set(0, 2, tuple.getY());

      matrixToPack.set(1, 0, tuple.getZ());
      matrixToPack.set(1, 1, 0.0);
      matrixToPack.set(1, 2, -tuple.getX());

      matrixToPack.set(2, 0, -tuple.getY());
      matrixToPack.set(2, 1, tuple.getX());
      matrixToPack.set(2, 2, 0.0);
   }

   /*
    * M = \tilde{a} * \tilde{b}
    */
   public static void setTildeTimesTilde(Matrix3D M, Tuple3DReadOnly a, Tuple3DReadOnly b)
   {
      double axbx = a.getX() * b.getX();
      double ayby = a.getY() * b.getY();
      double azbz = a.getZ() * b.getZ();

      M.setM00(-azbz - ayby);
      M.setM01(a.getY() * b.getX());
      M.setM02(a.getZ() * b.getX());

      M.setM10(a.getX() * b.getY());
      M.setM11(-axbx - azbz);
      M.setM12(a.getZ() * b.getY());

      M.setM20(a.getX() * b.getZ());
      M.setM21(a.getY() * b.getZ());
      M.setM22(-axbx - ayby);
   }

   public static void setMatrixColumnToVector(int columnIndex, DenseMatrix64F Matrix, DenseMatrix64F vector)
   {
      for (int i = 0; i < Matrix.getNumRows(); i++)
      {
         Matrix.set(i, columnIndex, vector.get(i, columnIndex));
      }
   }

   public static void setMatrixRowToVector(int rowIndex, DenseMatrix64F Matrix, DenseMatrix64F vector)
   {
      for (int i = 0; i < Matrix.getNumCols(); i++)
      {
         Matrix.set(rowIndex, i, vector.get(i));
      }
   }

   public static void tranformSe3IntoTransform3D(Se3_F64 from, RigidBodyTransform to)
   {

      DenseMatrix64F R = from.getR();
      Vector3D_F64 T = from.getT();

      RotationMatrix Rd = new RotationMatrix(R);

      to.setRotation(Rd);
      to.setTranslation(new Vector3D(T.x, T.y, T.z));
   }

   public static void transformFramePoint2dIntoColumnVector(DenseMatrix64F matrix, FramePoint2D framePoint)
   {
      matrix.set(0, 0, framePoint.getX());
      matrix.set(1, 0, framePoint.getY());
   }

   public static void transformColumnVectorIntoFramePoint2d(DenseMatrix64F matrix, FramePoint2D framePoint)
   {
      framePoint.setX(matrix.get(0, 0));
      framePoint.setY(matrix.get(1, 0));
   }

   public static void transformFrameVector2dIntoColumnVector(DenseMatrix64F matrix, FrameVector2D frameVector)
   {
      matrix.set(0, 0, frameVector.getX());
      matrix.set(1, 0, frameVector.getY());
   }

   public static void transformColumnVectorIntoFrameVector2d(DenseMatrix64F matrix, FrameVector2D frameVector)
   {
      frameVector.setX(matrix.get(0, 0));
      frameVector.setY(matrix.get(1, 0));
   }

   public static void transformFramePoint2dIntoRowVector(DenseMatrix64F matrix, FramePoint2D framePoint)
   {
      matrix.set(0, 0, framePoint.getX());
      matrix.set(0, 1, framePoint.getY());
   }

   public static void transformRowVectorIntoFramePoint2d(DenseMatrix64F matrix, FramePoint2D framePoint)
   {
      framePoint.setX(matrix.get(0, 0));
      framePoint.setX(matrix.get(0, 1));
   }

   public static void transformFrameVector2dIntoRowVector(DenseMatrix64F matrix, FrameVector2D frameVector)
   {
      matrix.set(0, 0, frameVector.getX());
      matrix.set(0, 1, frameVector.getY());
   }

   public static void transformRowVectorIntoFrameVector2d(DenseMatrix64F matrix, FrameVector2D frameVector)
   {
      frameVector.setX(matrix.get(0, 0));
      frameVector.setY(matrix.get(0, 1));
   }

   public static int denseMatrixToArrayColumnMajor(DenseMatrix64F src, int srcStartRow, int srcStartCol, int numRows, int numCols, double[] dest,
                                                   int destStartIndex)
   {
      int currentIndex = destStartIndex;
      for (int j = srcStartCol; j < srcStartCol + numCols; j++)
      {
         for (int i = srcStartRow; i < srcStartRow + numRows; i++)
         {
            dest[currentIndex++] = src.get(i, j);
         }
      }

      return currentIndex - destStartIndex;
   }

   public static void extractDiagonal(DenseMatrix64F matrix, double[] diagonal)
   {
      for (int i = 0; i < Math.min(matrix.getNumRows(), matrix.getNumCols()); i++)
      {
         diagonal[i] = matrix.get(i, i);
      }
   }

   public static int denseMatrixToArrayColumnMajor(DenseMatrix64F src, double[] dest)
   {
      return denseMatrixToArrayColumnMajor(src, 0, 0, src.getNumRows(), src.getNumCols(), dest, 0);
   }

   public static void extractTuple3dFromEJMLVector(Tuple3DBasics tuple3d, DenseMatrix64F ejmlVector, int[] indices)
   {
      tuple3d.setX(ejmlVector.get(indices[0], 0));
      tuple3d.setY(ejmlVector.get(indices[1], 0));
      tuple3d.setZ(ejmlVector.get(indices[2], 0));
   }

   public static void extractTuple3dFromEJMLVector(Tuple3DBasics tuple3d, DenseMatrix64F ejmlVector, int startIndex)
   {
      tuple3d.setX(ejmlVector.get(startIndex + 0, 0));
      tuple3d.setY(ejmlVector.get(startIndex + 1, 0));
      tuple3d.setZ(ejmlVector.get(startIndex + 2, 0));
   }

   public static void extractFrameTupleFromEJMLVector(FrameTuple3DBasics frameTuple, DenseMatrix64F ejmlVector, ReferenceFrame desiredFrame, int startIndex)
   {
      frameTuple.setToZero(desiredFrame);
      frameTuple.setX(ejmlVector.get(startIndex + 0, 0));
      frameTuple.setY(ejmlVector.get(startIndex + 1, 0));
      frameTuple.setZ(ejmlVector.get(startIndex + 2, 0));
   }

   public static void extractFixedFrameTupleFromEJMLVector(FixedFrameTuple3DBasics yoFrameTuple, DenseMatrix64F ejmlVector, int startIndex)
   {
      yoFrameTuple.setX(ejmlVector.get(startIndex + 0, 0));
      yoFrameTuple.setY(ejmlVector.get(startIndex + 1, 0));
      yoFrameTuple.setZ(ejmlVector.get(startIndex + 2, 0));
   }

   public static void extractAddFixedFrameTupleFromEJMLVector(FixedFrameTuple3DBasics yoFrameTuple, DenseMatrix64F ejmlVector, int startIndex)
   {
      yoFrameTuple.addX(ejmlVector.get(startIndex + 0, 0));
      yoFrameTuple.addY(ejmlVector.get(startIndex + 1, 0));
      yoFrameTuple.addZ(ejmlVector.get(startIndex + 2, 0));
   }

   public static void extractYoFrameQuaternionFromEJMLVector(YoFrameQuaternion yoFrameQuaternion, DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      double x = matrix.get(index++, 0);
      double y = matrix.get(index++, 0);
      double z = matrix.get(index++, 0);
      double w = matrix.get(index++, 0);
      yoFrameQuaternion.set(x, y, z, w);
   }

   public static void extractFrameOrientationFromEJMLVector(FrameQuaternion frameOrientation, DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      double x = matrix.get(index++, 0);
      double y = matrix.get(index++, 0);
      double z = matrix.get(index++, 0);
      double w = matrix.get(index++, 0);
      frameOrientation.set(x, y, z, w);
   }

   public static void insertTuple3dIntoEJMLVector(Tuple3DReadOnly tuple3d, DenseMatrix64F ejmlVector, int[] indices)
   {
      ejmlVector.set(indices[0], 0, tuple3d.getX());
      ejmlVector.set(indices[1], 0, tuple3d.getY());
      ejmlVector.set(indices[2], 0, tuple3d.getZ());
   }

   public static void insertFrameTupleIntoEJMLVector(FrameTuple3DReadOnly frameTuple, DenseMatrix64F ejmlVector, int startIndex)
   {
      ejmlVector.set(startIndex + 0, 0, frameTuple.getX());
      ejmlVector.set(startIndex + 1, 0, frameTuple.getY());
      ejmlVector.set(startIndex + 2, 0, frameTuple.getZ());
   }

   public static void insertFrameQuaternionIntoEJMLVector(FrameQuaternionReadOnly frameOrientation, DenseMatrix64F matrix, int rowStart)
   {
      int index = rowStart;
      matrix.set(index++, 0, frameOrientation.getX());
      matrix.set(index++, 0, frameOrientation.getY());
      matrix.set(index++, 0, frameOrientation.getZ());
      matrix.set(index++, 0, frameOrientation.getS());
   }

   public static <T> int computeIndicesIntoVector(List<T> keys, Map<T, Integer> indices, Map<T, Integer> sizes)
   {
      int stateStartIndex = 0;

      for (T port : keys)
      {
         indices.put(port, stateStartIndex);
         stateStartIndex += sizes.get(port);
      }

      return stateStartIndex;
   }

   public static <RowKeyType, ColumnKeyType> void insertMatrixBlock(DenseMatrix64F bigMatrix, DenseMatrix64F matrixBlock, RowKeyType rowKey,
                                                                    Map<? super RowKeyType, Integer> rowStartIndices, ColumnKeyType columnKey,
                                                                    Map<? super ColumnKeyType, Integer> columnStartIndices)
   {
      if (matrixBlock != null)
      {
         int rowStartIndex = rowStartIndices.get(rowKey);
         int columnStartIndex = columnStartIndices.get(columnKey);
         CommonOps.insert(matrixBlock, bigMatrix, rowStartIndex, columnStartIndex);
      }
   }

   public static <RowKeyType> void insertVectorBlock(DenseMatrix64F bigVector, DenseMatrix64F vectorBlock, RowKeyType rowKey,
                                                     Map<? super RowKeyType, Integer> startIndices)
   {
      if (vectorBlock != null)
      {
         int rowStartIndex = startIndices.get(rowKey);
         CommonOps.insert(vectorBlock, bigVector, rowStartIndex, 0);
      }
   }

   public static <RowKeyType> void extractVectorBlock(DenseMatrix64F vectorBlockToPack, DenseMatrix64F bigVector, RowKeyType rowKey,
                                                      Map<? super RowKeyType, Integer> rowStartIndices, Map<? super RowKeyType, Integer> sizes)
   {
      int rowStartIndex = rowStartIndices.get(rowKey);
      int stateSize = sizes.get(rowKey);
      vectorBlockToPack.reshape(stateSize, 1);
      CommonOps.extract(bigVector, rowStartIndex, rowStartIndex + stateSize, 0, 1, vectorBlockToPack, 0, 0);
   }

   public static String denseMatrixToString(DenseMatrix64F mat)
   {
      ByteArrayOutputStream stream = new ByteArrayOutputStream();
      MatrixIO.print(new PrintStream(stream), mat, "%13.6g");

      return stream.toString();
   }

   public static void multOuter(Matrix3D result, Vector3DReadOnly vector)
   {
      double x = vector.getX();
      double y = vector.getY();
      double z = vector.getZ();

      result.setElement(0, 0, x * x);
      result.setElement(1, 1, y * y);
      result.setElement(2, 2, z * z);

      double xy = x * y;
      result.setElement(0, 1, xy);
      result.setElement(1, 0, xy);

      double xz = x * z;
      result.setElement(0, 2, xz);
      result.setElement(2, 0, xz);

      double yz = y * z;
      result.setElement(1, 2, yz);
      result.setElement(2, 1, yz);
   }

   /**
    * Multiply a 3x3 matrix by a 3x1 vector. Since result is stored in vector, the matrix must be
    * 3x3.
    * 
    * @param matrix
    * @param point
    */
   public static void mult(DenseMatrix64F matrix, Point3DBasics point)
   {
      if (matrix.numCols != 3 || matrix.numRows != 3)
      {
         throw new RuntimeException("Improperly sized matrices.");
      }
      double x = point.getX();
      double y = point.getY();
      double z = point.getZ();

      point.setX(matrix.get(0, 0) * x + matrix.get(0, 1) * y + matrix.get(0, 2) * z);
      point.setY(matrix.get(1, 0) * x + matrix.get(1, 1) * y + matrix.get(1, 2) * z);
      point.setZ(matrix.get(2, 0) * x + matrix.get(2, 1) * y + matrix.get(2, 2) * z);
   }

   /**
    * Multiply a 3x3 matrix by a 3x1 vector. Since result is stored in vector, the matrix must be
    * 3x3.
    * 
    * @param matrix
    * @param vector
    */
   public static void mult(DenseMatrix64F matrix, Vector3DBasics vector)
   {
      if (matrix.numCols != 3 || matrix.numRows != 3)
      {
         throw new RuntimeException("Improperly sized matrices.");
      }
      double x = vector.getX();
      double y = vector.getY();
      double z = vector.getZ();

      vector.setX(matrix.get(0, 0) * x + matrix.get(0, 1) * y + matrix.get(0, 2) * z);
      vector.setY(matrix.get(1, 0) * x + matrix.get(1, 1) * y + matrix.get(1, 2) * z);
      vector.setZ(matrix.get(2, 0) * x + matrix.get(2, 1) * y + matrix.get(2, 2) * z);
   }

   /**
    * Multiply a 4x4 matrix by a 4x1 vector. Since result is stored in vector, the matrix must be
    * 4x4.
    * 
    * @param matrix
    * @param vector
    */
   public static void mult(DenseMatrix64F matrix, Vector4DBasics vector)
   {
      if (matrix.numCols != 4 || matrix.numRows != 4)
      {
         throw new RuntimeException("Improperly sized matrices.");
      }
      double x = vector.getX();
      double y = vector.getY();
      double z = vector.getZ();
      double s = vector.getS();

      vector.setX(matrix.get(0, 0) * x + matrix.get(0, 1) * y + matrix.get(0, 2) * z + matrix.get(0, 3) * s);
      vector.setY(matrix.get(1, 0) * x + matrix.get(1, 1) * y + matrix.get(1, 2) * z + matrix.get(1, 3) * s);
      vector.setZ(matrix.get(2, 0) * x + matrix.get(2, 1) * y + matrix.get(2, 2) * z + matrix.get(2, 3) * s);
      vector.setS(matrix.get(3, 0) * x + matrix.get(3, 1) * y + matrix.get(3, 2) * z + matrix.get(3, 3) * s);
   }


   /**
    * Removes a row of the given matrix, indicated by {@code indexOfRowToRemove}.
    *
    * @param matrixToRemoveRowTo the matrix from which the row is to be removed. Modified.
    * @param indexOfRowToRemove the column index to remove.
    */
   public static void removeRow(DenseMatrix64F matrixToRemoveRowTo, int indexOfRowToRemove)
   {
      if (indexOfRowToRemove >= matrixToRemoveRowTo.getNumRows())
         throw new RuntimeException("The index indexOfRowToRemove was expected to be in [0, " + (matrixToRemoveRowTo.getNumRows() - 1) + "], but was: " + indexOfRowToRemove);

      for (int columnIndex = 0; columnIndex < matrixToRemoveRowTo.getNumCols(); columnIndex++)
      {
         for (int currentRowIndex = indexOfRowToRemove; currentRowIndex < matrixToRemoveRowTo.getNumRows() - 1; currentRowIndex++)
         {
            int nextRowIndex = currentRowIndex + 1;
            double valueOfNextRow = matrixToRemoveRowTo.get(nextRowIndex, columnIndex);
            double valueOfCurrentRow = matrixToRemoveRowTo.get(currentRowIndex, columnIndex);

            matrixToRemoveRowTo.set(nextRowIndex, columnIndex, valueOfCurrentRow);
            matrixToRemoveRowTo.set(currentRowIndex, columnIndex, valueOfNextRow);
         }
      }

      matrixToRemoveRowTo.reshape(matrixToRemoveRowTo.getNumRows() - 1, matrixToRemoveRowTo.getNumCols(), true);
   }

   /**
    * Removes a column of the given matrix, indicated by {@code indexOfColumnToRemove}.
    *
    * @param matrixToRemoveColumnTo the matrix from which the column is to be removed. Modified.
    * @param indexOfColumnToRemove the column index to remove.
    */
   public static void removeColumn(DenseMatrix64F matrixToRemoveColumnTo, int indexOfColumnToRemove)
   {
      if (indexOfColumnToRemove >= matrixToRemoveColumnTo.getNumCols())
         throw new RuntimeException("The index indexOfColumnToRemove was expected to be in [0, " + (matrixToRemoveColumnTo.getNumCols() - 1) + "], but was: " + indexOfColumnToRemove);

      int rowIndex = 1;
      for (int index = indexOfColumnToRemove + 1; index < matrixToRemoveColumnTo.getNumElements(); index++)
      {
         if (index == rowIndex * matrixToRemoveColumnTo.getNumCols() + indexOfColumnToRemove)
            rowIndex++;
         else
            matrixToRemoveColumnTo.set(index - rowIndex, matrixToRemoveColumnTo.get(index));
      }

      matrixToRemoveColumnTo.reshape(matrixToRemoveColumnTo.getNumRows(), matrixToRemoveColumnTo.getNumCols() - 1, true);
   }

   /**
    * Removes the rows of the given matrix that contain only zeros to an {@code epsilon}.
    * <p>
    * A row is determined to be a 'zero-row' if: &sum;<sub>i=0:N</sub>|M<sub>row,i</sub>| <= epsilon
    * </p>
    * 
    * @param matrixToModify the matrix from which 'zero-rows' have to be removed. Modified.
    * @param epsilon the tolerance to use for determining if a row is a 'zero-row'.
    */
   public static void removeZeroRows(DenseMatrix64F matrixToModify, double epsilon)
   {
      removeZeroRows(matrixToModify, 0, matrixToModify.getNumRows() - 1, epsilon);
   }

   /**
    * Removes the rows in [startRow, endRow] of the given matrix that contain only zeros to an
    * {@code epsilon}.
    * <p>
    * A row is determined to be a 'zero-row' if: &sum;<sub>i=0:N</sub>|M<sub>row,i</sub>| <= epsilon
    * </p>
    * 
    * @param matrixToModify the matrix from which 'zero-rows' have to be removed. Modified.
    * @param startRow the first row index to be tested.
    * @param endRow the last row index to be tested.
    * @param epsilon the tolerance to use for determining if a row is a 'zero-row'.
    * @throws IllegalArgumentException if {@code startRow > endRow}.
    * @throws RuntimeException if {@code startRow} or {@code endRow} are not in [0,
    *            {@code matrixToModify.getNumrows()}[.
    */
   public static void removeZeroRows(DenseMatrix64F matrixToModify, int startRow, int endRow, double epsilon)
   {
      if (startRow > endRow)
         throw new IllegalArgumentException("The index startRow cannot be greater than endRow.");
      if (startRow < 0 || startRow >= matrixToModify.getNumRows())
         throw new RuntimeException("The index startRow was expected to be in [0, " + (matrixToModify.getNumRows() - 1) + "], but was: " + startRow);
      if (endRow < 0 || endRow >= matrixToModify.getNumRows())
         throw new RuntimeException("The index endRow was expected to be in [0, " + (matrixToModify.getNumRows() - 1) + "], but was: " + endRow);

      for (int rowIndex = endRow; rowIndex >= startRow; rowIndex--)
      {
         double sumOfRowElements = 0.0;

         for (int columnIndex = 0; columnIndex < matrixToModify.getNumCols(); columnIndex++)
         {
            sumOfRowElements += Math.abs(matrixToModify.get(rowIndex, columnIndex));
         }

         boolean isZeroRow = MathTools.epsilonEquals(sumOfRowElements, 0.0, epsilon);
         if (isZeroRow)
            removeRow(matrixToModify, rowIndex);
      }
   }

   /**
    * <p>
    * Transposes matrix 'a' and stores the results in 'b':<br>
    * <br>
    * b<sub>ij</sub> = &alpha;*a<sub>ji</sub><br>
    * where 'b' is the scaled transpose of 'a'.
    * </p>
    * 
    * Transpose algorithm taken from
    * {@link TransposeAlgs#standard(org.ejml.data.RowD1Matrix64F, org.ejml.data.RowD1Matrix64F)}.
    * 
    * @param alpha the amount each element is multiplied by.
    * @param a The matrix that is to be scaled and transposed. Not modified.
    * @param b Where the scaled transpose is stored. Modified.
    */
   public static void scaleTranspose(double alpha, DenseMatrix64F a, DenseMatrix64F b)
   {
      if (a.getNumRows() != b.getNumCols() || a.getNumCols() != b.getNumRows())
         throw new IllegalArgumentException("Incompatible matrix dimensions");

      int index = 0;
      for (int i = 0; i < b.numRows; i++)
      {
         int index2 = i;

         int end = index + b.numCols;
         while (index < end)
         {
            b.data[index++] = alpha * a.data[index2];
            index2 += a.numCols;
         }
      }
   }

   /**
    * <p>
    * Scales the elements of {@param column} of {@param matrix} by the value {@param alpha}.
    * </p>
    *
    * @param alpha value to scale by
    * @param column column to scale
    * @param matrix matrix modify
    */
   public static void scaleColumn(double alpha, int column, DenseMatrix64F matrix)
   {
      if (column < 0 || column >= matrix.getNumCols())
         throw new IllegalArgumentException("Specified column index is out of bounds: " + column + ", number of columns in matrix: " + matrix.getNumCols());

      for (int row = 0; row < matrix.getNumRows(); row++)
         matrix.unsafe_set(row, column, alpha * matrix.unsafe_get(row, column));
   }

   /**
    * <p>
    * Scales the elements of {@param row} of {@param matrix} by the value {@param alpha}.
    * </p>
    *
    * @param alpha value to scale by
    * @param row row to scale
    * @param matrix matrix modify
    */
   public static void scaleRow(double alpha, int row, DenseMatrix64F matrix)
   {
      if (row < 0 || row >= matrix.getNumRows())
         throw new IllegalArgumentException("Specified row index is out of bounds: " + row + ", number of rows in matrix: " + matrix.getNumRows());

      for (int column = 0; column < matrix.getNumCols(); column++)
         matrix.unsafe_set(row, column, alpha * matrix.unsafe_get(row, column));
   }

   /**
    * <p>
    * Sets the elements of {@param valuesToSet} to the elements of {@param rowIndex} row of {@param matrix}.
    * </p>
    *
    * @param valuesToSet row vector to add
    * @param rowIndex row to add to
    * @param matrix matrix modify
    */
   public static void setRow(DenseMatrix64F valuesToSet, int rowIndex, DenseMatrix64F matrix)
   {
      setRow(0, valuesToSet, rowIndex, matrix);
   }

   /**
    * <p>
    * Multiplies the elements of {@param valuesToAdd} by {@param alpha} and sets them to the elements of {@param rowIndex} row of {@param matrix}.
    * </p>
    *
    * @param alpha row value multiplier
    * @param valuesToSet row vector to add
    * @param rowIndex row to add to
    * @param matrix matrix modify
    */
   public static void setRow(double alpha, DenseMatrix64F valuesToSet, int rowIndex, DenseMatrix64F matrix)
   {
      setRow(0, alpha, valuesToSet, rowIndex, matrix);
   }

   /**
    * <p>
    * Sets the elements of {@param valuesToSet} to the elements of {@param rowIndex} row of {@param matrix}.
    * </p>
    *
    * @param originRowIndex row index to set
    * @param valuesToSet row vector to add
    * @param destRowIndex row to set to
    * @param matrix matrix modify
    */
   public static void setRow(int originRowIndex, DenseMatrix64F valuesToSet, int destRowIndex, DenseMatrix64F matrix)
   {
      if (destRowIndex < 0 || destRowIndex >= matrix.getNumRows())
         throw new IllegalArgumentException("Specified row index is out of bounds: " + destRowIndex + ", number of rows in matrix: " + matrix.getNumRows());

      if (originRowIndex < 0 || originRowIndex >= valuesToSet.getNumRows())
         throw new IllegalArgumentException("Specified row index is out of bounds: " + originRowIndex + ", number of rows in matrix: " + valuesToSet.getNumRows());

      if (valuesToSet.getNumCols() != matrix.getNumCols())
         throw new IllegalArgumentException("Trying to add a row that is the improper length");


      for (int column = 0; column < matrix.getNumCols(); column++)
         matrix.unsafe_set(destRowIndex, column, valuesToSet.unsafe_get(originRowIndex, column));
   }

   /**
    * <p>
    * Scales the elements of the {@param originRowIndex} of {@param valuesToSet} and sets them to the elements of {@param rowIndex} row of {@param matrix}.
    * </p>
    *
    * @param originRowIndex row index to set
    * @param valuesToSet row vector to set
    * @param destRowIndex row index to set to
    * @param matrix matrix modify
    */
   public static void setRow(int originRowIndex, double alpha, DenseMatrix64F valuesToSet, int destRowIndex, DenseMatrix64F matrix)
   {
      if (destRowIndex < 0 || destRowIndex >= matrix.getNumRows())
         throw new IllegalArgumentException("Specified row index is out of bounds: " + destRowIndex + ", number of rows in matrix: " + matrix.getNumRows());

      if (originRowIndex < 0 || originRowIndex >= valuesToSet.getNumRows())
         throw new IllegalArgumentException("Specified row index is out of bounds: " + originRowIndex + ", number of rows in matrix: " + valuesToSet.getNumRows());

      if (valuesToSet.getNumCols() != matrix.getNumCols())
         throw new IllegalArgumentException("Trying to add a row that is the improper length");


      for (int column = 0; column < matrix.getNumCols(); column++)
         matrix.unsafe_set(destRowIndex, column, alpha * valuesToSet.unsafe_get(originRowIndex, column));
   }

   /**
    * <p>
    * Sets the elements of rows {@param originRowIndices} of {@param valuesToAdd} to the elements of rows {@param destRowIndices} of {@param matrix}.
    * </p>
    *
    * @param originRowIndices indices of the rows to add
    * @param valuesToSet matrix to pull row vectors from
    * @param destRowIndices indices of the row to add to
    * @param matrix matrix modify
    */
   public static void setRows(int[] originRowIndices, DenseMatrix64F valuesToSet, int[] destRowIndices, DenseMatrix64F matrix)
   {
      if (originRowIndices.length != destRowIndices.length)
         throw new IllegalArgumentException("Specified indices are not of equivalent length.");

      for (int i = 0; i < originRowIndices.length; i++)
      {
         setRow(originRowIndices[i], valuesToSet, destRowIndices[i], matrix);
      }
   }

   /**
    * <p>
    * Adds the elements of {@param valuesToAdd} to the elements of {@param rowIndex} row of {@param matrix}.
    * </p>
    *
    * @param valuesToAdd row vector to add
    * @param rowIndex row to add to
    * @param matrix matrix modify
    */
   public static void addRow(DenseMatrix64F valuesToAdd, int rowIndex, DenseMatrix64F matrix)
   {
      addRow(0, valuesToAdd, rowIndex, matrix);
   }

   /**
    * <p>
    * Multiplies the elements of {@param valuesToAdd} by {@param alpha} and adds to the elements of {@param rowIndex} row of {@param matrix}.
    * </p>
    *
    * @param alpha scalar multiplier of row being added
    * @param valuesToAdd row vector to add
    * @param rowIndex row index of vector destination
    * @param matrix matrix modify
    */
   public static void addRow(double alpha, DenseMatrix64F valuesToAdd, int rowIndex, DenseMatrix64F matrix)
   {
      addRow(0, alpha, valuesToAdd, rowIndex, matrix);
   }

   /**
    * <p>
    * Adds the elements of row {@param originRowIndex} of {@param valuesToAdd} to the elements of {@param rowIndex} row of {@param matrix}.
    * </p>
    *
    * @param originRowIndex row index of vector to add
    * @param valuesToAdd row vector to add
    * @param destRowIndex row index of vector destination
    * @param matrix matrix modify
    */
   public static void addRow(int originRowIndex, DenseMatrix64F valuesToAdd, int destRowIndex, DenseMatrix64F matrix)
   {
      if (destRowIndex < 0 || destRowIndex >= matrix.getNumRows())
         throw new IllegalArgumentException("Specified row index is out of bounds: " + destRowIndex + ", number of rows in matrix: " + matrix.getNumRows());

      if (originRowIndex < 0 || originRowIndex >= valuesToAdd.getNumRows())
         throw new IllegalArgumentException("Specified row index is out of bounds: " + originRowIndex + ", number of rows in matrix: " + valuesToAdd.getNumRows());

      if (valuesToAdd.getNumCols() != matrix.getNumCols())
         throw new IllegalArgumentException("Trying to add a row that is the improper length");


      for (int column = 0; column < matrix.getNumCols(); column++)
         matrix.unsafe_set(destRowIndex, column, valuesToAdd.unsafe_get(originRowIndex, column) + matrix.unsafe_get(destRowIndex, column));
   }

   /**
    * <p>
    * Scales the elements of row {@param originRowIndex} of {@param valuesToAdd} by {@param alpha} and adds them to the elements of {@param rowIndex} row of {@param matrix}.
    * </p>
    *
    * @param originRowIndex row index of vector to add
    * @param alpha scalar multiplier of row
    * @param valuesToAdd row vector to add
    * @param destRowIndex row index of vector destination
    * @param matrix matrix modify
    */
   public static void addRow(int originRowIndex, double alpha, DenseMatrix64F valuesToAdd, int destRowIndex, DenseMatrix64F matrix)
   {
      if (destRowIndex < 0 || destRowIndex >= matrix.getNumRows())
         throw new IllegalArgumentException("Specified row index is out of bounds: " + destRowIndex + ", number of rows in matrix: " + matrix.getNumRows());

      if (originRowIndex < 0 || originRowIndex >= valuesToAdd.getNumRows())
         throw new IllegalArgumentException("Specified row index is out of bounds: " + originRowIndex + ", number of rows in matrix: " + valuesToAdd.getNumRows());

      if (valuesToAdd.getNumCols() != matrix.getNumCols())
         throw new IllegalArgumentException("Trying to add a row that is the improper length");


      for (int column = 0; column < matrix.getNumCols(); column++)
         matrix.unsafe_set(destRowIndex, column, alpha * valuesToAdd.unsafe_get(originRowIndex, column) + matrix.unsafe_get(destRowIndex, column));
   }


   /**
    * <p>
    * Adds the elements of rows {@param originRowIndices} of {@param valuesToAdd} to the elements of rows {@param destRowIndices} of {@param matrix}.
    * </p>
    *
    * @param originRowIndices indices of the rows to add
    * @param valuesToAdd matrix to pull row vectors from
    * @param destRowIndices indices of the row to add to
    * @param matrix matrix modify
    */
   public static void addRows(int[] originRowIndices, DenseMatrix64F valuesToAdd, int[] destRowIndices, DenseMatrix64F matrix)
   {
      if (originRowIndices.length != destRowIndices.length)
         throw new IllegalArgumentException("Specified indices are not of equivalent length.");

      for (int i = 0; i < originRowIndices.length; i++)
      {
         addRow(originRowIndices[i], valuesToAdd, destRowIndices[i], matrix);
      }
   }


   /**
    * <p>
    * Zeros the elements of {@param column} of {@param matrix}.
    * </p>
    *
    * @param column column to scale
    * @param matrix matrix modify
    */
   public static void zeroColumn(int column, DenseMatrix64F matrix)
   {
      scaleColumn(0.0, column, matrix);
   }

   /**
    * <p>
    * Zeros the elements of {@param row} of {@param matrix}.
    * </p>
    *
    * @param row row to scale
    * @param matrix matrix modify
    */
   public static void zeroRow(int row, DenseMatrix64F matrix)
   {
      scaleRow(0.0, row, matrix);
   }

   public static void printJavaForConstruction(String name, DenseMatrix64F matrix)
   {
      StringBuffer stringBuffer = new StringBuffer();
      printJavaForConstruction(stringBuffer, name, matrix);
      System.out.println(stringBuffer);
   }

   public static void printJavaForConstruction(StringBuffer stringBuffer, String name, DenseMatrix64F matrix)
   {
      int numRows = matrix.getNumRows();
      int numColumns = matrix.getNumCols();

      stringBuffer.append("      double[][] " + name + "Data = new double[][]{");

      for (int i = 0; i < numRows; i++)
      {
         stringBuffer.append("\n            {");

         for (int j = 0; j < numColumns; j++)
         {
            stringBuffer.append(matrix.get(i, j));
            if (j < numColumns - 1)
               stringBuffer.append(", ");

         }
         stringBuffer.append("}");
         if (i < numRows - 1)
            stringBuffer.append(", ");
      }
      stringBuffer.append("};\n");

      stringBuffer.append("      DenseMatrix64F " + name + " = new DenseMatrix64F(" + name + "Data);");
   }

   public static void checkMatrixDimensions(DenseMatrix64F matrixToCheck, int expectedRows, int expectedColumns)
   {
      if ((matrixToCheck.getNumRows() != expectedRows) || (matrixToCheck.getNumCols() != expectedColumns))
      {
         String message = "Matrix dimensions are (" + matrixToCheck.getNumRows() + ", " + matrixToCheck.getNumCols() + "), expected (" + expectedRows + ","
               + expectedColumns + ")";

         throw new RuntimeException(message);
      }
   }

   /**
    * <p>Performs the following operation:<br>
    * <br>
    * c = c + a * b
    * </br>
    * </p>
    * where we are only modifying a block of the c matrix, starting a rowStart, colStart
    * @param a The left matrix in the multiplication operation. Not modified.
    * @param b The right matrix in the multiplication operation. Not modified.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void multAddBlock( RowD1Matrix64F a , RowD1Matrix64F b , RowD1Matrix64F c, int rowStart, int colStart )
   {
      if( a == c || b == c )
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      else if( a.numCols != b.numRows ) {
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      }

      int aIndexStart = 0;

      for( int i = 0; i < a.numRows; i++ ) {
         for( int j = 0; j < b.numCols; j++ ) {
            double total = 0;

            int indexA = aIndexStart;
            int indexB = j;
            int end = indexA + b.numRows;
            while( indexA < end ) {
               total += a.data[indexA++] * b.data[indexB];
               indexB += b.numCols;
            }

            int cIndex = (i+rowStart)*c.numCols + j + colStart;
            c.data[ cIndex] +=  total;
         }
         aIndexStart += a.numCols;
      }
   }

   /**
    * <p>Performs the following operation:<br>
    * <br>
    * c = c + scalar * a * b
    * </br>
    * </p>
    * where we are only modifying a block of the c matrix, starting a rowStart, colStart
    * @param a The left matrix in the multiplication operation. Not modified.
    * @param b The right matrix in the multiplication operation. Not modified.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void multAddBlock( double scalar, RowD1Matrix64F a , RowD1Matrix64F b , RowD1Matrix64F c, int rowStart, int colStart )
   {
      if( a == c || b == c )
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      else if( a.numCols != b.numRows ) {
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      }

      int aIndexStart = 0;

      for( int i = 0; i < a.numRows; i++ ) {
         for( int j = 0; j < b.numCols; j++ ) {
            double total = 0;

            int indexA = aIndexStart;
            int indexB = j;
            int end = indexA + b.numRows;
            while( indexA < end ) {
               total += a.data[indexA++] * b.data[indexB];
               indexB += b.numCols;
            }

            int cIndex = (i+rowStart)*c.numCols + j + colStart;
            c.data[ cIndex] += scalar * total;
         }
         aIndexStart += a.numCols;
      }
   }

   /**
    * <p>Computes the matrix multiplication inner product:<br>
    * <br>
    * c = c + a * b<sup>T</sup> * b <br>
    * <br>
    * c<sub>ij</sub> = c<sub>ij</sub> + a * &sum;<sub>k=1:n</sub> { b<sub>ki</sub> * b<sub>kj</sub>}
    * </p>
    * <p>
    * Is faster than using a generic matrix multiplication by taking advantage of symmetry.
    * </p>
    * @param a The scalar multiplier of the matrix.
    * @param b The matrix being multiplied. Not modified.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void multAddInner(double a, RowD1Matrix64F b, RowD1Matrix64F c)
   {
      if (b == c)
         throw new IllegalArgumentException("'b' cannot be the same matrix as 'c'");
      else if (b.numCols != c.numRows || b.numCols != c.numCols)
         throw new MatrixDimensionException("The results matrix does not have the desired dimensions");

      for (int i = 0; i < b.numCols; i++)
      {
         int j = i;
         int indexC1 = i * c.numCols + j;
         int indexA = i;
         double sum = 0;
         int end = indexA + b.numRows * b.numCols;
         for (; indexA < end; indexA += b.numCols)
         {
            sum += b.data[indexA] * b.data[indexA];
         }
         c.data[indexC1] += a * sum;
         j++;

         for (; j < b.numCols; j++)
         {
            indexC1 = i * c.numCols + j;
            int indexC2 = j * c.numCols + i;
            indexA = i;
            int indexB = j;
            sum = 0;
            end = indexA + b.numRows * b.numCols;
            for (; indexA < end; indexA += b.numCols, indexB += b.numCols)
            {
               sum += b.data[indexA] * b.data[indexB];
            }
            sum *= a;
            c.data[indexC1] += sum;
            c.data[indexC2] += sum;
         }
      }
   }

   /**
    * <p>Computes the matrix multiplication inner product:<br>
    * <br>
    * c = c + a * b<sup>T</sup> * b <br>
    * <br>
    * c<sub>(cRowStart + i) (cColStart + j)</sub> = c<sub>(cRowStart + i) (cColStart + j)</sub> + a * &sum;<sub>k=1:n</sub> { b<sub>ki</sub> * b<sub>kj</sub> }
    * </p>
    * <p> The block is added to matrix 'c' starting at cStartRow, cStartCol </p>
    * <p>
    * Is faster than using a generic matrix multiplication by taking advantage of symmetry.
    * </p>
    * @param a The scalar multiplier for the inner operation.
    * @param b The matrix being multiplied. Not modified.
    * @param c Where the results of the operation are stored. Modified.
    * @param cRowStart The row index to start writing to in the block 'c'.
    * @param cColStart The col index to start writing to in the block 'c'.
    */
   public static void multAddBlockInner(double a, RowD1Matrix64F b,  RowD1Matrix64F c, int cRowStart, int cColStart)
   {
      if (b == c)
         throw new IllegalArgumentException("'b' cannot be the same matrix as 'c'");
      else if (b.numCols + cRowStart > c.numRows || b.numCols + cColStart > c.numCols)
         throw new MatrixDimensionException("The results matrix does not have the desired dimensions");

      for (int i = 0; i < b.numCols; i++)
      {
         int j = i;
         int indexA = i;
         double sum = 0;
         int end = indexA + b.numRows * b.numCols;
         for (; indexA < end; indexA += b.numCols)
         {
            sum += b.data[indexA] * b.data[indexA];
         }
         int indexC1 = (i + cRowStart) * c.numCols + j + cColStart;
         c.data[indexC1] += a * sum;
         j++;

         for (; j < b.numCols; j++)
         {
            indexA = i;
            int indexB = j;
            sum = 0;
            end = indexA + b.numRows * b.numCols;
            for (; indexA < end; indexA += b.numCols, indexB += b.numCols)
            {
               sum += b.data[indexA] * b.data[indexB];
            }
            indexC1 = (i + cRowStart) * c.numCols + j + cColStart;
            int indexC2 = (j + cRowStart) * c.numCols + i + cColStart;
            sum *= a;
            c.data[indexC1] += sum;
            c.data[indexC2] += sum;
         }
      }
   }

   /**
    * <p>Performs the following operation:<br>
    * <br>
    * c = c + a<sup>T</sup> * b
    * </br>
    * </p>
    * where we are only modifying a block of the c matrix, starting a rowStart, colStart
    * @param a The left matrix in the multiplication operation. Not modified.
    * @param b The right matrix in the multiplication operation. Not modified.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void multAddBlockTransA( RowD1Matrix64F a , RowD1Matrix64F b , RowD1Matrix64F c, int rowStart, int colStart)
   {
      if( a == c || b == c )
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      else if( a.numRows != b.numRows ) {
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      }

      for( int i = 0; i < a.numCols; i++ ) {
         for( int j = 0; j < b.numCols; j++ ) {
            int indexA = i;
            int indexB = j;
            int end = indexB + b.numRows*b.numCols;

            double total = 0;

            // loop for k
            for(; indexB < end; indexB += b.numCols ) {
               total += a.data[indexA] * b.data[indexB];
               indexA += a.numCols;
            }

            int cIndex = (i+rowStart)*c.numCols + j + colStart;
            c.data[cIndex] += total;
         }
      }
   }

   /**
    * <p>Performs the following operation:<br>
    * <br>
    * c = c + scalar * a<sup>T</sup> * b
    * </br>
    * </p>
    * where we are only modifying a block of the c matrix, starting a rowStart, colStart
    * @param a The left matrix in the multiplication operation. Not modified.
    * @param b The right matrix in the multiplication operation. Not modified.
    * @param c Where the results of the operation are stored. Modified.
    */
   public static void multAddBlockTransA(double scalar,  RowD1Matrix64F a , RowD1Matrix64F b , RowD1Matrix64F c, int rowStart, int colStart)
   {
      if( a == c || b == c )
         throw new IllegalArgumentException("Neither 'a' or 'b' can be the same matrix as 'c'");
      else if( a.numRows != b.numRows ) {
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      }

      for( int i = 0; i < a.numCols; i++ ) {
         for( int j = 0; j < b.numCols; j++ ) {
            int indexA = i;
            int indexB = j;
            int end = indexB + b.numRows*b.numCols;

            double total = 0;

            // loop for k
            for(; indexB < end; indexB += b.numCols ) {
               total += a.data[indexA] * b.data[indexB];
               indexA += a.numCols;
            }

            int cIndex = (i+rowStart)*c.numCols + j + colStart;
            c.data[cIndex] += scalar * total;
         }
      }
   }

   /**
    * Checks if two matrices are equal.
    * <p>
    * Checks that the dimensions are equal and that all elements are equal.
    *
    * @param a first matrix
    * @param b second matrix
    */
   public static boolean equals(DenseMatrix64F a, DenseMatrix64F b)
   {
      if (a.numRows != b.numRows)
         return false;
      if (a.numCols != b.numCols)
         return false;
      for (int i = 0; i < a.getNumElements(); i++)
      {
         if (Double.compare(a.get(i), b.get(i)) != 0)
            return false;
      }
      return true;
   }
}
