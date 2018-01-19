package us.ihmc.robotics.geometry;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;

/**
 * One of the main goals of this class is to check, at runtime, that operations on matrices occur within the same Frame.
 * This method checks for one matrix argument.
 *
 */
public class FrameMatrix3D extends FrameGeometryObject<FrameMatrix3D, Matrix3D>
{
   private final Matrix3D matrix;

   public FrameMatrix3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameMatrix3D(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Matrix3D());
      this.matrix = this.getGeometryObject();
   }

   public FrameMatrix3D(FrameMatrix3D frameMatrix3D)
   {
      this();
      setIncludingFrame(frameMatrix3D);
   }

   public FrameMatrix3D(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix)
   {
      this();
      setIncludingFrame(referenceFrame, matrix);
   }

   public void set(Matrix3DReadOnly matrix)
   {
      this.matrix.set(matrix);
   }

   public void setM00(double m00)
   {
      this.matrix.setM00(m00);
   }

   public void setM01(double m01)
   {
      this.matrix.setM01(m01);
   }

   public void setM02(double m02)
   {
      this.matrix.setM02(m02);
   }

   public void setM10(double m10)
   {
      this.matrix.setM10(m10);
   }

   public void setM11(double m11)
   {
      this.matrix.setM11(m11);
   }

   public void setM12(double m12)
   {
      this.matrix.setM12(m12);
   }

   public void setM20(double m20)
   {
      this.matrix.setM20(m20);
   }

   public void setM21(double m21)
   {
      this.matrix.setM21(m21);
   }

   public void setM22(double m22)
   {
      this.matrix.setM22(m22);
   }

   public void setElement(int row, int column, double value)
   {
      this.matrix.setElement(row, column, value);
   }

   public void setRow(int row, double x, double y, double z)
   {
      this.matrix.setRow(row, x, y, z);
   }

   public void setColumn(int column, double x, double y, double z)
   {
      this.matrix.setColumn(column, x, y, z);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix)
   {
      this.referenceFrame = referenceFrame;
      this.matrix.set(matrix);
   }

   public void setToIdentity()
   {
      matrix.setIdentity();
   }

   public void setToIdentity(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      setToIdentity();
   }

   public double getElement(int row, int column)
   {
      return matrix.getElement(row, column);
   }

   public void getMatrix(Matrix3DBasics matrixToPack)
   {
      matrixToPack.set(matrix);
   }

   public void getDenseMatrix(DenseMatrix64F matrixToPack)
   {
      getDenseMatrix(matrixToPack, 0, 0);
   }

   public void getDenseMatrix(DenseMatrix64F matrixToPack, int startRow, int startColumn)
   {
      matrix.get(startRow, startColumn, matrixToPack);
   }

   /**
     * Multiply this frameMatrix3D by the FrameTuple frameTupleToPack and place the result
     * back into the FrameTuple (frameTupleToPack = this * frameTupleToPack).
     * @param frameTupleToPack the frameTuple to be multiplied by this frameMatrix3D and then replaced
     * @throws ReferenceFrameMismatchException
     */
   public void transform(FrameTuple3DBasics frameTupleToPack)
   {
      checkReferenceFrameMatch(frameTupleToPack);
      matrix.transform(frameTupleToPack);
   }

   /**
     * Multiply this frameMatrix3D by the FrameTuple frameTupleOriginal and and place the result
     * into the FrameTuple frameTupleToPack (frameTupleToPack = this * frameTupleOriginal).
     * @param frameTupleOriginal the FrameTuple to be multiplied by this frameMatrix3D
     * @param frameTupleToPack the FrameTuple into which the product is placed
     * @throws ReferenceFrameMismatchException
     */
   public void transform(FrameTuple3DReadOnly frameTupleOriginal, FrameTuple3DBasics frameTupleToPack)
   {
      checkReferenceFrameMatch(frameTupleOriginal);
      frameTupleToPack.setIncludingFrame(frameTupleOriginal);
      matrix.transform(frameTupleToPack);
   }

   public void setMainDiagonal(double x, double y, double z)
   {
      setElement(0, 0, x);
      setElement(1, 1, y);
      setElement(2, 2, z);
   }
}
