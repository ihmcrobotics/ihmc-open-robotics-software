package us.ihmc.robotics.geometry;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * One of the main goals of this class is to check, at runtime, that operations on matrices occur within the same Frame.
 * This method checks for one matrix argument.
 *
 */
public class FrameMatrix3D extends AbstractFrameObject<FrameMatrix3D, Matrix3D>
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

   public void set(FrameMatrix3D frameMatrix3D)
   {
      checkReferenceFrameMatch(frameMatrix3D);
      matrix.set(frameMatrix3D.matrix);
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

   public void setIncludingFrame(FrameMatrix3D frameMatrix3D)
   {
      referenceFrame = frameMatrix3D.referenceFrame;
      matrix.set(frameMatrix3D.matrix);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix)
   {
      this.referenceFrame = referenceFrame;
      this.matrix.set(matrix);
   }

   public void setToZero()
   {
      matrix.setToZero();
   }

   public void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      setToZero();
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

   public void setToNaN()
   {
      setElement(0, 0, Double.NaN);
      setElement(0, 1, Double.NaN);
      setElement(0, 2, Double.NaN);
      setElement(1, 0, Double.NaN);
      setElement(1, 1, Double.NaN);
      setElement(1, 2, Double.NaN);
      setElement(2, 0, Double.NaN);
      setElement(2, 1, Double.NaN);
      setElement(2, 2, Double.NaN);
   }

   public void setToNaN(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      setToNaN();
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
   public void transform(FrameTuple<?, ?> frameTupleToPack)
   {
      checkReferenceFrameMatch(frameTupleToPack);
      matrix.transform(frameTupleToPack.tuple);
   }

   /**
     * Multiply this frameMatrix3D by the FrameTuple frameTupleOriginal and and place the result
     * into the FrameTuple frameTupleToPack (frameTupleToPack = this * frameTupleOriginal).
     * @param frameTupleOriginal the FrameTuple to be multiplied by this frameMatrix3D
     * @param frameTupleToPack the FrameTuple into which the product is placed
     * @throws ReferenceFrameMismatchException
     */
   public void transform(FrameTuple<?, ?> frameTupleOriginal, FrameTuple<?, ?> frameTupleToPack)
   {
      checkReferenceFrameMatch(frameTupleOriginal);
      frameTupleToPack.setIncludingFrame(frameTupleOriginal);
      matrix.transform(frameTupleToPack.tuple);
   }

   public boolean containsNaN()
   {
      if (Double.isNaN(matrix.getElement(0, 0))) return true;
      if (Double.isNaN(matrix.getElement(0, 1))) return true;
      if (Double.isNaN(matrix.getElement(0, 2))) return true;
      if (Double.isNaN(matrix.getElement(1, 0))) return true;
      if (Double.isNaN(matrix.getElement(1, 1))) return true;
      if (Double.isNaN(matrix.getElement(1, 2))) return true;
      if (Double.isNaN(matrix.getElement(2, 0))) return true;
      if (Double.isNaN(matrix.getElement(2, 1))) return true;
      if (Double.isNaN(matrix.getElement(2, 2))) return true;
      return false;
   }

   @Override
   public String toString()
   {
      return "Expressed in frame: " + referenceFrame + "\n" + matrix.toString();
   }
}
