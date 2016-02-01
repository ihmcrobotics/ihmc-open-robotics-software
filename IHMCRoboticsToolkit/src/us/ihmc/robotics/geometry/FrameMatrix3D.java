package us.ihmc.robotics.geometry;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Matrix3d;

/**
 * One of the main goals of this class is to check, at runtime, that operations on matrices occur within the same Frame.
 * This method checks for one matrix argument.
 *
 */
public class FrameMatrix3D extends ReferenceFrameHolder
{
   private ReferenceFrame referenceFrame;
   private final Matrix3d matrix = new Matrix3d();

   public FrameMatrix3D()
   {
      referenceFrame = ReferenceFrame.getWorldFrame();
   }

   public FrameMatrix3D(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public FrameMatrix3D(FrameMatrix3D frameMatrix3D)
   {
      setIncludingFrame(frameMatrix3D);
   }

   public FrameMatrix3D(ReferenceFrame referenceFrame, Matrix3d matrix)
   {
      setIncludingFrame(referenceFrame, matrix);
   }

   public void set(FrameMatrix3D frameMatrix3D)
   {
      checkReferenceFrameMatch(frameMatrix3D);
      matrix.set(frameMatrix3D.matrix);
   }

   public void set(Matrix3d matrix)
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

   public void setIncludingFrame(ReferenceFrame referenceFrame, Matrix3d matrix)
   {
      this.referenceFrame = referenceFrame;
      this.matrix.set(matrix);
   }

   public void setToZero()
   {
      matrix.setZero();
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

   public void getMatrix(Matrix3d matrixToPack)
   {
      matrixToPack.set(matrix);
   }

   public void getDenseMatrix(DenseMatrix64F matrixToPack)
   {
      getDenseMatrix(matrixToPack, 0, 0);
   }

   public void getDenseMatrix(DenseMatrix64F matrixToPack, int startRow, int startColumn)
   {
      MatrixTools.matrix3DToDenseMatrix(matrix, matrixToPack, startRow, startColumn);
   }

   /**
     * Multiply this frameMatrix3D by the FrameTuple frameTupleToPack and place the result
     * back into the FrameTuple (frameTupleToPack = this * frameTupleToPack).
     * @param frameTupleToPack the frameTuple to be multiplied by this frameMatrix3D and then replaced
     * @throws ReferenceFrameMismatchException
     */
   public void transform(FrameTuple<?> frameTupleToPack)
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
   public void transform(FrameTuple<?> frameTupleOriginal, FrameTuple<?> frameTupleToPack)
   {
      checkReferenceFrameMatch(frameTupleOriginal);
      frameTupleToPack.setIncludingFrame(frameTupleOriginal);
      matrix.transform(frameTupleToPack.tuple);
   }

   private final Matrix3d temporaryMatrix = new Matrix3d();

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // this is in the correct frame already
      if (desiredFrame == referenceFrame)
      {
         return;
      }

      referenceFrame.verifySameRoots(desiredFrame);
      RigidBodyTransform referenceTf, desiredTf;

      if ((referenceTf = referenceFrame.getTransformToRoot()) != null)
      {
         referenceTf.get(temporaryMatrix);
         matrix.mul(temporaryMatrix, matrix);
         matrix.mulTransposeRight(matrix, temporaryMatrix);
      }

      if ((desiredTf = desiredFrame.getInverseTransformToRoot()) != null)
      {
         desiredTf.get(temporaryMatrix);
         matrix.mul(temporaryMatrix, matrix);
         matrix.mulTransposeRight(matrix, temporaryMatrix);
      }

      referenceFrame = desiredFrame;
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
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public String toString()
   {
      return "Expressed in frame: " + referenceFrame + "\n" + matrix.toString();
   }
}
