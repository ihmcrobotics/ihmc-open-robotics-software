package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Vector3d;

/**
 * Jacobian that maps joint velocities to the velocity of a point fixed
 * to the end effector of a GeometricJacobian, with respect to the base
 * of the GeometricJacobian, expressed in base frame.
 *
 * @author twan
 *         Date: 5/18/13
 */
public class PointJacobian
{
   private GeometricJacobian geometricJacobian;
   private final FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame());
   private final DenseMatrix64F jacobianMatrix = new DenseMatrix64F(1, 1);

   // temp stuff
   private final Vector3d tempJacobianColumn = new Vector3d();
   private final Vector3d translation = new Vector3d();
   private final Vector3d tempVector = new Vector3d();

   public void set(GeometricJacobian geometricJacobian, FramePoint point)
   {
      if (geometricJacobian.getBaseFrame() != geometricJacobian.getJacobianFrame())
         throw new RuntimeException("Jacobian must be expressed in base frame");

      this.geometricJacobian = geometricJacobian;
      this.point.setIncludingFrame(point);
      this.point.changeFrame(geometricJacobian.getBaseFrame());
      this.jacobianMatrix.reshape(3, geometricJacobian.getNumberOfColumns());
   }

   /**
    * assumes that the GeometricJacobian.compute() method has already been called
    */
   public void compute()
   {
      point.get(translation);

      int angularPartStartRow = 0;
      int linearPartStartRow = SpatialMotionVector.SIZE / 2;

      for (int i = 0; i < geometricJacobian.getNumberOfColumns(); i++)
      {
         DenseMatrix64F geometricJacobianMatrix = geometricJacobian.getJacobianMatrix();

         // angular part:
         MatrixTools.denseMatrixToVector3d(geometricJacobianMatrix, tempVector, angularPartStartRow, i);
         tempJacobianColumn.cross(tempVector, translation);
         MatrixTools.denseMatrixToVector3d(geometricJacobianMatrix, tempVector, linearPartStartRow, i);

         // linear part
         tempJacobianColumn.add(tempVector);
         MatrixTools.setDenseMatrixFromTuple3d(jacobianMatrix, tempJacobianColumn, angularPartStartRow, i);
      }
   }

   public ReferenceFrame getFrame()
   {
      return geometricJacobian.getBaseFrame();
   }

   public DenseMatrix64F getJacobianMatrix()
   {
      return jacobianMatrix;
   }

   public GeometricJacobian getGeometricJacobian()
   {
      return geometricJacobian;
   }

   public FramePoint getPoint()
   {
      return point;
   }
}
