package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

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
   private final Vector3D tempJacobianColumn = new Vector3D();
   private final Vector3D translation = new Vector3D();
   private final Vector3D tempVector = new Vector3D();

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
         tempVector.set(angularPartStartRow, i, geometricJacobianMatrix);
         tempJacobianColumn.cross(tempVector, translation);
         tempVector.set(linearPartStartRow, i, geometricJacobianMatrix);

         // linear part
         tempJacobianColumn.add(tempVector);
         tempJacobianColumn.get(angularPartStartRow, i, jacobianMatrix);
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
