package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

public class PointAccelerationCommand extends InverseDynamicsCommand<PointAccelerationCommand>
{
   private GeometricJacobian rootToEndEffectorJacobian;
   private final FramePoint contactPoint = new FramePoint();
   private final FrameVector desiredAcceleration = new FrameVector();
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(3);

   public PointAccelerationCommand()
   {
      super(InverseDynamicsCommandType.TASKSPACE_POINT_MOTION);
   }

   public PointAccelerationCommand(GeometricJacobian rootToEndEffectorJacobian, FramePoint contactPoint, FrameVector desiredAcceleration,
         DenseMatrix64F selectionMatrix)
   {
      super(InverseDynamicsCommandType.TASKSPACE_POINT_MOTION);
      set(rootToEndEffectorJacobian, contactPoint, desiredAcceleration, selectionMatrix);
   }

   public PointAccelerationCommand(GeometricJacobian jacobian, FramePoint bodyFixedPoint, FrameVector desiredAccelerationWithRespectToBase)
   {
      this(jacobian, bodyFixedPoint, desiredAccelerationWithRespectToBase, null);
   }

   public PointAccelerationCommand(PointAccelerationCommand pointAccelerationCommand)
   {
      super(InverseDynamicsCommandType.TASKSPACE_POINT_MOTION);
      this.rootToEndEffectorJacobian = pointAccelerationCommand.rootToEndEffectorJacobian;
      this.contactPoint.set(pointAccelerationCommand.contactPoint);
      this.desiredAcceleration.set(pointAccelerationCommand.desiredAcceleration);
      if (selectionMatrix != null)
         this.selectionMatrix.set(pointAccelerationCommand.selectionMatrix);
      else
         setSelectionMatrixToIdentity();
   }

   public void set(GeometricJacobian jacobian, FramePoint bodyFixedPoint, FrameVector desiredAccelerationWithRespectToBase)
   {
      set(jacobian, bodyFixedPoint, desiredAccelerationWithRespectToBase, null);
   }

   @Override
   public void set(PointAccelerationCommand other)
   {
      this.rootToEndEffectorJacobian = other.rootToEndEffectorJacobian;
      this.contactPoint.setIncludingFrame(other.contactPoint);
      this.desiredAcceleration.setIncludingFrame(other.desiredAcceleration);
      if (selectionMatrix != null)
         this.selectionMatrix.set(selectionMatrix);
      else
         setSelectionMatrixToIdentity();
   }

   public void set(GeometricJacobian rootToEndEffectorJacobian, FramePoint contactPoint, FrameVector desiredAcceleration, DenseMatrix64F selectionMatrix)
   {
      this.rootToEndEffectorJacobian = rootToEndEffectorJacobian;
      this.contactPoint.setIncludingFrame(contactPoint);
      this.desiredAcceleration.setIncludingFrame(desiredAcceleration);
      if (selectionMatrix != null)
         this.selectionMatrix.set(selectionMatrix);
      else
         setSelectionMatrixToIdentity();
   }

   private void setSelectionMatrixToIdentity()
   {
      selectionMatrix.reshape(3, 3);
      CommonOps.setIdentity(selectionMatrix);
   }

   public GeometricJacobian getRootToEndEffectorJacobian()
   {
      return rootToEndEffectorJacobian;
   }

   public FramePoint getContactPoint()
   {
      return contactPoint;
   }

   public FrameVector getDesiredAcceleration()
   {
      return desiredAcceleration;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": rootToEndEffectorJacobian = " + rootToEndEffectorJacobian;
   }
}
