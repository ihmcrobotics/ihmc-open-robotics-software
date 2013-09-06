package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.GeometricJacobian;

public class DesiredPointAccelerationCommand
{
   private final GeometricJacobian rootToEndEffectorJacobian;
   private final FramePoint contactPoint;
   private final FrameVector desiredAcceleration;
   private final DenseMatrix64F selectionMatrix;

   public DesiredPointAccelerationCommand(GeometricJacobian rootToEndEffectorJacobian, FramePoint contactPoint, FrameVector desiredAcceleration, DenseMatrix64F selectionMatrix)
   {
      this.rootToEndEffectorJacobian = rootToEndEffectorJacobian;
      this.contactPoint = contactPoint;
      this.desiredAcceleration = desiredAcceleration;
      this.selectionMatrix = selectionMatrix;
   }

   public DesiredPointAccelerationCommand(GeometricJacobian jacobian, FramePoint bodyFixedPoint, FrameVector desiredAccelerationWithRespectToBase)
   {
      this(jacobian, bodyFixedPoint, desiredAccelerationWithRespectToBase, null);
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

   public String toString()
   {
      return "DesiredPointAccelerationCommand: rootToEndEffectorJacobian = " + rootToEndEffectorJacobian;
   }

}
