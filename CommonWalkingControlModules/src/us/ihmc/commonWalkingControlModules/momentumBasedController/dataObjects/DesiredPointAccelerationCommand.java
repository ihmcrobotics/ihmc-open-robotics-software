package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;

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
   
   public DesiredPointAccelerationCommand(DesiredPointAccelerationCommand desiredPointAccelerationCommand)
   {
      this.rootToEndEffectorJacobian = desiredPointAccelerationCommand.rootToEndEffectorJacobian;
      this.contactPoint = desiredPointAccelerationCommand.contactPoint;
      this.desiredAcceleration = desiredPointAccelerationCommand.desiredAcceleration;
      this.selectionMatrix = desiredPointAccelerationCommand.selectionMatrix;
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
   
   public void computeAchievedPointAcceleration(DenseMatrix64F achievedSpatialAcceleration)
   {
      DenseMatrix64F jacobianMatrix = rootToEndEffectorJacobian.getJacobianMatrix();
      InverseDynamicsJoint[] jointsInOrder = rootToEndEffectorJacobian.getJointsInOrder();
      
      DenseMatrix64F jointAccelerations = new DenseMatrix64F(rootToEndEffectorJacobian.getNumberOfColumns(), 1);
      
      int index = 0;
      for (int i=0; i<jointsInOrder.length; i++)
      {
         InverseDynamicsJoint joint = jointsInOrder[i];
         
         jointsInOrder[i].getDesiredAccelerationMatrix(jointAccelerations, index);
         index = index + joint.getDegreesOfFreedom();
      }
      
      CommonOps.mult(jacobianMatrix, jointAccelerations, achievedSpatialAcceleration);
   }
}

