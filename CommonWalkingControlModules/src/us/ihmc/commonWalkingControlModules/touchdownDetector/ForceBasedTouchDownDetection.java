package us.ihmc.commonWalkingControlModules.touchdownDetector;

import org.ejml.alg.dense.misc.UnrolledInverseFromMinor;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.models.FullQuadrupedRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ForceBasedTouchDownDetection implements TouchdownDetector
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry ;
   private final GeometricJacobian footJacobian;
   private final DenseMatrix64F linearPartOfJacobian = new DenseMatrix64F(3, 3);   
   private final DenseMatrix64F linearJacobianInverse = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(6);   
   private final DenseMatrix64F jointTorques = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F footLinearForce = new DenseMatrix64F(3, 1);
   private final OneDoFJoint[] legjoints;
   
   private final BooleanYoVariable isInContact;
   private final DoubleYoVariable zForceThreshold;
   private final DoubleYoVariable measuredZForce;
   
   
   
   public ForceBasedTouchDownDetection(FullQuadrupedRobotModel robotModel, RobotQuadrant robotQuadrant, YoVariableRegistry parentRegistry)
   {
      String prefix = robotQuadrant.getCamelCaseName() + name;
      registry = new YoVariableRegistry(prefix);
      
      isInContact = new BooleanYoVariable(prefix + "isInContact", registry);
      zForceThreshold = new DoubleYoVariable(prefix + "zForceThreshold", registry);
      measuredZForce = new DoubleYoVariable(prefix + "measuredZForce", registry);
      
      zForceThreshold.set(10.0);
      
      RigidBody elevator = robotModel.getElevator();
      RigidBody foot = robotModel.getFoot(robotQuadrant);
      footJacobian = new GeometricJacobian(elevator, foot, foot.getBodyFixedFrame());
      legjoints = (OneDoFJoint[]) footJacobian.getJointsInOrder();
      
      //remove angular part
      MatrixTools.removeRow(selectionMatrix, 0);
      MatrixTools.removeRow(selectionMatrix, 0);
      MatrixTools.removeRow(selectionMatrix, 0);
      
      parentRegistry.addChild(registry);
   }

   @Override
   public boolean hasTouchedDown()
   {
      return isInContact.getBooleanValue();
   }

   @Override
   public void update()
   {
      for(int i = 0; i < legjoints.length; i++)
      {
         jointTorques.set(i, 0, legjoints[i].getTauMeasured());
      }
      
      
      footJacobian.compute();
      DenseMatrix64F jacobianMatrix = footJacobian.getJacobianMatrix();
      CommonOps.mult(jacobianMatrix, selectionMatrix , linearJacobianInverse);
      UnrolledInverseFromMinor.inv3(linearPartOfJacobian, linearJacobianInverse, 1.0);
      CommonOps.mult(linearJacobianInverse, jointTorques, footLinearForce);
      measuredZForce.set(footLinearForce.get(0, 3));
      isInContact.set(measuredZForce.getDoubleValue() > zForceThreshold.getDoubleValue());
   }
}
