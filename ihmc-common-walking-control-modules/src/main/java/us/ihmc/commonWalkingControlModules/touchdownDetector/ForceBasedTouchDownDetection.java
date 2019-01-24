package us.ihmc.commonWalkingControlModules.touchdownDetector;

import java.util.List;

import org.ejml.alg.dense.misc.UnrolledInverseFromMinor;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

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
   private final List<OneDoFJointBasics> legOneDoFJoints;
   
   private final YoBoolean isInContact;
   private final DoubleProvider zForceThreshold;
   private final YoDouble measuredZForce;
   private final FrameVector3D footForce = new FrameVector3D();
   
   public ForceBasedTouchDownDetection(FullQuadrupedRobotModel robotModel, RobotQuadrant robotQuadrant, ReferenceFrame soleFrame, YoVariableRegistry parentRegistry)
   {
      String prefix = robotQuadrant.getCamelCaseName() + name;
      registry = new YoVariableRegistry(prefix);
      
      isInContact = new YoBoolean(prefix + "isInContact", registry);
      zForceThreshold = new DoubleParameter(prefix + "zForceThreshold", registry, 40.0);
      measuredZForce = new YoDouble(prefix + "measuredZForce", registry);
      
      RigidBodyBasics body = robotModel.getRootBody();
      RigidBodyBasics foot = robotModel.getFoot(robotQuadrant);
      footJacobian = new GeometricJacobian(body, foot, soleFrame);
      
      legOneDoFJoints = robotModel.getLegJointsList(robotQuadrant);
      
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
      for(int i = 0; i < legOneDoFJoints.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = legOneDoFJoints.get(i);
         jointTorques.set(i, 0, oneDoFJoint.getTau());
      }
      
      footJacobian.compute();
      DenseMatrix64F jacobianMatrix = footJacobian.getJacobianMatrix();
      CommonOps.mult(selectionMatrix, jacobianMatrix, linearPartOfJacobian);
      UnrolledInverseFromMinor.inv3(linearPartOfJacobian, linearJacobianInverse, 1.0);
      
      CommonOps.multTransA(linearJacobianInverse, jointTorques, footLinearForce);
      
      footForce.setToZero(footJacobian.getJacobianFrame());
      footForce.set(footLinearForce);
      footForce.changeFrame(ReferenceFrame.getWorldFrame());
      measuredZForce.set(footForce.getZ() * -1.0);
      isInContact.set(measuredZForce.getDoubleValue() > zForceThreshold.getValue());
   }

   public void reset()
   {
      measuredZForce.set(0.0);
      isInContact.set(false);
   }
}
