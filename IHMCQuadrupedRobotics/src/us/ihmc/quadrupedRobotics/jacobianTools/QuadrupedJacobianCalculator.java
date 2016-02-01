package us.ihmc.quadrupedRobotics.jacobianTools;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.PointJacobian;
import us.ihmc.robotics.screwTheory.Twist;

public class QuadrupedJacobianCalculator
{
   private final QuadrantDependentList<OneDoFJoint[]> oneDoFJoints = new QuadrantDependentList<>();
   private final QuadrantDependentList<GeometricJacobian> bodyToLegJacobians = new QuadrantDependentList<>();
   private final QuadrantDependentList<PointJacobian> bodyToFootJacobians = new QuadrantDependentList<>();
   private final QuadrupedReferenceFrames referenceFrames;
   private final FramePoint footFramePoint = new FramePoint();
   private final FrameVector footLinearVelocity = new FrameVector();
   private final FrameVector footAngularVelocity = new FrameVector();
   
   private final DenseMatrix64F linearVelocityVector = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F jointVelocitiesVector;
   
   public QuadrupedJacobianCalculator(SDFFullRobotModel fullRobotModel, QuadrupedRobotParameters robotParameters)
   {
      QuadrupedJointNameMap jointMap = robotParameters.getJointMap();
      QuadrupedPhysicalProperties physicalProperties = robotParameters.getPhysicalProperties();
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, jointMap, physicalProperties);
      
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         OneDoFJoint knee = fullRobotModel.getOneDoFJointByName(jointMap.getJointBeforeFootName(quadrant));
         ArrayList<OneDoFJoint> oneDoFJointsToPack = new ArrayList<>();
         fullRobotModel.getOneDoFJointsFromRootToHere(knee, oneDoFJointsToPack);
         OneDoFJoint[] oneDofJoints = new OneDoFJoint[oneDoFJointsToPack.size()];
         oneDoFJointsToPack.toArray(oneDofJoints);
         oneDoFJoints.set(quadrant, oneDofJoints);
         GeometricJacobian geometricJacobian = new GeometricJacobian(oneDofJoints, referenceFrames.getBodyFrame());
         bodyToLegJacobians.set(quadrant, geometricJacobian);
         
         PointJacobian pointJacobian = new PointJacobian();
         bodyToFootJacobians.set(quadrant, pointJacobian);
      }
      
      DenseMatrix64F jacobianMatrix = bodyToLegJacobians.get(RobotQuadrant.FRONT_LEFT).getJacobianMatrix();
      jointVelocitiesVector = new DenseMatrix64F(jacobianMatrix.getNumCols(), 1);
   }
   
   public void computeJacobian()
   {
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         GeometricJacobian bodyToShinJacobian = bodyToLegJacobians.get(quadrant);
         bodyToShinJacobian.compute();
         
         ReferenceFrame footFrame = referenceFrames.getFootFrame(quadrant);
         footFramePoint.setToZero(footFrame);
         footFramePoint.changeFrame(bodyToShinJacobian.getBaseFrame());
         
         PointJacobian bodyToFoot = bodyToFootJacobians.get(quadrant);
         bodyToFoot.set(bodyToShinJacobian, footFramePoint);
      }
   }
   
   /**
    *   DenseMatrix64F pointVelocityFromJacobianMatrix = new DenseMatrix64F(3, 1);
      CommonOps.mult(pointJacobian.getJacobianMatrix(), jointVelocities, pointVelocityFromJacobianMatrix);
      FrameVector pointVelocityFromJacobian = new FrameVector(pointJacobian.getFrame());
      MatrixTools.denseMatrixToVector3d(pointVelocityFromJacobianMatrix, pointVelocityFromJacobian.getVector(), 0, 0);
    */
   
   /**
    * Returns the twist of the foot frame with respect to the Body frame, expressed in the Foot frame,
    * corresponding to the given joint velocities. Uses a point jacobian to calculate twist, angular part will be zero
    */
   public void getLinearVelocityOfFootWithRespectToBodyInFootFrame(RobotQuadrant quadrant, double[] jointVelocities, Twist twistToPack)
   {
      if(jointVelocitiesVector.getNumRows() != jointVelocities.length)
      {
         throw new IllegalArgumentException("Jacobian matrix and number of joint velocities doesn't match");
      }
      
      GeometricJacobian bodyToShinJacobian = bodyToLegJacobians.get(quadrant);
      PointJacobian pointJacobian = bodyToFootJacobians.get(quadrant);
      ReferenceFrame footFrame = referenceFrames.getFootFrame(quadrant);
      
      for(int i = 0; i < jointVelocities.length; i++)
      {
         jointVelocitiesVector.set(i, 0, jointVelocities[i]);
      }
      
      pointJacobian.compute();
      
      footAngularVelocity.changeFrame(bodyToShinJacobian.getBaseFrame());
      footLinearVelocity.changeFrame(bodyToShinJacobian.getBaseFrame());

      CommonOps.mult(pointJacobian.getJacobianMatrix(), jointVelocitiesVector, linearVelocityVector);
      footLinearVelocity.setX(linearVelocityVector.get(0, 0));
      footLinearVelocity.setY(linearVelocityVector.get(1, 0));
      footLinearVelocity.setZ(linearVelocityVector.get(2, 0));
      
      twistToPack.set(footFrame, bodyToShinJacobian.getBaseFrame(), bodyToShinJacobian.getJacobianFrame(), footLinearVelocity, footAngularVelocity);
   }

   public ReferenceFrame getJacobianFrame(RobotQuadrant robotQuadrant)
   {
      GeometricJacobian bodyToShinJacobian = bodyToLegJacobians.get(robotQuadrant);
      return bodyToShinJacobian.getJacobianFrame();
   }
   
   public ReferenceFrame getBaseFrame(RobotQuadrant robotQuadrant)
   {
      GeometricJacobian bodyToShinJacobian = bodyToLegJacobians.get(robotQuadrant);
      return bodyToShinJacobian.getBaseFrame();
   }

   public OneDoFJoint[] getJointsInOrder(RobotQuadrant robotQuadrant)
   {
      return oneDoFJoints.get(robotQuadrant);
   }
}
