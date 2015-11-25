package us.ihmc.quadrupedRobotics.virtualModelController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.PointJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class QuadrupedVirtualModelController
{
   ReferenceFrame comFrame;
   QuadrantDependentList<ReferenceFrame> toeFrame;

   FrameVector desiredComForce;
   FrameVector optimalComForce;
   FrameVector desiredBodyTorque;
   FrameVector optimalBodyTorque;
   QuadrantDependentList<FrameVector> desiredToeForce;
   QuadrantDependentList<FrameVector> optimalToeForce;

   QuadrantDependentList<double[]> minJointTorque;
   QuadrantDependentList<double[]> maxJointTorque;
   QuadrantDependentList<double[]> minJointPosition;
   QuadrantDependentList<double[]> maxJointPosition;
   QuadrantDependentList<double[]> jointPositionStiffness;
   QuadrantDependentList<double[]> jointPositionDamping;

   QuadrantDependentList<OneDoFJoint[]> legJoints;
   QuadrantDependentList<FramePoint> toePosition;
   QuadrantDependentList<GeometricJacobian> footJacobian;
   QuadrantDependentList<PointJacobian> toeJacobian;
   
   DenseMatrix64F comWrenchMap;
   DenseMatrix64F comWrenchMapInverse;
   DenseMatrix64F comWrenchVector;
   DenseMatrix64F toeForcesVector;
   DenseMatrix64F toeForceVector;
   QuadrantDependentList<DenseMatrix64F> legTorqueVector;

   public QuadrupedVirtualModelController(SDFFullRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, QuadrupedJointNameMap jointNameMap)
   {
      // initialize reference frames
      comFrame = referenceFrames.getCenterOfMassFrame();
      toeFrame = referenceFrames.getFootReferenceFrames();

      // initialize forces and torques
      desiredComForce = new FrameVector(comFrame);
      optimalComForce = new FrameVector(comFrame);
      desiredBodyTorque = new FrameVector(comFrame);
      optimalBodyTorque = new FrameVector(comFrame);
      desiredToeForce = new QuadrantDependentList<FrameVector>();
      optimalToeForce = new QuadrantDependentList<FrameVector>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         desiredToeForce.set(robotQuadrant, new FrameVector(comFrame));
         optimalToeForce.set(robotQuadrant, new FrameVector(comFrame));
      }

      // initialize leg joints and jacobians
      legJoints = new QuadrantDependentList<OneDoFJoint[]>();
      toePosition = new QuadrantDependentList<FramePoint>();
      footJacobian = new QuadrantDependentList<GeometricJacobian>();
      toeJacobian = new QuadrantDependentList<PointJacobian>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         String jointBeforeFootName = jointNameMap.getJointBeforeFootName(robotQuadrant);
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
         RigidBody body = fullRobotModel.getRootJoint().getSuccessor();
         RigidBody foot = jointBeforeFoot.getSuccessor();
         legJoints.set(robotQuadrant, ScrewTools.filterJoints(ScrewTools.createJointPath(body, foot), OneDoFJoint.class));
         toePosition.set(robotQuadrant, new FramePoint(comFrame));
         footJacobian.set(robotQuadrant, new GeometricJacobian(legJoints.get(robotQuadrant), body.getBodyFixedFrame()));
         toeJacobian.set(robotQuadrant, new PointJacobian());
      }

      // initialize joint torque and position limits
      minJointTorque = new QuadrantDependentList<double[]>();
      maxJointTorque = new QuadrantDependentList<double[]>();
      minJointPosition = new QuadrantDependentList<double[]>();
      maxJointPosition = new QuadrantDependentList<double[]>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         minJointTorque.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         maxJointTorque.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         minJointPosition.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         maxJointPosition.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         for (int i = 0; i < legJoints.get(robotQuadrant).length; ++i)
         {
            minJointTorque.get(robotQuadrant)[i] = Double.MIN_VALUE;
            maxJointTorque.get(robotQuadrant)[i] = Double.MAX_VALUE;
            minJointPosition.get(robotQuadrant)[i] = Double.MIN_VALUE;
            maxJointPosition.get(robotQuadrant)[i] = Double.MAX_VALUE;
         }
      }

      // initialize matrix terms
      comWrenchMap = new DenseMatrix64F(6, 12);
      comWrenchMapInverse = new DenseMatrix64F(12, 6);
      comWrenchVector = new DenseMatrix64F(6, 1);
      toeForcesVector = new DenseMatrix64F(12, 1);
      toeForceVector = new DenseMatrix64F(3, 1);
      legTorqueVector = new QuadrantDependentList<DenseMatrix64F>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         legTorqueVector.set(robotQuadrant, new DenseMatrix64F(legJoints.get(robotQuadrant).length, 1));
      }

   }
   
   public void setLegTorqueHardLimits(RobotQuadrant robotQuadrant, double[] min, double[] max)
   {
      for (int i = 0; i < legJoints.get(robotQuadrant).length; ++i)
      {
         minJointTorque.get(robotQuadrant)[i] = min[i];
         maxJointTorque.get(robotQuadrant)[i] = max[i];
      }
   }

   public void setJointPositionSoftLimits(RobotQuadrant robotQuadrant, double[] min, double[] max)
   {
      for (int i = 0; i < legJoints.get(robotQuadrant).length; ++i)
      {
         minJointPosition.get(robotQuadrant)[i] = min[i];
         maxJointPosition.get(robotQuadrant)[i] = max[i];
      }
   }

   public void setJointPositionStiffness(RobotQuadrant robotQuadrant, double[] stiffness)
   {
      for (int i = 0; i < legJoints.get(robotQuadrant).length; ++i)
      {
         jointPositionStiffness.get(robotQuadrant)[i] = stiffness[i];
      }
      
   }

   public void setJointPositionDamping(RobotQuadrant robotQuadrant, double[] damping)
   {
      for (int i = 0; i < legJoints.get(robotQuadrant).length; ++i)
      {
         jointPositionDamping.get(robotQuadrant)[i] = damping[i];
      }
      
   }

   public void setDesiredComForce(FrameVector comForce)
   {
      desiredComForce.setIncludingFrame(comForce);
   }

   public void setDesiredBodyTorque(FrameVector bodyTorque)
   {
      desiredBodyTorque.setIncludingFrame(bodyTorque);
   }

   public void setDesiredToeForce(RobotQuadrant robotQuadrant, FrameVector toeForce)
   {
      desiredToeForce.get(robotQuadrant).set(toeForce);
   }

   public void getOptimalComForce(FrameVector comForce)
   {
      comForce.setIncludingFrame(optimalComForce); 
   }

   public void getOptimalBodyTorque(FrameVector bodyTorque)
   {
      bodyTorque.setIncludingFrame(optimalBodyTorque); 
   }

   public void getOptimalToeForce(RobotQuadrant robotQuadrant, FrameVector toeForce)
   {
      toeForce.set(optimalToeForce.get(robotQuadrant));
   }

   public void update()
   {
      // rotate desired forces and torques to center of mass frame
      desiredComForce.changeFrame(comFrame);
      desiredBodyTorque.changeFrame(comFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         desiredToeForce.get(robotQuadrant).changeFrame(comFrame);
      }

      // compute toe positions and jacobians in center of mass frame
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         toePosition.get(robotQuadrant).setToZero(toeFrame.get(robotQuadrant));
         toePosition.get(robotQuadrant).changeFrame(comFrame);
         footJacobian.get(robotQuadrant).compute();
         toeJacobian.get(robotQuadrant).set(footJacobian.get(robotQuadrant), toePosition.get(robotQuadrant));
         toeJacobian.get(robotQuadrant).compute();

//       System.out.println(robotQuadrant + " " + footJacobian.get(robotQuadrant));
//       System.out.println(robotQuadrant + " " + toeJacobian.get(robotQuadrant))
      }

      // compute map from toe forces to centroidal forces and torques
      comWrenchMap.zero();
      int columnOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         comWrenchMap.set(0, 1 + columnOffset, -toePosition.get(robotQuadrant).getZ()); // mX row
         comWrenchMap.set(0, 2 + columnOffset, toePosition.get(robotQuadrant).getY());
         comWrenchMap.set(1, 0 + columnOffset, toePosition.get(robotQuadrant).getZ());  // mY row
         comWrenchMap.set(1, 2 + columnOffset, -toePosition.get(robotQuadrant).getX());
         comWrenchMap.set(2, 0 + columnOffset, -toePosition.get(robotQuadrant).getY()); // mZ row
         comWrenchMap.set(2, 1 + columnOffset, toePosition.get(robotQuadrant).getX());
         comWrenchMap.set(3, 0 + columnOffset, 1.0); // fX row
         comWrenchMap.set(4, 1 + columnOffset, 1.0); // fY row
         comWrenchMap.set(5, 2 + columnOffset, 1.0); // fZ row
         columnOffset += 3;
      }

      // compute centroidal wrench vector
      comWrenchVector.set(0, 0, desiredBodyTorque.getX());
      comWrenchVector.set(1, 0, desiredBodyTorque.getY());
      comWrenchVector.set(2, 0, desiredBodyTorque.getZ());
      comWrenchVector.set(3, 0, desiredComForce.getX());
      comWrenchVector.set(4, 0, desiredComForce.getY());
      comWrenchVector.set(5, 0, desiredComForce.getZ());

      // compute optimal foot forces using least squares solution
      try
      {
         CommonOps.pinv(comWrenchMap, comWrenchMapInverse);
      }
      catch (Exception e)
      {
         return;
      }
      CommonOps.mult(comWrenchMapInverse, comWrenchVector, toeForcesVector);
      int rowOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         optimalToeForce.get(robotQuadrant).changeFrame(comFrame);
         optimalToeForce.get(robotQuadrant).setX(toeForcesVector.get(0 + rowOffset, 0));
         optimalToeForce.get(robotQuadrant).setY(toeForcesVector.get(1 + rowOffset, 0));
         optimalToeForce.get(robotQuadrant).setZ(toeForcesVector.get(2 + rowOffset, 0));
         rowOffset += 3;
      }

      // compute optimal centroidal forces and torques
      CommonOps.mult(comWrenchMap, toeForcesVector, comWrenchVector);
      optimalBodyTorque.changeFrame(comFrame);
      optimalBodyTorque.setX(comWrenchVector.get(0, 0));
      optimalBodyTorque.setY(comWrenchVector.get(1, 0));
      optimalBodyTorque.setZ(comWrenchVector.get(2, 0));
      optimalComForce.changeFrame(comFrame);
      optimalComForce.setX(comWrenchVector.get(3, 0));
      optimalComForce.setY(comWrenchVector.get(4, 0));
      optimalComForce.setZ(comWrenchVector.get(5, 0));

      // compute joint torques using jacobian transpose
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         DenseMatrix64F jacobianMatrix = toeJacobian.get(robotQuadrant).getJacobianMatrix();
         ReferenceFrame jacobianFrame = toeJacobian.get(robotQuadrant).getFrame();
         optimalToeForce.get(robotQuadrant).changeFrame(jacobianFrame);
         toeForceVector.set(0, 0, -optimalToeForce.get(robotQuadrant).getX());
         toeForceVector.set(1, 0, -optimalToeForce.get(robotQuadrant).getY());
         toeForceVector.set(2, 0, -optimalToeForce.get(robotQuadrant).getZ());
         optimalToeForce.get(robotQuadrant).changeFrame(comFrame);
         CommonOps.multTransA(jacobianMatrix, toeForceVector, legTorqueVector.get(robotQuadrant));
   
         // update joint torques in full robot model
         int row = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            joint.setTau(legTorqueVector.get(robotQuadrant).get(row++, 0));
         }
         //		    System.out.println("forces = " + forces);
         //		    System.out.println("jacobian = " + jacobian);
         //		    System.out.println("torques = " + torques);
      }
      
      // TODO
      // apply joint torque limits
      // apply contact force limits
      // compute quadratic cost terms (com wrench error, foot force error, regularization)
      // compute joint torque inequality constraints
      // compute friction pyramid inequality constraints
      // compute foot forces using quadratic program
   }

}
