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
   private final ReferenceFrame comFrame;
   private final QuadrantDependentList<ReferenceFrame> soleFrame;

   private final FrameVector desiredComForce;
   private final FrameVector optimalComForce;
   private final FrameVector desiredBodyTorque;
   private final FrameVector optimalBodyTorque;
   private final QuadrantDependentList<FrameVector> desiredSoleForce;
   private final QuadrantDependentList<FrameVector> optimalSoleForce;

   private final QuadrantDependentList<double[]> jointEffortLowerLimit;
   private final QuadrantDependentList<double[]> jointEffortUpperLimit;
   private final QuadrantDependentList<double[]> jointPositionLowerLimit;
   private final QuadrantDependentList<double[]> jointPositionUpperLimit;
   private final QuadrantDependentList<double[]> jointPositionLimitStiffness;
   private final QuadrantDependentList<double[]> jointPositionLimitDamping;
   private final QuadrantDependentList<double[]> coefficientOfFriction;

   private final QuadrantDependentList<OneDoFJoint[]> legJoints;
   private final QuadrantDependentList<FramePoint> solePosition;
   private final QuadrantDependentList<GeometricJacobian> footJacobian;
   private final QuadrantDependentList<PointJacobian> soleJacobian;

   private final DenseMatrix64F comWrenchMap;
   private final DenseMatrix64F comWrenchMapInverse;
   private final DenseMatrix64F comWrenchVector;
   private final DenseMatrix64F soleForcesVector;
   private final DenseMatrix64F soleForceVector;
   private final QuadrantDependentList<DenseMatrix64F> legEffortVector;

   public QuadrupedVirtualModelController(SDFFullRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, QuadrupedJointNameMap jointNameMap)
   {
      // initialize reference frames
      comFrame = referenceFrames.getCenterOfMassZUpFrame();
      soleFrame = referenceFrames.getFootReferenceFrames();

      // initialize desired and optimal values
      desiredComForce = new FrameVector(comFrame);
      optimalComForce = new FrameVector(comFrame);
      desiredBodyTorque = new FrameVector(comFrame);
      optimalBodyTorque = new FrameVector(comFrame);
      desiredSoleForce = new QuadrantDependentList<FrameVector>();
      optimalSoleForce = new QuadrantDependentList<FrameVector>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         desiredSoleForce.set(robotQuadrant, new FrameVector(comFrame));
         optimalSoleForce.set(robotQuadrant, new FrameVector(comFrame));
      }

      // initialize jacobians
      legJoints = new QuadrantDependentList<OneDoFJoint[]>();
      solePosition = new QuadrantDependentList<FramePoint>();
      footJacobian = new QuadrantDependentList<GeometricJacobian>();
      soleJacobian = new QuadrantDependentList<PointJacobian>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String jointBeforeFootName = jointNameMap.getJointBeforeFootName(robotQuadrant);
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
         RigidBody body = fullRobotModel.getRootJoint().getSuccessor();
         RigidBody foot = jointBeforeFoot.getSuccessor();
         legJoints.set(robotQuadrant, ScrewTools.filterJoints(ScrewTools.createJointPath(body, foot), OneDoFJoint.class));
         solePosition.set(robotQuadrant, new FramePoint(foot.getBodyFixedFrame()));
         footJacobian.set(robotQuadrant, new GeometricJacobian(legJoints.get(robotQuadrant), body.getBodyFixedFrame()));
         soleJacobian.set(robotQuadrant, new PointJacobian());
      }

      // initialize limits
      coefficientOfFriction = new QuadrantDependentList<double[]>();
      jointEffortLowerLimit = new QuadrantDependentList<double[]>();
      jointEffortUpperLimit = new QuadrantDependentList<double[]>();
      jointPositionLowerLimit = new QuadrantDependentList<double[]>();
      jointPositionUpperLimit = new QuadrantDependentList<double[]>();
      jointPositionLimitStiffness = new QuadrantDependentList<double[]>();
      jointPositionLimitDamping = new QuadrantDependentList<double[]>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         jointEffortLowerLimit.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointEffortUpperLimit.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointPositionLowerLimit.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointPositionUpperLimit.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointPositionLimitStiffness.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         jointPositionLimitDamping.set(robotQuadrant, new double[legJoints.get(robotQuadrant).length]);
         coefficientOfFriction.set(robotQuadrant, new double[1]);
         for (int i = 0; i < legJoints.get(robotQuadrant).length; i++)
         {
            jointEffortLowerLimit.get(robotQuadrant)[i] = -(Double.MAX_VALUE - 1);
            jointEffortUpperLimit.get(robotQuadrant)[i] = Double.MAX_VALUE;
            jointPositionLowerLimit.get(robotQuadrant)[i] = -(Double.MAX_VALUE - 1);
            jointPositionUpperLimit.get(robotQuadrant)[i] = Double.MAX_VALUE;
            jointPositionLimitStiffness.get(robotQuadrant)[i] = 1000;
            jointPositionLimitDamping.get(robotQuadrant)[i] = 250;
         }
         coefficientOfFriction.get(robotQuadrant)[0] = 0.6;
      }

      // initialize matrix terms
      comWrenchMap = new DenseMatrix64F(6, 12);
      comWrenchMapInverse = new DenseMatrix64F(12, 6);
      comWrenchVector = new DenseMatrix64F(6, 1);
      soleForcesVector = new DenseMatrix64F(12, 1);
      soleForceVector = new DenseMatrix64F(3, 1);
      legEffortVector = new QuadrantDependentList<DenseMatrix64F>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         legEffortVector.set(robotQuadrant, new DenseMatrix64F(legJoints.get(robotQuadrant).length, 1));
      }
   }

   public void setCoefficientOfFriction(RobotQuadrant robotQuadrant, double coefficientOfFriction)
   {
      this.coefficientOfFriction.get(robotQuadrant)[0] = coefficientOfFriction;
   }

   public void setJointEffortLimits(String jointName, double lower, double upper)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            if (joint.getName().equals(jointName))
            {
               jointEffortLowerLimit.get(robotQuadrant)[index] = lower;
               jointEffortUpperLimit.get(robotQuadrant)[index] = upper;
            }
            index++;
         }
      }
   }

   public void setJointPositionLimits(String jointName, double lower, double upper)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            if (joint.getName().equals(jointName))
            {
               jointPositionLowerLimit.get(robotQuadrant)[index] = lower;
               jointPositionUpperLimit.get(robotQuadrant)[index] = upper;
            }
            index++;
         }
      }
   }

   public void setJointPositionLimitStiffness(String jointName, double stiffness)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            if (joint.getName().equals(jointName))
            {
               jointPositionLimitStiffness.get(robotQuadrant)[index] = stiffness;
            }
            index++;
         }
      }
   }

   public void setJointPositionLimitDamping(String jointName, double damping)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            if (joint.getName().equals(jointName))
            {
               jointPositionLimitDamping.get(robotQuadrant)[index] = damping;
            }
            index++;
         }
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

   public void setDesiredSoleForce(RobotQuadrant robotQuadrant, FrameVector soleForce)
   {
      desiredSoleForce.get(robotQuadrant).set(soleForce);
   }

   public void getOptimalComForce(FrameVector comForce)
   {
      comForce.setIncludingFrame(optimalComForce);
   }

   public void getOptimalBodyTorque(FrameVector bodyTorque)
   {
      bodyTorque.setIncludingFrame(optimalBodyTorque);
   }

   public void getOptimalSoleForce(RobotQuadrant robotQuadrant, FrameVector soleForce)
   {
      soleForce.set(optimalSoleForce.get(robotQuadrant));
   }

   public void compute()
   {
      // rotate desired forces and torques to center of mass frame
      desiredComForce.changeFrame(comFrame);
      desiredBodyTorque.changeFrame(comFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         desiredSoleForce.get(robotQuadrant).changeFrame(comFrame);
      }

      // compute sole positions and jacobians in center of mass frame
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         solePosition.get(robotQuadrant).changeFrame(comFrame);
         footJacobian.get(robotQuadrant).compute();
         soleJacobian.get(robotQuadrant).set(footJacobian.get(robotQuadrant), solePosition.get(robotQuadrant));
         soleJacobian.get(robotQuadrant).compute();
      }

      // compute map from sole forces to centroidal forces and torques
      comWrenchMap.zero();
      int columnOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         comWrenchMap.set(0, 1 + columnOffset, -solePosition.get(robotQuadrant).getZ()); // mX row
         comWrenchMap.set(0, 2 + columnOffset, solePosition.get(robotQuadrant).getY());
         comWrenchMap.set(1, 0 + columnOffset, solePosition.get(robotQuadrant).getZ()); // mY row
         comWrenchMap.set(1, 2 + columnOffset, -solePosition.get(robotQuadrant).getX());
         comWrenchMap.set(2, 0 + columnOffset, -solePosition.get(robotQuadrant).getY()); // mZ row
         comWrenchMap.set(2, 1 + columnOffset, solePosition.get(robotQuadrant).getX());
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

      // compute optimal sole forces using least squares solution
      try
      {
         CommonOps.pinv(comWrenchMap, comWrenchMapInverse);
      }
      catch (Exception e)
      {
         return;
      }
      CommonOps.mult(comWrenchMapInverse, comWrenchVector, soleForcesVector);
      int rowOffset = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         optimalSoleForce.get(robotQuadrant).changeFrame(comFrame);
         optimalSoleForce.get(robotQuadrant).setX(soleForcesVector.get(0 + rowOffset, 0));
         optimalSoleForce.get(robotQuadrant).setY(soleForcesVector.get(1 + rowOffset, 0));
         optimalSoleForce.get(robotQuadrant).setZ(soleForcesVector.get(2 + rowOffset, 0));
         rowOffset += 3;
      }

      // apply contact force limits
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         double mu = coefficientOfFriction.get(robotQuadrant)[0];
         double fx = optimalSoleForce.get(robotQuadrant).getX();
         double fy = optimalSoleForce.get(robotQuadrant).getY();
         double fz = optimalSoleForce.get(robotQuadrant).getZ();
         fz = Math.max(fz, 0);
         fx = Math.min(fx, mu * fz / Math.sqrt(2));
         fy = Math.min(fy, mu * fz / Math.sqrt(2));
         optimalSoleForce.get(robotQuadrant).setX(fx);
         optimalSoleForce.get(robotQuadrant).setY(fy);
         optimalSoleForce.get(robotQuadrant).setZ(fz);
      }

      // compute optimal centroidal forces and torques
      CommonOps.mult(comWrenchMap, soleForcesVector, comWrenchVector);
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
         DenseMatrix64F jacobianMatrix = soleJacobian.get(robotQuadrant).getJacobianMatrix();
         ReferenceFrame jacobianFrame = soleJacobian.get(robotQuadrant).getFrame();
         optimalSoleForce.get(robotQuadrant).changeFrame(jacobianFrame);
         soleForceVector.set(0, 0, -optimalSoleForce.get(robotQuadrant).getX());
         soleForceVector.set(1, 0, -optimalSoleForce.get(robotQuadrant).getY());
         soleForceVector.set(2, 0, -optimalSoleForce.get(robotQuadrant).getZ());
         optimalSoleForce.get(robotQuadrant).changeFrame(comFrame);
         CommonOps.multTransA(jacobianMatrix, soleForceVector, legEffortVector.get(robotQuadrant));

         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            // apply joint position and torque limits
            double tauPositionLowerLimit = jointPositionLimitStiffness.get(robotQuadrant)[index]
                  * (jointPositionLowerLimit.get(robotQuadrant)[index] - joint.getQ()) - jointPositionLimitDamping.get(robotQuadrant)[index] * joint.getQd();
            double tauPositionUpperLimit = jointPositionLimitStiffness.get(robotQuadrant)[index]
                  * (jointPositionUpperLimit.get(robotQuadrant)[index] - joint.getQ()) - jointPositionLimitDamping.get(robotQuadrant)[index] * joint.getQd();
            double tauEffortLowerLimit = jointEffortLowerLimit.get(robotQuadrant)[index];
            double tauEffortUpperLimit = jointEffortUpperLimit.get(robotQuadrant)[index];
            double tau = legEffortVector.get(robotQuadrant).get(index, 0);
            tau = Math.min(Math.max(tau, tauPositionLowerLimit), tauPositionUpperLimit);
            tau = Math.min(Math.max(tau, tauEffortLowerLimit), tauEffortUpperLimit);

            // update joint torques in full robot model
            joint.setTau(tau);
            index++;
         }
      }

      // TODO
      // compute quadratic cost terms (com wrench error, foot force error, force regularization)
      // compute joint torque inequality constraints
      // compute friction pyramid inequality constraints
      // compute min / max sole pressure constraints?
      // compute sole forces using quadratic program
   }

}
