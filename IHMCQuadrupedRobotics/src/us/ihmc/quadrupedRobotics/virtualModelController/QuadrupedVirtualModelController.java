package us.ihmc.quadrupedRobotics.virtualModelController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointLimits;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedVirtualModelParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
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
   private final YoVariableRegistry registry; 

   private final QuadrupedVirtualModelParameters parameters;
   private final QuadrupedJointNameMap jointMap;
   private final QuadrupedJointLimits jointLimits;
   private final QuadrupedReferenceFrames referenceFrames;
   private final ReferenceFrame comFrame;
   private final QuadrantDependentList<ReferenceFrame> soleFrame;
   private final ReferenceFrame worldFrame;

   private final FrameVector comForceSetpoint;
   private final FrameVector comForceOptimal;
   private final FrameVector bodyTorqueSetpoint;
   private final FrameVector bodyTorqueOptimal;
   private final QuadrantDependentList<FrameVector> soleForceSetpoint;
   private final QuadrantDependentList<FrameVector> soleForceOptimal;
   private final QuadrantDependentList<double []> soleCoefficientOfFriction;

   private final YoFrameVector yoComForceSetpoint;
   private final YoFrameVector yoComForceOptimal;
   private final YoFrameVector yoBodyTorqueSetpoint;
   private final YoFrameVector yoBodyTorqueOptimal;
   private final QuadrantDependentList<YoFrameVector> yoSoleForceSetpoint;
   private final QuadrantDependentList<YoFrameVector> yoSoleForceOptimal;

   private final QuadrantDependentList<double[]> jointEffortLowerLimit;
   private final QuadrantDependentList<double[]> jointEffortUpperLimit;
   private final QuadrantDependentList<double[]> jointPositionLowerLimit;
   private final QuadrantDependentList<double[]> jointPositionUpperLimit;
   private final QuadrantDependentList<double[]> jointPositionLimitStiffness;
   private final QuadrantDependentList<double[]> jointPositionLimitDamping;

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

   public QuadrupedVirtualModelController(SDFFullRobotModel fullRobotModel, QuadrupedRobotParameters robotParameters, YoVariableRegistry parentRegistry)
   {
      parameters = robotParameters.getQuadrupedVirtualModelParameters();
      jointMap = robotParameters.getJointMap();
      jointLimits = robotParameters.getJointLimits();
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, jointMap, robotParameters.getPhysicalProperties());
      registry = new YoVariableRegistry(getClass().getSimpleName());
      
      // initialize reference frames
      comFrame = referenceFrames.getCenterOfMassZUpFrame();
      soleFrame = referenceFrames.getFootReferenceFrames();
      worldFrame = ReferenceFrame.getWorldFrame();

      // initialize optimization variables
      comForceSetpoint = new FrameVector(comFrame);
      comForceOptimal = new FrameVector(comFrame);
      bodyTorqueSetpoint = new FrameVector(comFrame);
      bodyTorqueOptimal = new FrameVector(comFrame);
      soleForceSetpoint = new QuadrantDependentList<FrameVector>();
      soleForceOptimal = new QuadrantDependentList<FrameVector>();
      soleCoefficientOfFriction = new QuadrantDependentList<double[]>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleForceSetpoint.set(robotQuadrant, new FrameVector(comFrame));
         soleForceOptimal.set(robotQuadrant, new FrameVector(comFrame));
         soleCoefficientOfFriction.set(robotQuadrant, new double[1]);
      }

      // initialize yo variables
      yoComForceSetpoint = new YoFrameVector("comForceSetpoint", worldFrame, registry);
      yoComForceOptimal = new YoFrameVector("comForceOptimal", worldFrame, registry);
      yoBodyTorqueSetpoint = new YoFrameVector("bodyTorqueSetpoint", worldFrame, registry);
      yoBodyTorqueOptimal = new YoFrameVector("soleForceSetpoint", worldFrame, registry);
      yoSoleForceSetpoint = new QuadrantDependentList<YoFrameVector>();
      yoSoleForceOptimal = new QuadrantDependentList<YoFrameVector>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSoleForceSetpoint.set(robotQuadrant,
               new YoFrameVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SoleForceSetpoint", worldFrame, registry));
         yoSoleForceOptimal.set(robotQuadrant,
               new YoFrameVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SoleForceOptimal", worldFrame, registry));
      }

      // initialize jacobians
      legJoints = new QuadrantDependentList<OneDoFJoint[]>();
      solePosition = new QuadrantDependentList<FramePoint>();
      footJacobian = new QuadrantDependentList<GeometricJacobian>();
      soleJacobian = new QuadrantDependentList<PointJacobian>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String jointBeforeFootName = robotParameters.getJointMap().getJointBeforeFootName(robotQuadrant);
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
         RigidBody body = fullRobotModel.getRootJoint().getSuccessor();
         RigidBody foot = jointBeforeFoot.getSuccessor();
         legJoints.set(robotQuadrant, ScrewTools.filterJoints(ScrewTools.createJointPath(body, foot), OneDoFJoint.class));
         solePosition.set(robotQuadrant, new FramePoint(foot.getBodyFixedFrame()));
         footJacobian.set(robotQuadrant, new GeometricJacobian(legJoints.get(robotQuadrant), body.getBodyFixedFrame()));
         soleJacobian.set(robotQuadrant, new PointJacobian());
      }

      // initialize limits
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

      parentRegistry.addChild(registry);

      this.reinitialize();
   }

   public void reinitialize()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         for (int i = 0; i < jointMap.getLegJointNames().length; i++)
         {
            // initialize joint limits
            LegJointName legJointName = jointMap.getLegJointNames()[i];
            String jointName = jointMap.getLegJointName(robotQuadrant, legJointName);
            double positionLowerLimit = jointLimits.getJointSoftPositionLowerLimit(jointName);
            double positionUpperLimit = jointLimits.getJointSoftPositionUpperLimit(jointName);
            double effortLimit = jointLimits.getJointEffortLimit(jointName);
            setJointEffortLimits(jointName, -effortLimit, effortLimit);
            setJointPositionLimits(jointName, positionLowerLimit, positionUpperLimit);
            setJointPositionLimitStiffness(jointName, parameters.getDefaultJointPositionLimitStiffness());
            setJointPositionLimitDamping(jointName, parameters.getDefaultJointPositionLimitDamping());
         }
         // initialize friction limits
         setSoleCoefficientOfFriction(robotQuadrant, parameters.getDefaultSoleCoefficientOfFriction());
      }
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

   public void setComForceSetpoint(FrameVector comForce)
   {
      comForceSetpoint.setIncludingFrame(comForce);
   }

   public void setBodyTorqueSetpoint(FrameVector bodyTorque)
   {
      bodyTorqueSetpoint.setIncludingFrame(bodyTorque);
   }

   public void setSoleForceSetpoint(RobotQuadrant robotQuadrant, FrameVector soleForce)
   {
      soleForceSetpoint.get(robotQuadrant).set(soleForce);
   }

   public void setSoleCoefficientOfFriction(RobotQuadrant robotQuadrant, double coefficientOfFriction)
   {
      this.soleCoefficientOfFriction.get(robotQuadrant)[0] = coefficientOfFriction;
   }

   public void getComForceOptimal(FrameVector comForce)
   {
      comForce.setIncludingFrame(comForceOptimal);
   }

   public void getBodyTorqueOptimal(FrameVector bodyTorque)
   {
      bodyTorque.setIncludingFrame(bodyTorqueOptimal);
   }

   public void getSoleForceOptimal(RobotQuadrant robotQuadrant, FrameVector soleForce)
   {
      soleForce.set(soleForceOptimal.get(robotQuadrant));
   }

   public double getSoleCoefficientOfFriction(RobotQuadrant robotQuadrant)
   {
      return soleCoefficientOfFriction.get(robotQuadrant)[0];
   }

   public void compute()
   {
      // rotate desired forces and torques to center of mass frame
      comForceSetpoint.changeFrame(comFrame);
      bodyTorqueSetpoint.changeFrame(comFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleForceSetpoint.get(robotQuadrant).changeFrame(comFrame);
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
      comWrenchVector.set(0, 0, bodyTorqueSetpoint.getX());
      comWrenchVector.set(1, 0, bodyTorqueSetpoint.getY());
      comWrenchVector.set(2, 0, bodyTorqueSetpoint.getZ());
      comWrenchVector.set(3, 0, comForceSetpoint.getX());
      comWrenchVector.set(4, 0, comForceSetpoint.getY());
      comWrenchVector.set(5, 0, comForceSetpoint.getZ());

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
         soleForceOptimal.get(robotQuadrant).changeFrame(comFrame);
         soleForceOptimal.get(robotQuadrant).setX(soleForcesVector.get(0 + rowOffset, 0));
         soleForceOptimal.get(robotQuadrant).setY(soleForcesVector.get(1 + rowOffset, 0));
         soleForceOptimal.get(robotQuadrant).setZ(soleForcesVector.get(2 + rowOffset, 0));
         rowOffset += 3;

         // apply contact force limits
         double mu = soleCoefficientOfFriction.get(robotQuadrant)[0];
         double fx = soleForceOptimal.get(robotQuadrant).getX();
         double fy = soleForceOptimal.get(robotQuadrant).getY();
         double fz = soleForceOptimal.get(robotQuadrant).getZ();
         fz = Math.max(fz, 0);
         fx = Math.max(fx,-mu * fz / Math.sqrt(2));
         fx = Math.min(fx, mu * fz / Math.sqrt(2));
         fy = Math.max(fy,-mu * fz / Math.sqrt(2));
         fy = Math.min(fy, mu * fz / Math.sqrt(2));
         soleForceOptimal.get(robotQuadrant).setX(fx);
         soleForceOptimal.get(robotQuadrant).setY(fy);
         soleForceOptimal.get(robotQuadrant).setZ(fz);
      }

      // compute optimal body torques and CoM forces (accounting for sole force limits)
      CommonOps.mult(comWrenchMap, soleForcesVector, comWrenchVector);
      bodyTorqueOptimal.changeFrame(comFrame);
      bodyTorqueOptimal.setX(comWrenchVector.get(0, 0));
      bodyTorqueOptimal.setY(comWrenchVector.get(1, 0));
      bodyTorqueOptimal.setZ(comWrenchVector.get(2, 0));
      comForceOptimal.changeFrame(comFrame);
      comForceOptimal.setX(comWrenchVector.get(3, 0));
      comForceOptimal.setY(comWrenchVector.get(4, 0));
      comForceOptimal.setZ(comWrenchVector.get(5, 0));

      // compute joint torques using jacobian transpose
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         DenseMatrix64F jacobianMatrix = soleJacobian.get(robotQuadrant).getJacobianMatrix();
         ReferenceFrame jacobianFrame = soleJacobian.get(robotQuadrant).getFrame();
         soleForceOptimal.get(robotQuadrant).changeFrame(jacobianFrame);
         soleForceVector.set(0, 0, -soleForceOptimal.get(robotQuadrant).getX());
         soleForceVector.set(1, 0, -soleForceOptimal.get(robotQuadrant).getY());
         soleForceVector.set(2, 0, -soleForceOptimal.get(robotQuadrant).getZ());
         soleForceOptimal.get(robotQuadrant).changeFrame(comFrame);
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

      // update yo variables
      comForceSetpoint.changeFrame(yoComForceSetpoint.getReferenceFrame());
      yoComForceSetpoint.set(comForceSetpoint);
      comForceOptimal.changeFrame(yoComForceOptimal.getReferenceFrame());
      yoComForceOptimal.set(comForceOptimal);
      bodyTorqueSetpoint.changeFrame(yoBodyTorqueSetpoint.getReferenceFrame());
      yoBodyTorqueSetpoint.set(bodyTorqueSetpoint);
      bodyTorqueOptimal.changeFrame(yoBodyTorqueOptimal.getReferenceFrame());
      yoBodyTorqueOptimal.set(bodyTorqueOptimal);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleForceSetpoint.get(robotQuadrant).changeFrame(yoSoleForceSetpoint.get(robotQuadrant).getReferenceFrame());
         yoSoleForceSetpoint.get(robotQuadrant).set(soleForceSetpoint.get(robotQuadrant));
         soleForceOptimal.get(robotQuadrant).changeFrame(yoSoleForceOptimal.get(robotQuadrant).getReferenceFrame());
         yoSoleForceOptimal.get(robotQuadrant).set(soleForceOptimal.get(robotQuadrant));
      }

      // TODO
      // compute quadratic cost terms (com wrench error, foot force error, force regularization)
      // compute joint torque inequality constraints
      // compute friction pyramid inequality constraints
      // compute min / max sole pressure constraints
      // compute sole forces using quadratic program
   }
   
   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
  
   public QuadrupedReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }
}
