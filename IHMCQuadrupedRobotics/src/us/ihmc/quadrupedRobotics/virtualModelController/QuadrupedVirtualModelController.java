package us.ihmc.quadrupedRobotics.virtualModelController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointLimits;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedVirtualModelParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.PointJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedVirtualModelController
{
   private final YoVariableRegistry registry;
   private final QuadrupedVirtualModelParameters parameters;
   private final QuadrupedJointNameMap jointMap;
   private final QuadrupedJointLimits jointLimits;

   private final QuadrupedReferenceFrames referenceFrames;
   private final ReferenceFrame comFrame;
   private final ReferenceFrame worldFrame;
   private final QuadrantDependentList<ReferenceFrame> soleFrame;

   private final QuadrupedContactForceLimits contactForceLimits;
   private final QuadrupedContactForceOptimizationSettings contactForceOptimizationSettings;
   private final QuadrupedContactForceOptimization contactForceOptimization;

   private final FrameVector comForceCommand;
   private final FrameVector comForceSolution;
   private final FrameVector comTorqueCommand;
   private final FrameVector comTorqueSolution;
   private final QuadrantDependentList<FrameVector> swingForceCommand;
   private final QuadrantDependentList<FrameVector> swingForceSolution;
   private final QuadrantDependentList<FrameVector> contactForceSolution;
   private final QuadrantDependentList<boolean[]> contactState;
   private final QuadrantDependentList<FramePoint> solePosition;

   private final YoFrameVector yoComForceCommand;
   private final YoFrameVector yoComForceSolution;
   private final YoFrameVector yoComTorqueCommand;
   private final YoFrameVector yoComTorqueSolution;
   private final QuadrantDependentList<YoFrameVector> yoSwingForceCommand;
   private final QuadrantDependentList<YoFrameVector> yoSwingForceSolution;
   private final QuadrantDependentList<YoFrameVector> yoContactForceSolution;
   private final QuadrantDependentList<BooleanYoVariable> yoContactState;
   private final QuadrantDependentList<YoFramePoint> yoSolePosition;

   private final QuadrantDependentList<double[]> jointEffortLowerLimit;
   private final QuadrantDependentList<double[]> jointEffortUpperLimit;
   private final QuadrantDependentList<double[]> jointPositionLowerLimit;
   private final QuadrantDependentList<double[]> jointPositionUpperLimit;
   private final QuadrantDependentList<double[]> jointPositionLimitStiffness;
   private final QuadrantDependentList<double[]> jointPositionLimitDamping;

   private final QuadrantDependentList<OneDoFJoint[]> legJoints;
   private final QuadrantDependentList<GeometricJacobian> footJacobian;
   private final QuadrantDependentList<PointJacobian> soleJacobian;
   private final QuadrantDependentList<DenseMatrix64F> legEffortVector;
   private final DenseMatrix64F virtualForceVector;

   private final YoGraphicsList yoGraphicsList;
   private final QuadrantDependentList<YoGraphicVector> yoContactForceSolutionViz;
   private boolean contactForceIsVisible;

   public QuadrupedVirtualModelController(SDFFullRobotModel fullRobotModel, QuadrupedRobotParameters robotParameters, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      parameters = robotParameters.getQuadrupedVirtualModelParameters();
      jointMap = robotParameters.getJointMap();
      jointLimits = robotParameters.getJointLimits();
      registry = new YoVariableRegistry(getClass().getSimpleName());

      // initialize reference frames
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, jointMap, robotParameters.getPhysicalProperties());
      comFrame = referenceFrames.getCenterOfMassZUpFrame();
      worldFrame = ReferenceFrame.getWorldFrame();
      soleFrame = referenceFrames.getFootReferenceFrames();

      // initialize contact force optimization
      contactForceLimits = new QuadrupedContactForceLimits();
      contactForceOptimizationSettings = new QuadrupedContactForceOptimizationSettings();
      contactForceOptimization = new QuadrupedContactForceOptimization(referenceFrames);

      // initialize control variables
      comForceCommand = new FrameVector(comFrame);
      comForceSolution = new FrameVector(comFrame);
      comTorqueCommand = new FrameVector(comFrame);
      comTorqueSolution = new FrameVector(comFrame);
      swingForceCommand = new QuadrantDependentList<FrameVector>();
      swingForceSolution = new QuadrantDependentList<FrameVector>();
      contactForceSolution = new QuadrantDependentList<FrameVector>();
      contactState = new QuadrantDependentList<boolean[]>();
      solePosition = new QuadrantDependentList<FramePoint>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingForceCommand.set(robotQuadrant, new FrameVector(comFrame));
         swingForceSolution.set(robotQuadrant, new FrameVector(comFrame));
         contactForceSolution.set(robotQuadrant, new FrameVector(comFrame));
         contactState.set(robotQuadrant, new boolean[1]);
         solePosition.set(robotQuadrant, new FramePoint(comFrame));
      }

      // initialize yo variables
      yoComForceCommand = new YoFrameVector("comForceCommand", worldFrame, registry);
      yoComForceSolution = new YoFrameVector("comForceSolution", worldFrame, registry);
      yoComTorqueCommand = new YoFrameVector("comTorqueCommand", worldFrame, registry);
      yoComTorqueSolution = new YoFrameVector("comTorqueSolution", worldFrame, registry);
      yoSwingForceCommand = new QuadrantDependentList<YoFrameVector>();
      yoSwingForceSolution = new QuadrantDependentList<YoFrameVector>();
      yoContactForceSolution = new QuadrantDependentList<YoFrameVector>();
      yoContactState = new QuadrantDependentList<BooleanYoVariable>();
      yoSolePosition = new QuadrantDependentList<YoFramePoint>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSwingForceCommand.set(robotQuadrant,
               new YoFrameVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SwingForceCommand", worldFrame, registry));
         yoSwingForceSolution.set(robotQuadrant,
               new YoFrameVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SwingForceSolution", worldFrame, registry));
         yoContactForceSolution.set(robotQuadrant,
               new YoFrameVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "ContactForceSolution", worldFrame, registry));
         yoContactState.set(robotQuadrant, new BooleanYoVariable(robotQuadrant.getCamelCaseNameForStartOfExpression() + "ContactState", registry));
         yoSolePosition.set(robotQuadrant, new YoFramePoint(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SolePosition", worldFrame, registry));
      }

      // initialize jacobians
      legJoints = new QuadrantDependentList<OneDoFJoint[]>();
      footJacobian = new QuadrantDependentList<GeometricJacobian>();
      soleJacobian = new QuadrantDependentList<PointJacobian>();
      legEffortVector = new QuadrantDependentList<DenseMatrix64F>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String jointBeforeFootName = robotParameters.getJointMap().getJointBeforeFootName(robotQuadrant);
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
         RigidBody body = fullRobotModel.getRootJoint().getSuccessor();
         RigidBody foot = jointBeforeFoot.getSuccessor();
         legJoints.set(robotQuadrant, ScrewTools.filterJoints(ScrewTools.createJointPath(body, foot), OneDoFJoint.class));
         footJacobian.set(robotQuadrant, new GeometricJacobian(legJoints.get(robotQuadrant), body.getBodyFixedFrame()));
         soleJacobian.set(robotQuadrant, new PointJacobian());
         legEffortVector.set(robotQuadrant, new DenseMatrix64F(legJoints.get(robotQuadrant).length, 1));
      }
      virtualForceVector = new DenseMatrix64F(3, 1);

      // initialize joint limits
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

      // initialize graphics
      yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      yoContactForceSolutionViz = new QuadrantDependentList<YoGraphicVector>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         yoContactForceSolutionViz.set(robotQuadrant, new YoGraphicVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SoleForceSolution",
               yoSolePosition.get(robotQuadrant), yoContactForceSolution.get(robotQuadrant), 0.002, YoAppearance.Chartreuse()));
         yoGraphicsList.add(yoContactForceSolutionViz.get(robotQuadrant));
      }
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

      // initialize limits
      this.reinitialize();

      parentRegistry.addChild(registry);
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
         // initialize contact state
         setContactState(robotQuadrant, true);
      }

      // initialize commands
      comForceCommand.setToZero();
      comTorqueCommand.setToZero();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingForceCommand.get(robotQuadrant).setToZero();
      }

      // initialize visualizers
      setVisible(false);
   }

   public QuadrupedReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
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
               jointPositionLimitStiffness.get(robotQuadrant)[index] = Math.max(stiffness, 0);
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
               jointPositionLimitDamping.get(robotQuadrant)[index] = Math.max(damping, 0);
            }
            index++;
         }
      }
   }

   public void setComForceCommand(FrameVector comForce)
   {
      comForceCommand.setIncludingFrame(comForce);
   }

   public void setComForceCommandWeights(double weights[])
   {
      contactForceOptimizationSettings.setComForceCommandWeights(weights);
   }

   public void getComForceSolution(FrameVector comForce)
   {
      comForce.setIncludingFrame(comForceSolution);
   }

   public void setComTorqueCommand(FrameVector comTorque)
   {
      comTorqueCommand.setIncludingFrame(comTorque);
   }

   public void setComTorqueCommandWeights(double weights[])
   {
      contactForceOptimizationSettings.setComTorqueCommandWeights(weights);
   }

   public void getComTorqueSolution(FrameVector comTorque)
   {
      comTorque.setIncludingFrame(comTorqueSolution);
   }

   public void setSwingForceCommand(RobotQuadrant robotQuadrant, FrameVector swingForce)
   {
      swingForceCommand.get(robotQuadrant).set(swingForce);
   }

   public void getSwingForceSolution(RobotQuadrant robotQuadrant, FrameVector swingForce)
   {
      swingForce.set(swingForceSolution.get(robotQuadrant));
   }

   public void setContactState(RobotQuadrant robotQuadrant, boolean inContact)
   {
      contactState.get(robotQuadrant)[0] = inContact;
   }

   public void setContactPressureLimits(RobotQuadrant robotQuadrant, double lower, double upper)
   {
      contactForceLimits.setPressureLowerLimit(robotQuadrant, lower);
      contactForceLimits.setPressureUpperLimit(robotQuadrant, upper);
   }

   public void setContactCoefficientOfFriction(RobotQuadrant robotQuadrant, double coefficientOfFriction)
   {
      contactForceLimits.setCoefficientOfFriction(robotQuadrant, coefficientOfFriction);
   }

   public void setContactForceRegularizationWeights(double weights)
   {
      contactForceOptimizationSettings.setContactForceRegularizationWeights(weights);
   }

   public void getContactForceSolution(RobotQuadrant robotQuadrant, FrameVector contactForce)
   {
      contactForce.set(contactForceSolution.get(robotQuadrant));
   }

   public int getNumberOfContacts()
   {
      int numberOfActiveContacts = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant)[0])
         {
            numberOfActiveContacts++;
         }
      }
      return numberOfActiveContacts;
   }

   public void compute()
   {
      // compute sole positions and jacobians in center of mass frame
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         solePosition.get(robotQuadrant).changeFrame(comFrame);
         footJacobian.get(robotQuadrant).compute();
         soleJacobian.get(robotQuadrant).set(footJacobian.get(robotQuadrant), solePosition.get(robotQuadrant));
         soleJacobian.get(robotQuadrant).compute();
      }

      // compute optimal contact forces
      contactForceOptimization.setComTorqueCommand(comTorqueCommand);
      contactForceOptimization.setComForceCommand(comForceCommand);
      contactForceOptimization.solve(solePosition, contactState, contactForceLimits, contactForceOptimizationSettings);
      contactForceOptimization.getComTorqueSolution(comTorqueSolution);
      contactForceOptimization.getComForceSolution(comForceSolution);
      contactForceOptimization.getContactForceSolution(contactForceSolution);

      // compute optimal swing forces
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant)[0])
         {
            swingForceSolution.get(robotQuadrant).setToZero();
         }
         else
         {
            swingForceSolution.get(robotQuadrant).setIncludingFrame(swingForceCommand.get(robotQuadrant));
         }
      }

      // compute joint torques using jacobian transpose
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         DenseMatrix64F jacobianMatrix = soleJacobian.get(robotQuadrant).getJacobianMatrix();
         ReferenceFrame jacobianFrame = soleJacobian.get(robotQuadrant).getFrame();
         if (contactState.get(robotQuadrant)[0])
         {
            contactForceSolution.get(robotQuadrant).changeFrame(jacobianFrame);
            virtualForceVector.set(0, 0, -contactForceSolution.get(robotQuadrant).getX());
            virtualForceVector.set(1, 0, -contactForceSolution.get(robotQuadrant).getY());
            virtualForceVector.set(2, 0, -contactForceSolution.get(robotQuadrant).getZ());
            contactForceSolution.get(robotQuadrant).changeFrame(comFrame);
         }
         else
         {
            swingForceSolution.get(robotQuadrant).changeFrame(jacobianFrame);
            virtualForceVector.set(0, 0, swingForceSolution.get(robotQuadrant).getX());
            virtualForceVector.set(1, 0, swingForceSolution.get(robotQuadrant).getY());
            virtualForceVector.set(2, 0, swingForceSolution.get(robotQuadrant).getZ());
            swingForceSolution.get(robotQuadrant).changeFrame(comFrame);
         }
         CommonOps.multTransA(jacobianMatrix, virtualForceVector, legEffortVector.get(robotQuadrant));

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
      comForceCommand.changeFrame(yoComForceCommand.getReferenceFrame());
      yoComForceCommand.set(comForceCommand);
      comForceSolution.changeFrame(yoComForceSolution.getReferenceFrame());
      yoComForceSolution.set(comForceSolution);
      comTorqueCommand.changeFrame(yoComTorqueCommand.getReferenceFrame());
      yoComTorqueCommand.set(comTorqueCommand);
      comTorqueSolution.changeFrame(yoComTorqueSolution.getReferenceFrame());
      yoComTorqueSolution.set(comTorqueSolution);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingForceCommand.get(robotQuadrant).changeFrame(yoSwingForceCommand.get(robotQuadrant).getReferenceFrame());
         yoSwingForceCommand.get(robotQuadrant).set(swingForceCommand.get(robotQuadrant));
         swingForceSolution.get(robotQuadrant).changeFrame(yoSwingForceSolution.get(robotQuadrant).getReferenceFrame());
         yoSwingForceSolution.get(robotQuadrant).set(swingForceSolution.get(robotQuadrant));
         yoContactState.get(robotQuadrant).set(contactState.get(robotQuadrant)[0]);
         contactForceSolution.get(robotQuadrant).changeFrame(yoContactForceSolution.get(robotQuadrant).getReferenceFrame());
         yoContactForceSolution.get(robotQuadrant).set(contactForceSolution.get(robotQuadrant));
         solePosition.get(robotQuadrant).changeFrame(yoSolePosition.get(robotQuadrant).getReferenceFrame());
         yoSolePosition.get(robotQuadrant).set(solePosition.get(robotQuadrant));
      }

      // update graphics
      yoGraphicsList.setVisible(false);
      if (contactForceIsVisible)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (contactState.get(robotQuadrant)[0])
            {
               yoContactForceSolutionViz.get(robotQuadrant).setVisible(true);
            }
         }
      }
   }

   public void setVisible(boolean visible)
   {
      setContactForceVisible(visible);
      yoGraphicsList.setVisible(false);
   }

   public void setContactForceVisible(boolean visible)
   {
      contactForceIsVisible = visible;
      yoGraphicsList.setVisible(false);
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
}