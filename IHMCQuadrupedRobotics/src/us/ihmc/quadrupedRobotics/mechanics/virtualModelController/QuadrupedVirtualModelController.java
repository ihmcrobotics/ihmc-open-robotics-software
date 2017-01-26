package us.ihmc.quadrupedRobotics.mechanics.virtualModelController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.kinematics.JointLimit;
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

public class QuadrupedVirtualModelController
{
   private final YoVariableRegistry registry;

   private final FullQuadrupedRobotModel fullRobotModel;
   private final QuadrupedReferenceFrames referenceFrames;
   private final ReferenceFrame worldFrame;
   private final QuadrantDependentList<ReferenceFrame> soleFrame;

   private final QuadrantDependentList<FrameVector> soleVirtualForce;
   private final QuadrantDependentList<FrameVector> soleContactForce;
   private final QuadrantDependentList<FramePoint> solePosition;
   private final QuadrantDependentList<FrameVector[]> jointTorques;
   private final QuadrantDependentList<YoFrameVector> yoSoleVirtualForce;
   private final QuadrantDependentList<YoFrameVector> yoSoleContactForce;
   private final QuadrantDependentList<YoFramePoint> yoSolePosition;
   private final QuadrantDependentList<YoFrameVector[]> yoJointTorques;

   private final QuadrantDependentList<OneDoFJoint[]> legJoints;
   private final LegJointName[] legJointNames;
   private final QuadrantDependentList<GeometricJacobian> footJacobian;
   private final QuadrantDependentList<PointJacobian> soleJacobian;
   private final QuadrantDependentList<DenseMatrix64F> legEffortVector;
   private final DenseMatrix64F virtualForceVector;

   private final YoGraphicsList yoGraphicsList;
   private final QuadrantDependentList<YoGraphicVector> yoSoleVirtualForceGraphic;
   private final QuadrantDependentList<YoFramePoint> yoSoleVirtualForceGraphicPosition;
   private final QuadrantDependentList<BooleanYoVariable> yoSoleVirtualForceGraphicVisible;
   private final QuadrantDependentList<YoGraphicVector> yoSoleContactForceGraphic;
   private final QuadrantDependentList<YoFramePoint> yoSoleContactForceGraphicPosition;
   private final QuadrantDependentList<BooleanYoVariable> yoSoleContactForceGraphicVisible;
   private final QuadrantDependentList<YoFramePoint[]> yoJointTorqueGraphicPositions;
   private final QuadrantDependentList<BooleanYoVariable> yoJointTorqueGraphicsVisible;
   private final QuadrantDependentList<YoGraphicVector[]> yoJointTorqueGraphics;
   private final FrameVector jointAxisTempVector = new FrameVector();

   public QuadrupedVirtualModelController(FullQuadrupedRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, double controlDT,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      legJointNames = fullRobotModel.getRobotSpecificJointNames().getLegJointNames();
      registry = new YoVariableRegistry(getClass().getSimpleName());

      // initialize reference frames
      this.referenceFrames = referenceFrames;
      worldFrame = ReferenceFrame.getWorldFrame();
      soleFrame = referenceFrames.getFootReferenceFrames();

      // initialize control variables
      solePosition = new QuadrantDependentList<>();
      soleVirtualForce = new QuadrantDependentList<>();
      soleContactForce = new QuadrantDependentList<>();
      jointTorques = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.set(robotQuadrant, new FramePoint(worldFrame));
         soleVirtualForce.set(robotQuadrant, new FrameVector(worldFrame));
         soleContactForce.set(robotQuadrant, new FrameVector(worldFrame));
         jointTorques.set(robotQuadrant, new FrameVector[legJointNames.length]);
         for (int i = 0; i < legJointNames.length; i++)
         {
            jointTorques.get(robotQuadrant)[i] = new FrameVector(worldFrame);
         }
      }

      // initialize yo variables
      yoSolePosition = new QuadrantDependentList<>();
      yoSoleVirtualForce = new QuadrantDependentList<>();
      yoSoleContactForce = new QuadrantDependentList<>();
      yoJointTorques = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSolePosition.set(robotQuadrant, new YoFramePoint(robotQuadrant.getCamelCaseName() + "SolePosition", worldFrame, registry));
         yoSoleVirtualForce.set(robotQuadrant, new YoFrameVector(robotQuadrant.getCamelCaseName() + "SoleVirtualForce", worldFrame, registry));
         yoSoleContactForce.set(robotQuadrant, new YoFrameVector(robotQuadrant.getCamelCaseName() + "SoleContactForce", worldFrame, registry));
         yoJointTorques.set(robotQuadrant, new YoFrameVector[legJointNames.length]);
         for (int i = 0; i < legJointNames.length; i++)
         {
            yoJointTorques.get(robotQuadrant)[i] = new YoFrameVector(robotQuadrant.getCamelCaseName() + legJointNames[i].getPascalCaseName() + "JointTorques",
                  worldFrame, registry);
         }
      }

      // initialize jacobian variables
      legJoints = new QuadrantDependentList<>();
      footJacobian = new QuadrantDependentList<>();
      soleJacobian = new QuadrantDependentList<>();
      legEffortVector = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointBeforeFoot(robotQuadrant);
         RigidBody body = fullRobotModel.getRootJoint().getSuccessor();
         RigidBody foot = jointBeforeFoot.getSuccessor();
         legJoints.set(robotQuadrant, ScrewTools.filterJoints(ScrewTools.createJointPath(body, foot), OneDoFJoint.class));
         footJacobian.set(robotQuadrant, new GeometricJacobian(legJoints.get(robotQuadrant), body.getBodyFixedFrame()));
         soleJacobian.set(robotQuadrant, new PointJacobian());
         legEffortVector.set(robotQuadrant, new DenseMatrix64F(legJoints.get(robotQuadrant).length, 1));
      }
      virtualForceVector = new DenseMatrix64F(3, 1);

      // initialize graphics
      yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      yoSoleVirtualForceGraphic = new QuadrantDependentList<>();
      yoSoleVirtualForceGraphicPosition = new QuadrantDependentList<>();
      yoSoleVirtualForceGraphicVisible = new QuadrantDependentList<>();
      yoSoleContactForceGraphic = new QuadrantDependentList<>();
      yoSoleContactForceGraphicPosition = new QuadrantDependentList<>();
      yoSoleContactForceGraphicVisible = new QuadrantDependentList<>();
      yoJointTorqueGraphicPositions = new QuadrantDependentList<>();
      yoJointTorqueGraphicsVisible = new QuadrantDependentList<>();
      yoJointTorqueGraphics = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         String prefix = parentRegistry.getName() + robotQuadrant.getPascalCaseName();
         yoSoleVirtualForceGraphicPosition.set(robotQuadrant, new YoFramePoint(prefix + "SoleVirtualForceGraphicPosition", worldFrame, registry));
         yoSoleVirtualForceGraphicVisible.set(robotQuadrant, new BooleanYoVariable(prefix + "SoleVirtualForceGraphicVisible", registry));
         yoSoleVirtualForceGraphic.set(robotQuadrant,
               new YoGraphicVector(prefix + "SoleVirtualForce", yoSoleVirtualForceGraphicPosition.get(robotQuadrant), yoSoleVirtualForce.get(robotQuadrant),
                     0.002, YoAppearance.Blue()));
         yoSoleContactForceGraphicPosition.set(robotQuadrant, new YoFramePoint(prefix + "SoleContactForceGraphicPosition", worldFrame, registry));
         yoSoleContactForceGraphicVisible.set(robotQuadrant, new BooleanYoVariable(prefix + "SoleContactForceGraphicVisible", registry));
         yoSoleContactForceGraphic.set(robotQuadrant,
               new YoGraphicVector(prefix + "SoleContactForce", yoSoleContactForceGraphicPosition.get(robotQuadrant), yoSoleContactForce.get(robotQuadrant),
                     0.002, YoAppearance.Chartreuse()));
         yoJointTorqueGraphicsVisible.set(robotQuadrant, new BooleanYoVariable(prefix + "JointTorqueGraphicVisible", registry));
         yoJointTorqueGraphics.set(robotQuadrant, new YoGraphicVector[legJointNames.length]);
         yoJointTorqueGraphicPositions.set(robotQuadrant, new YoFramePoint[legJointNames.length]);
         for (int index = 0; index < legJointNames.length; index++)
         {
            yoJointTorqueGraphicPositions.get(robotQuadrant)[index] = new YoFramePoint(
                  prefix + legJointNames[index].getPascalCaseName() + "JointTorqueGraphicPosition", worldFrame, registry);
            yoJointTorqueGraphics.get(robotQuadrant)[index] = new YoGraphicVector(prefix + legJointNames[index].getPascalCaseName() + "JointTorqueGraphic",
                  yoJointTorqueGraphicPositions.get(robotQuadrant)[index], yoJointTorques.get(robotQuadrant)[index], 0.010, YoAppearance.Red());
            yoGraphicsList.add(yoJointTorqueGraphics.get(robotQuadrant)[index]);
         }

         yoGraphicsList.add(yoSoleVirtualForceGraphic.get(robotQuadrant));
         yoGraphicsList.add(yoSoleContactForceGraphic.get(robotQuadrant));
      }

      graphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      parentRegistry.addChild(registry);
      this.reset();
   }

   public void reset()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleVirtualForce.get(robotQuadrant).setToZero();
         soleContactForce.get(robotQuadrant).setToZero();
      }
   }

   public void setSoleVirtualForce(RobotQuadrant robotQuadrant, FrameVector virtualForce)
   {
      soleVirtualForce.get(robotQuadrant).setIncludingFrame(virtualForce);

      // contact force is in the opposite direction
      soleContactForce.get(robotQuadrant).setIncludingFrame(virtualForce);
      soleContactForce.get(robotQuadrant).scale(-1.0);
   }

   public void setSoleContactForce(RobotQuadrant robotQuadrant, FrameVector contactForce)
   {
      soleContactForce.get(robotQuadrant).setIncludingFrame(contactForce);

      // virtual force is in the opposite direction
      soleVirtualForce.get(robotQuadrant).setIncludingFrame(contactForce);
      soleVirtualForce.get(robotQuadrant).scale(-1.0);
   }

   public void compute(QuadrupedVirtualModelControllerSettings settings)
   {
      // compute sole positions and jacobians
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         footJacobian.get(robotQuadrant).compute();
         soleJacobian.get(robotQuadrant).set(footJacobian.get(robotQuadrant), solePosition.get(robotQuadrant));
         soleJacobian.get(robotQuadrant).compute();
      }

      // compute joint torques using jacobian transpose
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         DenseMatrix64F jacobianMatrix = soleJacobian.get(robotQuadrant).getJacobianMatrix();
         ReferenceFrame jacobianFrame = soleJacobian.get(robotQuadrant).getFrame();
         soleVirtualForce.get(robotQuadrant).changeFrame(jacobianFrame);
         virtualForceVector.set(0, 0, soleVirtualForce.get(robotQuadrant).getX());
         virtualForceVector.set(1, 0, soleVirtualForce.get(robotQuadrant).getY());
         virtualForceVector.set(2, 0, soleVirtualForce.get(robotQuadrant).getZ());
         CommonOps.multTransA(jacobianMatrix, virtualForceVector, legEffortVector.get(robotQuadrant));

         int index = 0;
         for (OneDoFJoint joint : legJoints.get(robotQuadrant))
         {
            QuadrupedJointName jointName = fullRobotModel.getNameForOneDoFJoint(joint);
            JointLimit jointLimit = fullRobotModel.getJointLimit(jointName);

            // compute desired joint torque with position and torque limits
            double tau = legEffortVector.get(robotQuadrant).get(index, 0);
            if (joint.getQ() < jointLimit.getSoftLowerPositionLimit())
               tau = Math.max(tau, settings.getJointPositionLimitStiffness(jointName) * (jointLimit.getSoftLowerPositionLimit() - joint.getQ())
                     - settings.getJointPositionLimitDamping(jointName) * joint.getQd());
            if (joint.getQ() > jointLimit.getSoftUpperPositionLimit())
               tau = Math.min(tau, settings.getJointPositionLimitStiffness(jointName) * (jointLimit.getSoftUpperPositionLimit() - joint.getQ())
                     - settings.getJointPositionLimitDamping(jointName) * joint.getQd());
            tau = Math.min(Math.max(tau, -jointLimit.getTorqueLimit()), jointLimit.getTorqueLimit());

            // compute joint damping
            tau = tau - settings.getJointDamping(jointName) * joint.getQd();

            // update joint torques in full robot model
            joint.setTau(tau);

            // update joint torque vectors
            jointTorques.get(robotQuadrant)[index].setToZero(joint.getFrameBeforeJoint());
            joint.getJointAxis(jointAxisTempVector);
            double x = tau * jointAxisTempVector.getX();
            double y = tau * jointAxisTempVector.getY();
            double z = tau * jointAxisTempVector.getZ();
            jointTorques.get(robotQuadrant)[index].set(x, y, z);
            jointTorques.get(robotQuadrant)[index].changeFrame(worldFrame);

            index++;
         }
      }

      // update yo variables
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSoleVirtualForce.get(robotQuadrant).setAndMatchFrame(soleVirtualForce.get(robotQuadrant));
         yoSoleContactForce.get(robotQuadrant).setAndMatchFrame(soleContactForce.get(robotQuadrant));
         yoSolePosition.get(robotQuadrant).setAndMatchFrame(solePosition.get(robotQuadrant));
         for (int i = 0; i < legJointNames.length; i++)
         {
            yoJointTorques.get(robotQuadrant)[i].setWithoutChecks(jointTorques.get(robotQuadrant)[i]);
         }
      }

      // update yo graphics
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         // move graphics off screen if visible is set to false
         if (yoSoleVirtualForceGraphicVisible.get(robotQuadrant).getBooleanValue())
            yoSoleVirtualForceGraphicPosition.get(robotQuadrant).setAndMatchFrame(solePosition.get(robotQuadrant));
         else
            yoSoleVirtualForceGraphicPosition.get(robotQuadrant).set(1E6, 1E6, 1E6);

         if (yoSoleContactForceGraphicVisible.get(robotQuadrant).getBooleanValue())
            yoSoleContactForceGraphicPosition.get(robotQuadrant).setAndMatchFrame(solePosition.get(robotQuadrant));
         else
            yoSoleContactForceGraphicPosition.get(robotQuadrant).set(1E6, 1E6, 1E6);
         for (int i = 0; i < legJointNames.length; i++)
         {
            if (yoJointTorqueGraphicsVisible.get(robotQuadrant).getBooleanValue())
               yoJointTorqueGraphicPositions.get(robotQuadrant)[i].setFromReferenceFrame(referenceFrames.getLegJointFrame(robotQuadrant, legJointNames[i]));
            else
               yoJointTorqueGraphicPositions.get(robotQuadrant)[i].set(1E6, 1E6, 1E6);
         }
      }
   }

   public void setVisible(boolean visible)
   {
      yoGraphicsList.setVisible(visible);
   }

   public void setSoleVirtualForceVisible(RobotQuadrant robotQuadrant, boolean visible)
   {
      yoSoleVirtualForceGraphicVisible.get(robotQuadrant).set(visible);
   }

   public void setSoleContactForceVisible(RobotQuadrant robotQuadrant, boolean visible)
   {
      yoSoleContactForceGraphicVisible.get(robotQuadrant).set(visible);
   }

   public void setJointTorquesVisible(RobotQuadrant robotQuadrant, boolean visible)
   {
      yoJointTorqueGraphicsVisible.get(robotQuadrant).set(visible);
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
}
