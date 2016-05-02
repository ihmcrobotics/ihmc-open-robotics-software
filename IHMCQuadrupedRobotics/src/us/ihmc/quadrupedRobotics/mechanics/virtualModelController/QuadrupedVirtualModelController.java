package us.ihmc.quadrupedRobotics.mechanics.virtualModelController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.quadrupedRobotics.util.LowPassFilter;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.SdfLoader.partNames.QuadrupedJointName;
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
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedVirtualModelController
{
   private final YoVariableRegistry registry;

   private final SDFFullQuadrupedRobotModel fullRobotModel;
   private final QuadrupedReferenceFrames referenceFrames;
   private final ReferenceFrame worldFrame;
   private final QuadrantDependentList<ReferenceFrame> soleFrame;

   private final QuadrantDependentList<FrameVector> soleVirtualForce;
   private final QuadrantDependentList<FrameVector> soleContactForce;
   private final QuadrantDependentList<FramePoint> solePosition;
   private final QuadrantDependentList<YoFrameVector> yoSoleVirtualForce;
   private final QuadrantDependentList<YoFrameVector> yoSoleContactForce;
   private final QuadrantDependentList<YoFramePoint> yoSolePosition;

   private final QuadrantDependentList<OneDoFJoint[]> legJoints;
   private final QuadrantDependentList<GeometricJacobian> footJacobian;
   private final QuadrantDependentList<PointJacobian> soleJacobian;
   private final QuadrantDependentList<DenseMatrix64F> legEffortVector;
   private final QuadrantDependentList<LowPassFilter[]> legEffortFilter;
   private final DenseMatrix64F virtualForceVector;

   private final YoGraphicsList yoGraphicsList;
   private final QuadrantDependentList<YoGraphicVector> yoSoleVirtualForceGraphic;
   private final QuadrantDependentList<YoFramePoint> yoSoleVirtualForceGraphicPosition;
   private final QuadrantDependentList<BooleanYoVariable> yoSoleVirtualForceGraphicVisible;
   private final QuadrantDependentList<YoGraphicVector> yoSoleContactForceGraphic;
   private final QuadrantDependentList<YoFramePoint> yoSoleContactForceGraphicPosition;
   private final QuadrantDependentList<BooleanYoVariable> yoSoleContactForceGraphicVisible;

   public QuadrupedVirtualModelController(SDFFullQuadrupedRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      registry = new YoVariableRegistry(getClass().getSimpleName());

      // initialize reference frames
      this.referenceFrames = referenceFrames;
      worldFrame = ReferenceFrame.getWorldFrame();
      soleFrame = referenceFrames.getFootReferenceFrames();

      // initialize control variables
      solePosition = new QuadrantDependentList<>();
      soleVirtualForce = new QuadrantDependentList<>();
      soleContactForce = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.set(robotQuadrant, new FramePoint(worldFrame));
         soleVirtualForce.set(robotQuadrant, new FrameVector(worldFrame));
         soleContactForce.set(robotQuadrant, new FrameVector(worldFrame));
      }

      // initialize yo variables
      yoSolePosition = new QuadrantDependentList<>();
      yoSoleVirtualForce = new QuadrantDependentList<>();
      yoSoleContactForce = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSolePosition.set(robotQuadrant, new YoFramePoint(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SolePosition", worldFrame, registry));
         yoSoleVirtualForce
               .set(robotQuadrant, new YoFrameVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SoleVirtualForce", worldFrame, registry));
         yoSoleContactForce
               .set(robotQuadrant, new YoFrameVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SoleContactForce", worldFrame, registry));
      }

      // initialize jacobian variables
      legJoints = new QuadrantDependentList<>();
      footJacobian = new QuadrantDependentList<>();
      soleJacobian = new QuadrantDependentList<>();
      legEffortVector = new QuadrantDependentList<>();
      legEffortFilter = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointBeforeFoot(robotQuadrant);
         RigidBody body = fullRobotModel.getRootJoint().getSuccessor();
         RigidBody foot = jointBeforeFoot.getSuccessor();
         legJoints.set(robotQuadrant, ScrewTools.filterJoints(ScrewTools.createJointPath(body, foot), OneDoFJoint.class));
         footJacobian.set(robotQuadrant, new GeometricJacobian(legJoints.get(robotQuadrant), body.getBodyFixedFrame()));
         soleJacobian.set(robotQuadrant, new PointJacobian());
         legEffortVector.set(robotQuadrant, new DenseMatrix64F(legJoints.get(robotQuadrant).length, 1));
         legEffortFilter.set(robotQuadrant, new LowPassFilter[legJoints.get(robotQuadrant).length]);
         for (int i = 0; i < legJoints.get(robotQuadrant).length; i++)
         {
            legEffortFilter.get(robotQuadrant)[i] = new LowPassFilter(controlDT, 0.5 / controlDT);
         }
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
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         String prefix = parentRegistry.getName() + robotQuadrant.getCamelCaseNameForMiddleOfExpression();
         yoSoleVirtualForceGraphicPosition.set(robotQuadrant, new YoFramePoint(prefix + "SoleVirtualForceGraphicPosition", worldFrame, registry));
         yoSoleVirtualForceGraphicVisible.set(robotQuadrant, new BooleanYoVariable(prefix + "SoleVirtualForceGraphicVisible", registry));
         yoSoleVirtualForceGraphic.set(robotQuadrant, new YoGraphicVector(prefix + "SoleVirtualForce", yoSoleVirtualForceGraphicPosition.get(robotQuadrant), yoSoleVirtualForce.get(robotQuadrant), 0.002, YoAppearance.Blue()));
         yoSoleContactForceGraphicPosition.set(robotQuadrant, new YoFramePoint(prefix + "SoleContactForceGraphicPosition", worldFrame, registry));
         yoSoleContactForceGraphicVisible.set(robotQuadrant, new BooleanYoVariable(prefix + "SoleContactForceGraphicVisible", registry));
         yoSoleContactForceGraphic.set(robotQuadrant, new YoGraphicVector(prefix + "SoleContactForce", yoSoleContactForceGraphicPosition.get(robotQuadrant), yoSoleContactForce.get(robotQuadrant), 0.002, YoAppearance.Chartreuse()));
         yoGraphicsList.add(yoSoleVirtualForceGraphic.get(robotQuadrant));
         yoGraphicsList.add(yoSoleContactForceGraphic.get(robotQuadrant));
      }

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

   public void registerGraphics(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
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
            double tauPositionLowerLimit = settings.getJointPositionLimitStiffness(jointName) * (jointLimit.getSoftLowerPositionLimit() - joint.getQ()) - settings.getJointPositionLimitDamping(jointName) * joint.getQd();
            double tauPositionUpperLimit = settings.getJointPositionLimitStiffness(jointName) * (jointLimit.getSoftUpperPositionLimit() - joint.getQ()) - settings.getJointPositionLimitDamping(jointName) * joint.getQd();
            double tauEffortLowerLimit = -jointLimit.getTorqueLimit();
            double tauEffortUpperLimit = jointLimit.getTorqueLimit();
            tau = Math.min(Math.max(tau, tauPositionLowerLimit), tauPositionUpperLimit);
            tau = Math.min(Math.max(tau, tauEffortLowerLimit), tauEffortUpperLimit);

            // filter desired joint torque and add damping
            legEffortFilter.get(robotQuadrant)[index].setBreakFrequency(settings.getJointEffortBreakFrequency(jointName));
            tau = legEffortFilter.get(robotQuadrant)[index].compute(tau) - settings.getJointDamping(jointName) * joint.getQd();

            // update joint torques in full robot model
            joint.setTau(tau);
            index++;
         }
      }

      // update yo variables
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSoleVirtualForce.get(robotQuadrant).setAndMatchFrame(soleVirtualForce.get(robotQuadrant));
         yoSoleContactForce.get(robotQuadrant).setAndMatchFrame(soleContactForce.get(robotQuadrant));
         yoSolePosition.get(robotQuadrant).setAndMatchFrame(solePosition.get(robotQuadrant));
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

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
}
