package us.ihmc.aware.vmc;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
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
import us.ihmc.quadrupedRobotics.virtualModelController.QuadrupedJointLimits;

public class QuadrupedVirtualModelController
{
   private final YoVariableRegistry registry;
   private final QuadrupedJointNameMap jointNameMap;

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
   private final DenseMatrix64F virtualForceVector;

   private final YoGraphicsList yoGraphicsList;
   private final QuadrantDependentList<YoGraphicVector> yoSoleVirtualForceViz;
   private final QuadrantDependentList<YoGraphicVector> yoSoleContactForceViz;

   public QuadrupedVirtualModelController(SDFFullRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, QuadrupedJointNameMap jointNameMap,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.jointNameMap = jointNameMap;
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
         yoSoleVirtualForce.set(robotQuadrant,
               new YoFrameVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SoleVirtualForce", worldFrame, registry));
         yoSoleContactForce.set(robotQuadrant,
               new YoFrameVector(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SoleContactForce", worldFrame, registry));
      }

      // initialize jacobian variables
      legJoints = new QuadrantDependentList<>();
      footJacobian = new QuadrantDependentList<>();
      soleJacobian = new QuadrantDependentList<>();
      legEffortVector = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String jointBeforeFootName = jointNameMap.getJointBeforeFootName(robotQuadrant);
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
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
      yoSoleVirtualForceViz = new QuadrantDependentList<>();
      yoSoleContactForceViz = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         String prefix = parentRegistry.getName();
         yoSoleVirtualForceViz.set(robotQuadrant,
               new YoGraphicVector(prefix + robotQuadrant.getCamelCaseNameForMiddleOfExpression() + "SoleVirtualForce", yoSolePosition.get(robotQuadrant),
                     yoSoleVirtualForce.get(robotQuadrant), 0.002, YoAppearance.OrangeRed()));
         yoSoleContactForceViz.set(robotQuadrant,
               new YoGraphicVector(prefix + robotQuadrant.getCamelCaseNameForMiddleOfExpression() + "SoleContactForce", yoSolePosition.get(robotQuadrant),
                     yoSoleContactForce.get(robotQuadrant), 0.002, YoAppearance.Chartreuse()));
         yoGraphicsList.add(yoSoleVirtualForceViz.get(robotQuadrant));
         yoGraphicsList.add(yoSoleContactForceViz.get(robotQuadrant));
      }
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

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

   public void compute(QuadrupedJointLimits jointLimits, QuadrupedVirtualModelControllerSettings controllerSettings)
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
            QuadrupedJointName jointName = jointNameMap.getJointNameForSDFName(joint.getName());

            // compute joint torque and with damping
            double tau = legEffortVector.get(robotQuadrant).get(index, 0) - controllerSettings.getJointDamping(jointName) * joint.getQd();

            // apply position and torque limits
            double tauPositionLowerLimit =
                  controllerSettings.getJointPositionLimitStiffness(jointName) * (jointLimits.getSoftPositionLowerLimit(jointName) - joint.getQ())
                        - controllerSettings.getJointPositionLimitDamping(jointName) * joint.getQd();
            double tauPositionUpperLimit =
                  controllerSettings.getJointPositionLimitStiffness(jointName) * (jointLimits.getSoftPositionUpperLimit(jointName) - joint.getQ())
                        - controllerSettings.getJointPositionLimitDamping(jointName) * joint.getQd();
            double tauEffortLowerLimit = -jointLimits.getEffortLimit(jointName);
            double tauEffortUpperLimit = jointLimits.getEffortLimit(jointName);
            tau = Math.min(Math.max(tau, tauPositionLowerLimit), tauPositionUpperLimit);
            tau = Math.min(Math.max(tau, tauEffortLowerLimit), tauEffortUpperLimit);

            // update joint torques in full robot model
            joint.setTau(tau);
            index++;
         }
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleVirtualForce.get(robotQuadrant).changeFrame(yoSoleVirtualForce.get(robotQuadrant).getReferenceFrame());
         yoSoleVirtualForce.get(robotQuadrant).set(soleVirtualForce.get(robotQuadrant));
         soleContactForce.get(robotQuadrant).changeFrame(yoSoleContactForce.get(robotQuadrant).getReferenceFrame());
         yoSoleContactForce.get(robotQuadrant).set(soleContactForce.get(robotQuadrant));
         solePosition.get(robotQuadrant).changeFrame(yoSolePosition.get(robotQuadrant).getReferenceFrame());
         yoSolePosition.get(robotQuadrant).set(solePosition.get(robotQuadrant));
      }
   }

   public void setVisible(boolean visible)
   {
      if (visible == false)
      {
         yoGraphicsList.setVisible(false);
      }
   }

   public void setSoleVirtualForceVisible(RobotQuadrant robotQuadrant, boolean visible)
   {
      yoSoleVirtualForceViz.get(robotQuadrant).setVisible(visible);
   }

   public void setSoleContactForceVisible(RobotQuadrant robotQuadrant, boolean visible)
   {
      yoSoleContactForceViz.get(robotQuadrant).setVisible(visible);
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
}
