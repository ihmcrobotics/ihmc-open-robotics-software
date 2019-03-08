package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointLimitEnforcementCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotModels.JointHashCodeResolver;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

/**
 * The objective of this class is to help the passing commands between two instances of the same
 * robot.
 * <p>
 * The main use-case is for passing commands from one thread to another. In such context, each
 * thread has its own instance of the robot and the corresponding reference frame tree.
 * </p>
 * <p>
 * The main challenge when passing commands is to retrieve the joints, rigid-bodies, and reference
 * frames properly.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class CrossRobotCommandResolver
{
   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;
   private final RigidBodyHashCodeResolver rigidBodyHashCodeResolver;
   private final JointHashCodeResolver jointHashCodeResolver;

   public CrossRobotCommandResolver(ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver, RigidBodyHashCodeResolver rigidBodyHashCodeResolver,
                                    JointHashCodeResolver jointHashCodeResolver)
   {
      this.referenceFrameHashCodeResolver = referenceFrameHashCodeResolver;
      this.rigidBodyHashCodeResolver = rigidBodyHashCodeResolver;
      this.jointHashCodeResolver = jointHashCodeResolver;
   }

   public void resolveCenterOfPressureCommand(CenterOfPressureCommand in, CenterOfPressureCommand out)
   {
      out.setConstraintType(in.getConstraintType());
      out.setContactingRigidBody(resolveRigidBody(in.getContactingRigidBody()));
      resolveFrameTuple2D(in.getWeight(), out.getWeight());
      resolveFrameTuple2D(in.getDesiredCoP(), out.getDesiredCoP());
   }

   public void resolveContactWrenchCommand(ContactWrenchCommand in, ContactWrenchCommand out)
   {
      out.setConstraintType(in.getConstraintType());
      out.setRigidBody(resolveRigidBody(in.getRigidBody()));
      resolveWrench(in.getWrench(), out.getWrench());
      resolveWeightMatrix6D(in.getWeightMatrix(), out.getWeightMatrix());
      resolveSelectionMatrix6D(in.getSelectionMatrix(), out.getSelectionMatrix());
   }

   public void resolveExternalWrenchCommand(ExternalWrenchCommand in, ExternalWrenchCommand out)
   {
      out.setRigidBody(resolveRigidBody(in.getRigidBody()));
      resolveWrench(in.getExternalWrench(), out.getExternalWrench());
   }

   public void resolveInverseDynamicsOptimizationSettingsCommand(InverseDynamicsOptimizationSettingsCommand in, InverseDynamicsOptimizationSettingsCommand out)
   {
      // There is no robot sensitive information in this command, so the output can directly be set to the input.
      out.set(in);
   }

   public void resolveJointAccelerationIntegrationCommand(JointAccelerationIntegrationCommand in, JointAccelerationIntegrationCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJointsToComputeDesiredPositionFor(); jointIndex++)
      {
         out.addJointToComputeDesiredPositionFor(resolveJoint(in.getJointToComputeDesiredPositionFor(jointIndex)));
         // There is no robot sensitive information in this command, so the output can directly be set to the input.
         out.setJointParameters(jointIndex, in.getJointParameters(jointIndex));
      }
   }

   public void resolveJointLimitEnforcementMethodCommand(JointLimitEnforcementMethodCommand in, JointLimitEnforcementMethodCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         OneDoFJointBasics joint = resolveJoint(in.getJoint(jointIndex));
         JointLimitParameters parameters = in.getJointLimitParameters(jointIndex);
         JointLimitEnforcement method = in.getJointLimitReductionFactor(jointIndex);
         out.addLimitEnforcementMethod(joint, method, parameters);
      }
   }

   public void resolveJointspaceAccelerationCommand(JointspaceAccelerationCommand in, JointspaceAccelerationCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         out.addJoint(resolveJoint(in.getJoint(jointIndex)), in.getDesiredAcceleration(jointIndex), in.getWeight(jointIndex));
      }
   }

   public void resolveMomentumRateCommand(MomentumRateCommand in, MomentumRateCommand out)

   {
      out.setMomentumRate(in.getMomentumRate());
      resolveWeightMatrix6D(in.getWeightMatrix(), out.getWeightMatrix());
      resolveSelectionMatrix6D(in.getSelectionMatrix(), out.getSelectionMatrix());
   }

   public void resolvePlaneContactStateCommand(PlaneContactStateCommand in, PlaneContactStateCommand out)
   {
      out.clearContactPoints();
      out.setContactingRigidBody(resolveRigidBody(in.getContactingRigidBody()));
      out.setCoefficientOfFriction(in.getCoefficientOfFriction());
      out.setUseHighCoPDamping(in.isUseHighCoPDamping());
      out.setHasContactStateChanged(in.getHasContactStateChanged());
      resolveFrameTuple3D(in.getContactNormal(), out.getContactNormal());
      out.getContactFramePoseInBodyFixedFrame().set(in.getContactFramePoseInBodyFixedFrame());

      for (int contactPointIndex = 0; contactPointIndex < in.getNumberOfContactPoints(); contactPointIndex++)
      {
         resolveFrameTuple3D(in.getContactPoint(contactPointIndex), out.addPointInContact());
         out.setMaxContactPointNormalForce(contactPointIndex, in.getMaxContactPointNormalForce(contactPointIndex));
         out.setRhoWeight(contactPointIndex, in.getRhoWeight(contactPointIndex));
      }
   }

   public void resolveSpatialAccelerationCommand(SpatialAccelerationCommand in, SpatialAccelerationCommand out)
   {
      resolveFramePose3D(in.getControlFramePose(), out.getControlFramePose());
      out.getDesiredLinearAcceleration().set(in.getDesiredLinearAcceleration());
      out.getDesiredAngularAcceleration().set(in.getDesiredAngularAcceleration());
      resolveWeightMatrix6D(in.getWeightMatrix(), out.getWeightMatrix());
      resolveSelectionMatrix6D(in.getSelectionMatrix(), out.getSelectionMatrix());
      out.set(resolveRigidBody(in.getBase()), resolveRigidBody(in.getEndEffector()));
      out.setPrimaryBase(resolveRigidBody(in.getPrimaryBase()));
      out.setScaleSecondaryTaskJointWeight(in.scaleSecondaryTaskJointWeight(), in.getSecondaryTaskJointWeightScale());
   }

   public void resolveInverseKinematicsOptimizationSettingsCommand(InverseKinematicsOptimizationSettingsCommand in,
                                                                   InverseKinematicsOptimizationSettingsCommand out)
   {
      // There is no robot sensitive information in this command, so the output can directly be set to the input.
      out.set(in);
   }

   public void resolveJointLimitReductionCommand(JointLimitReductionCommand in, JointLimitReductionCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         out.addReductionFactor(resolveJoint(in.getJoint(jointIndex)), in.getJointLimitReductionFactor(jointIndex));
      }
   }

   public void resolveJointspaceVelocityCommand(JointspaceVelocityCommand in, JointspaceVelocityCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         out.addJoint(resolveJoint(in.getJoint(jointIndex)), in.getDesiredVelocity(jointIndex), in.getWeight(jointIndex));
      }
   }

   public void resolveMomentumCommand(MomentumCommand in, MomentumCommand out)
   {
      out.setMomentum(in.getMomentum());
      resolveWeightMatrix6D(in.getWeightMatrix(), out.getWeightMatrix());
      resolveSelectionMatrix6D(in.getSelectionMatrix(), out.getSelectionMatrix());
   }

   public void resolvePrivilegedConfigurationCommand(PrivilegedConfigurationCommand in, PrivilegedConfigurationCommand out)
   {
      out.clear();
      out.setDefaultParameters(in.getDefaultParameters());

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         out.addJoint(resolveJoint(in.getJoint(jointIndex)), in.getJointSpecificParameters(jointIndex));
      }

      if (in.isEnabled())
         out.enable();
      else
         out.disable();
   }

   public void resolvePrivilegedJointSpaceCommand(PrivilegedJointSpaceCommand in, PrivilegedJointSpaceCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         out.addJoint(resolveJoint(in.getJoint(jointIndex)), in.getPrivilegedCommand(jointIndex));
         out.setWeight(jointIndex, in.getWeight(jointIndex));
      }

      if (in.isEnabled())
         out.enable();
      else
         out.disable();
   }

   public void resolveJointLimitEnforcementCommand(JointLimitEnforcementCommand in, JointLimitEnforcementCommand out)
   {
      out.clear();

      for (int jointIndex = 0; jointIndex < in.getNumberOfJoints(); jointIndex++)
      {
         out.addJoint(resolveJoint(in.getJoint(jointIndex)), in.getJointLimitData(jointIndex));
      }
   }

   public void resolveSpatialVelocityCommand(SpatialVelocityCommand in, SpatialVelocityCommand out)
   {
      resolveFramePose3D(in.getControlFramePose(), out.getControlFramePose());
      out.getDesiredLinearVelocity().set(in.getDesiredLinearVelocity());
      out.getDesiredAngularVelocity().set(in.getDesiredAngularVelocity());
      resolveWeightMatrix6D(in.getWeightMatrix(), out.getWeightMatrix());
      resolveSelectionMatrix6D(in.getSelectionMatrix(), out.getSelectionMatrix());
      out.set(resolveRigidBody(in.getBase()), resolveRigidBody(in.getEndEffector()));
      out.setPrimaryBase(resolveRigidBody(in.getPrimaryBase()));
      out.setScaleSecondaryTaskJointWeight(in.scaleSecondaryTaskJointWeight(), in.getSecondaryTaskJointWeightScale());
   }

   public void resolveWrench(WrenchReadOnly in, WrenchBasics out)
   {
      out.setIncludingFrame(in);
      out.setReferenceFrame(resolveReferenceFrame(in.getReferenceFrame()));
      out.setBodyFrame(resolveReferenceFrame(in.getBodyFrame()));
   }

   public void resolveSelectionMatrix3D(SelectionMatrix3D in, SelectionMatrix3D out)
   {
      out.set(in);
      out.setSelectionFrame(resolveReferenceFrame(in.getSelectionFrame()));
   }

   public void resolveSelectionMatrix6D(SelectionMatrix6D in, SelectionMatrix6D out)
   {
      resolveSelectionMatrix3D(in.getAngularPart(), out.getAngularPart());
      resolveSelectionMatrix3D(in.getLinearPart(), out.getLinearPart());
   }

   public void resolveWeightMatrix3D(WeightMatrix3D in, WeightMatrix3D out)
   {
      out.set(in);
      out.setWeightFrame(resolveReferenceFrame(in.getWeightFrame()));
   }

   public void resolveWeightMatrix6D(WeightMatrix6D in, WeightMatrix6D out)
   {
      resolveWeightMatrix3D(in.getAngularPart(), out.getAngularPart());
      resolveWeightMatrix3D(in.getLinearPart(), out.getLinearPart());
   }

   public void resolveFrameTuple2D(FrameTuple2DReadOnly in, FrameTuple2DBasics out)
   {
      out.setIncludingFrame(resolveReferenceFrame(in.getReferenceFrame()), in);
   }

   public void resolveFrameTuple3D(FrameTuple3DReadOnly in, FrameTuple3DBasics out)
   {
      out.setIncludingFrame(resolveReferenceFrame(in.getReferenceFrame()), in);
   }

   public void resolveFramePose3D(FramePose3DReadOnly in, FramePose3DBasics out)
   {
      out.setIncludingFrame(resolveReferenceFrame(in.getReferenceFrame()), in);
   }

   private ReferenceFrame resolveReferenceFrame(ReferenceFrame in)
   {
      if (in == null)
         return null;
      else
         return referenceFrameHashCodeResolver.getReferenceFrame(in.hashCode());
   }

   private <B extends RigidBodyReadOnly> B resolveRigidBody(B in)
   {
      if (in == null)
         return null;
      else
         return rigidBodyHashCodeResolver.castAndGetRigidBody(in.hashCode());
   }

   private <J extends JointReadOnly> J resolveJoint(J in)
   {
      if (in == null)
         return null;
      else
         return jointHashCodeResolver.castAndGetJoint(in.hashCode());
   }
}
