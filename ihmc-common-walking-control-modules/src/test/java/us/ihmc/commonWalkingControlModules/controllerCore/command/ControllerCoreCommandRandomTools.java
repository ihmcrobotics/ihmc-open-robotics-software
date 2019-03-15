package us.ihmc.commonWalkingControlModules.controllerCore.command;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import org.apache.commons.lang3.mutable.MutableInt;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;
import org.reflections.Reflections;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointLimitEnforcementCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualEffortCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.kinematics.JointLimitData;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;

public class ControllerCoreCommandRandomTools
{
   public static final String CONTROLLER_CORE_COMMANDS_PACKAGE = "us.ihmc.commonWalkingControlModules.controllerCore.command";

   @SafeVarargs
   public static <E> E nextElementIn(Random random, E... elements)
   {
      return elements[random.nextInt(elements.length)];
   }

   public static <E> E nextElementIn(Random random, List<E> list)
   {
      return list.get(random.nextInt(list.size()));
   }

   public static FramePoint2D nextFramePoint2D(Random random, ReferenceFrame... possibleFrames)
   {
      return EuclidFrameRandomTools.nextFramePoint2D(random, nextElementIn(random, possibleFrames));
   }

   public static FramePoint3D nextFramePoint3D(Random random, ReferenceFrame... possibleFrames)
   {
      return EuclidFrameRandomTools.nextFramePoint3D(random, nextElementIn(random, possibleFrames));
   }

   public static FrameVector2D nextFrameVector2D(Random random, ReferenceFrame... possibleFrames)
   {
      return EuclidFrameRandomTools.nextFrameVector2D(random, nextElementIn(random, possibleFrames));
   }

   public static FrameVector3D nextFrameVector3D(Random random, ReferenceFrame... possibleFrames)
   {
      return EuclidFrameRandomTools.nextFrameVector3D(random, nextElementIn(random, possibleFrames));
   }

   public static FrameQuaternion nextFrameQuaternion(Random random, ReferenceFrame... possibleFrames)
   {
      return EuclidFrameRandomTools.nextFrameQuaternion(random, nextElementIn(random, possibleFrames));
   }

   public static FramePose3D nextFramePose3D(Random random, ReferenceFrame... possibleFrames)
   {
      return EuclidFrameRandomTools.nextFramePose3D(random, nextElementIn(random, possibleFrames));
   }

   public static Wrench nextWrench(Random random, ReferenceFrame... possibleFrames)
   {
      return MecanoRandomTools.nextWrench(random, nextElementIn(random, possibleFrames), nextElementIn(random, possibleFrames));
   }

   public static SpatialAcceleration nextSpatialAcceleration(Random random, ReferenceFrame... possibleFrames)
   {
      return MecanoRandomTools.nextSpatialAcceleration(random, nextElementIn(random, possibleFrames), nextElementIn(random, possibleFrames),
                                                       nextElementIn(random, possibleFrames));
   }

   public static WeightMatrix3D nextWeightMatrix3D(Random random, ReferenceFrame... possibleFrames)
   {
      WeightMatrix3D next = new WeightMatrix3D();
      next.setWeights(random.nextDouble(), random.nextDouble(), random.nextDouble());
      next.setWeightFrame(nextElementIn(random, possibleFrames));
      return next;
   }

   public static WeightMatrix6D nextWeightMatrix6D(Random random, ReferenceFrame... possibleFrames)
   {
      WeightMatrix6D next = new WeightMatrix6D();
      next.setAngularPart(nextWeightMatrix3D(random, possibleFrames));
      next.setLinearPart(nextWeightMatrix3D(random, possibleFrames));
      return next;
   }

   public static SelectionMatrix3D nextSelectionMatrix3D(Random random, ReferenceFrame... possibleFrames)
   {
      SelectionMatrix3D next = new SelectionMatrix3D();
      next.setAxisSelection(random.nextBoolean(), random.nextBoolean(), random.nextBoolean());
      next.setSelectionFrame(nextElementIn(random, possibleFrames));
      return next;
   }

   public static SelectionMatrix6D nextSelectionMatrix6D(Random random, ReferenceFrame... possibleFrames)
   {
      SelectionMatrix6D next = new SelectionMatrix6D();
      next.setAngularPart(nextSelectionMatrix3D(random, possibleFrames));
      next.setLinearPart(nextSelectionMatrix3D(random, possibleFrames));
      return next;
   }

   public static JointAccelerationIntegrationParameters nextJointAccelerationIntegrationParameters(Random random)
   {
      JointAccelerationIntegrationParameters next = new JointAccelerationIntegrationParameters();
      next.setPositionBreakFrequency(random.nextDouble());
      next.setVelocityBreakFrequency(random.nextDouble());
      next.setMaxPositionError(random.nextDouble());
      next.setMaxVelocity(random.nextDouble());
      return next;
   }

   public static JointLimitParameters nextJointLimitParameters(Random random)
   {
      JointLimitParameters next = new JointLimitParameters();
      next.setMaxAbsJointVelocity(random.nextDouble());
      next.setJointLimitDistanceForMaxVelocity(random.nextDouble());
      next.setJointLimitFilterBreakFrequency(random.nextDouble());
      next.setVelocityControlGain(random.nextDouble());
      return next;
   }

   public static OneDoFJointPrivilegedConfigurationParameters nextOneDoFJointPrivilegedConfigurationParameters(Random random)
   {
      OneDoFJointPrivilegedConfigurationParameters next = new OneDoFJointPrivilegedConfigurationParameters();
      if (random.nextBoolean())
         next.setPrivilegedConfiguration(random.nextDouble());
      if (random.nextBoolean())
         next.setPrivilegedConfigurationOption(nextElementIn(random, PrivilegedConfigurationOption.values()));
      if (random.nextBoolean())
         next.setWeight(random.nextDouble());
      if (random.nextBoolean())
         next.setConfigurationGain(random.nextDouble());
      if (random.nextBoolean())
         next.setVelocityGain(random.nextDouble());
      if (random.nextBoolean())
         next.setMaxVelocity(random.nextDouble());
      if (random.nextBoolean())
         next.setMaxAcceleration(random.nextDouble());
      return next;
   }

   public static JointLimitData nextJointLimitData(Random random)
   {
      JointLimitData next = new JointLimitData();
      next.setPositionSoftLowerLimit(random.nextDouble());
      next.setPositionSoftUpperLimit(random.nextDouble());
      next.setVelocityLowerLimit(random.nextDouble());
      next.setVelocityUpperLimit(random.nextDouble());
      next.setTorqueLowerLimit(random.nextDouble());
      next.setTorqueUpperLimit(random.nextDouble());
      next.setPositionLimitStiffness(random.nextDouble());
      next.setPositionLimitDamping(random.nextDouble());
      return next;
   }

   public static PDGains nextPDGains(Random random)
   {
      PDGains next = new PDGains();
      next.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
      return next;
   }

   public static PID3DGains nextPID3DGains(Random random)
   {
      PID3DGains next = new DefaultPID3DGains();

      next.setProportionalGains(random.nextDouble(), random.nextDouble(), random.nextDouble());
      next.setDerivativeGains(random.nextDouble(), random.nextDouble(), random.nextDouble());
      next.setIntegralGains(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
      next.setMaxDerivativeError(random.nextDouble());
      next.setMaxFeedbackAndFeedbackRate(random.nextDouble(), random.nextDouble());

      return next;
   }

   public static PIDSE3Gains nextPIDSE3Gains(Random random)
   {
      PIDSE3Gains next = new DefaultPIDSE3Gains();
      next.setPositionGains(nextPID3DGains(random));
      next.setOrientationGains(nextPID3DGains(random));
      return next;
   }

   public static JointDesiredOutput nextJointDesiredOutput(Random random)
   {
      JointDesiredOutput next = new JointDesiredOutput();
      next.setControlMode(nextElementIn(random, JointDesiredControlMode.values()));
      if (random.nextBoolean())
         next.setDesiredTorque(random.nextDouble());
      if (random.nextBoolean())
         next.setDesiredPosition(random.nextDouble());
      if (random.nextBoolean())
         next.setDesiredVelocity(random.nextDouble());
      if (random.nextBoolean())
         next.setDesiredAcceleration(random.nextDouble());
      next.setResetIntegrators(random.nextBoolean());
      if (random.nextBoolean())
         next.setStiffness(random.nextDouble());
      if (random.nextBoolean())
         next.setDamping(random.nextDouble());
      if (random.nextBoolean())
         next.setMasterGain(random.nextDouble());
      if (random.nextBoolean())
         next.setVelocityScaling(random.nextDouble());
      if (random.nextBoolean())
         next.setVelocityIntegrationBreakFrequency(random.nextDouble());
      if (random.nextBoolean())
         next.setPositionIntegrationBreakFrequency(random.nextDouble());
      if (random.nextBoolean())
         next.setMaxPositionError(random.nextDouble());
      if (random.nextBoolean())
         next.setMaxVelocityError(random.nextDouble());
      return next;
   }

   public static CenterOfPressureCommand nextCenterOfPressureCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      CenterOfPressureCommand next = new CenterOfPressureCommand();
      next.setConstraintType(nextElementIn(random, ConstraintType.values()));
      next.setContactingRigidBody(nextElementIn(random, rootBody.subtreeList()));
      next.setWeight(nextFrameVector2D(random, possibleFrames));
      next.setDesiredCoP(nextFramePoint2D(random, possibleFrames));
      return next;
   }

   public static ContactWrenchCommand nextContactWrenchCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      ContactWrenchCommand next = new ContactWrenchCommand();
      next.setConstraintType(nextElementIn(random, ConstraintType.values()));
      next.setRigidBody(nextElementIn(random, rootBody.subtreeList()));
      next.getWrench().setIncludingFrame(nextWrench(random, possibleFrames));
      next.getWeightMatrix().set(nextWeightMatrix6D(random, possibleFrames));
      next.getSelectionMatrix().set(nextSelectionMatrix6D(random, possibleFrames));
      return next;
   }

   public static ExternalWrenchCommand nextExternalWrenchCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      ExternalWrenchCommand next = new ExternalWrenchCommand();
      next.setRigidBody(nextElementIn(random, rootBody.subtreeList()));
      next.getExternalWrench().setIncludingFrame(nextWrench(random, possibleFrames));
      return next;
   }

   public static InverseDynamicsOptimizationSettingsCommand nextInverseDynamicsOptimizationSettingsCommand(Random random, RigidBodyBasics rootBody,
                                                                                                           ReferenceFrame... possibleFrames)
   {
      InverseDynamicsOptimizationSettingsCommand next = new InverseDynamicsOptimizationSettingsCommand();
      next.setRhoMin(random.nextDouble());
      next.setJointAccelerationMax(random.nextDouble());
      next.setRhoWeight(random.nextDouble());
      next.setRhoRateWeight(random.nextDouble());
      next.setCenterOfPressureWeight(EuclidCoreRandomTools.nextPoint2D(random));
      next.setCenterOfPressureRateWeight(EuclidCoreRandomTools.nextPoint2D(random));
      next.setJointAccelerationWeight(random.nextDouble());
      next.setJointJerkWeight(random.nextDouble());
      next.setJointTorqueWeight(random.nextDouble());
      return next;
   }

   public static JointAccelerationIntegrationCommand nextJointAccelerationIntegrationCommand(Random random, RigidBodyBasics rootBody,
                                                                                             ReferenceFrame... possibleFrames)
   {
      JointAccelerationIntegrationCommand next = new JointAccelerationIntegrationCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         next.addJointToComputeDesiredPositionFor(allJoints.remove(random.nextInt(allJoints.size())));
         next.setJointParameters(jointIndex, nextJointAccelerationIntegrationParameters(random));
      }

      return next;
   }

   public static JointLimitEnforcementMethodCommand nextJointLimitEnforcementMethodCommand(Random random, RigidBodyBasics rootBody,
                                                                                           ReferenceFrame... possibleFrames)
   {
      JointLimitEnforcementMethodCommand next = new JointLimitEnforcementMethodCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         next.addLimitEnforcementMethod(allJoints.remove(random.nextInt(allJoints.size())), nextElementIn(random, JointLimitEnforcement.values()),
                                        nextJointLimitParameters(random));
      }

      return next;
   }

   public static JointspaceAccelerationCommand nextJointspaceAccelerationCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      JointspaceAccelerationCommand next = new JointspaceAccelerationCommand();

      List<JointBasics> allJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         JointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         DenseMatrix64F desiredAcceleration = RandomMatrices.createRandom(joint.getDegreesOfFreedom(), 1, random);
         next.addJoint(joint, desiredAcceleration, random.nextDouble());
      }

      return next;
   }

   public static MomentumRateCommand nextMomentumRateCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      MomentumRateCommand next = new MomentumRateCommand();
      next.setMomentumRate(RandomMatrices.createRandom(6, 1, random));
      next.setWeights(nextWeightMatrix6D(random, possibleFrames));
      next.setSelectionMatrix(nextSelectionMatrix6D(random, possibleFrames));
      return next;
   }

   public static PlaneContactStateCommand nextPlaneContactStateCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      PlaneContactStateCommand next = new PlaneContactStateCommand();
      next.setContactingRigidBody(nextElementIn(random, rootBody.subtreeList()));
      next.setCoefficientOfFriction(random.nextDouble());
      next.setContactNormal(EuclidFrameRandomTools.nextFrameVector3D(random, nextElementIn(random, possibleFrames)));
      next.setUseHighCoPDamping(random.nextBoolean());
      next.setHasContactStateChanged(random.nextBoolean());
      if (random.nextBoolean())
         next.getContactFramePoseInBodyFixedFrame().set(EuclidCoreRandomTools.nextRigidBodyTransform(random));

      int numberOfContactPoints = random.nextInt(20);

      for (int i = 0; i < numberOfContactPoints; i++)
      {
         next.addPointInContact(EuclidFrameRandomTools.nextFramePoint3D(random, nextElementIn(random, possibleFrames)));
         if (random.nextBoolean())
            next.setMaxContactPointNormalForce(i, random.nextDouble());
         if (random.nextBoolean())
            next.setRhoWeight(i, random.nextDouble());
      }

      return next;
   }

   public static SpatialAccelerationCommand nextSpatialAccelerationCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      SpatialAccelerationCommand next = new SpatialAccelerationCommand();
      next.set(nextElementIn(random, rootBody.subtreeList()), nextElementIn(random, rootBody.subtreeList()));
      next.getControlFramePose().setIncludingFrame(nextFramePose3D(random, possibleFrames));
      next.getDesiredLinearAcceleration().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getDesiredAngularAcceleration().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.setWeightMatrix(nextWeightMatrix6D(random, possibleFrames));
      next.setSelectionMatrix(nextSelectionMatrix6D(random, possibleFrames));
      if (random.nextBoolean())
         next.setPrimaryBase(nextElementIn(random, rootBody.subtreeList()));
      if (random.nextBoolean())
         next.setScaleSecondaryTaskJointWeight(true, random.nextDouble());
      return next;
   }

   public static InverseKinematicsOptimizationSettingsCommand nextInverseKinematicsOptimizationSettingsCommand(Random random, RigidBodyBasics rootBody,
                                                                                                               ReferenceFrame... possibleFrames)
   {
      InverseKinematicsOptimizationSettingsCommand next = new InverseKinematicsOptimizationSettingsCommand();
      next.setJointVelocityWeight(random.nextDouble());
      next.setJointAccelerationWeight(random.nextDouble());
      return next;
   }

   public static JointLimitReductionCommand nextJointLimitReductionCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      JointLimitReductionCommand next = new JointLimitReductionCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addReductionFactor(joint, random.nextDouble());
      }

      return next;
   }

   public static JointspaceVelocityCommand nextJointspaceVelocityCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      JointspaceVelocityCommand next = new JointspaceVelocityCommand();

      List<JointBasics> allJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         JointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addJoint(joint, RandomMatrices.createRandom(joint.getDegreesOfFreedom(), 1, random), random.nextDouble());
      }

      return next;
   }

   public static MomentumCommand nextMomentumCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      MomentumCommand next = new MomentumCommand();
      next.setMomentum(RandomMatrices.createRandom(6, 1, random));
      next.getWeightMatrix().set(nextWeightMatrix6D(random, possibleFrames));
      next.getSelectionMatrix().set(nextSelectionMatrix6D(random, possibleFrames));
      return next;
   }

   public static PrivilegedConfigurationCommand nextPrivilegedConfigurationCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      PrivilegedConfigurationCommand next = new PrivilegedConfigurationCommand();

      if (random.nextBoolean())
         next.setDefaultParameters(nextOneDoFJointPrivilegedConfigurationParameters(random));

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addJoint(joint, nextOneDoFJointPrivilegedConfigurationParameters(random));
      }

      if (random.nextBoolean())
         next.enable();
      else
         next.disable();

      return next;
   }

   public static PrivilegedJointSpaceCommand nextPrivilegedJointSpaceCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      PrivilegedJointSpaceCommand next = new PrivilegedJointSpaceCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));

         next.addJoint(joint, random.nextDouble());
         if (random.nextBoolean())
            next.setWeight(jointIndex, random.nextDouble());
      }

      if (random.nextBoolean())
         next.enable();
      else
         next.disable();

      return next;
   }

   public static SpatialVelocityCommand nextSpatialVelocityCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      SpatialVelocityCommand next = new SpatialVelocityCommand();
      next.set(nextElementIn(random, rootBody.subtreeList()), nextElementIn(random, rootBody.subtreeList()));
      next.getControlFramePose().setIncludingFrame(nextFramePose3D(random, possibleFrames));
      next.getDesiredLinearVelocity().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getDesiredAngularVelocity().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.setWeightMatrix(nextWeightMatrix6D(random, possibleFrames));
      next.setSelectionMatrix(nextSelectionMatrix6D(random, possibleFrames));
      if (random.nextBoolean())
         next.setPrimaryBase(nextElementIn(random, rootBody.subtreeList()));
      if (random.nextBoolean())
         next.setScaleSecondaryTaskJointWeight(true, random.nextDouble());
      return next;
   }

   public static JointLimitEnforcementCommand nextJointLimitEnforcementCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      JointLimitEnforcementCommand next = new JointLimitEnforcementCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addJoint(joint, nextJointLimitData(random));
      }

      return next;
   }

   public static JointTorqueCommand nextJointTorqueCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      JointTorqueCommand next = new JointTorqueCommand();

      List<JointBasics> allJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         JointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addJoint(joint, RandomMatrices.createRandom(joint.getDegreesOfFreedom(), 1, random));
      }

      return next;
   }

   public static VirtualForceCommand nextVirtualForceCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      VirtualForceCommand next = new VirtualForceCommand();

      next.set(nextElementIn(random, rootBody.subtreeList()), nextElementIn(random, rootBody.subtreeList()));
      next.getDesiredLinearForce().set(EuclidCoreRandomTools.nextVector3D(random));
      next.getControlFramePose().setIncludingFrame(nextFramePose3D(random, possibleFrames));
      next.setSelectionMatrix(nextSelectionMatrix3D(random, possibleFrames));

      return next;
   }

   public static VirtualModelControlOptimizationSettingsCommand nextVirtualModelControlOptimizationSettingsCommand(Random random, RigidBodyBasics rootBody,
                                                                                                                   ReferenceFrame... possibleFrames)
   {
      VirtualModelControlOptimizationSettingsCommand next = new VirtualModelControlOptimizationSettingsCommand();
      next.setRhoMin(random.nextDouble());
      next.setRhoWeight(random.nextDouble());
      next.setRhoRateWeight(random.nextDouble());
      next.setCenterOfPressureWeight(EuclidCoreRandomTools.nextPoint2D(random));
      next.setCenterOfPressureRateWeight(EuclidCoreRandomTools.nextPoint2D(random));
      next.setMomentumRateWeight(random.nextDouble());
      next.setMomentumAccelerationWeight(random.nextDouble());
      return next;
   }

   public static VirtualTorqueCommand nextVirtualTorqueCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      VirtualTorqueCommand next = new VirtualTorqueCommand();

      next.set(nextElementIn(random, rootBody.subtreeList()), nextElementIn(random, rootBody.subtreeList()));
      next.getDesiredAngularTorque().set(EuclidCoreRandomTools.nextVector3D(random));
      next.getControlFramePose().setIncludingFrame(nextFramePose3D(random, possibleFrames));
      next.setSelectionMatrix(nextSelectionMatrix3D(random, possibleFrames));

      return next;
   }

   public static VirtualWrenchCommand nextVirtualWrenchCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      VirtualWrenchCommand next = new VirtualWrenchCommand();

      next.set(nextElementIn(random, rootBody.subtreeList()), nextElementIn(random, rootBody.subtreeList()));
      next.getDesiredLinearForce().set(EuclidCoreRandomTools.nextVector3D(random));
      next.getDesiredAngularTorque().set(EuclidCoreRandomTools.nextVector3D(random));
      next.getControlFramePose().setIncludingFrame(nextFramePose3D(random, possibleFrames));
      next.setSelectionMatrix(nextSelectionMatrix6D(random, possibleFrames));

      return next;
   }

   public static CenterOfMassFeedbackControlCommand nextCenterOfMassFeedbackControlCommand(Random random, RigidBodyBasics rootBody,
                                                                                           ReferenceFrame... possibleFrames)
   {
      CenterOfMassFeedbackControlCommand next = new CenterOfMassFeedbackControlCommand();
      next.setControlMode(nextElementIn(random, WholeBodyControllerCoreMode.values()));
      next.getReferencePosition().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getReferenceLinearVelocity().set(EuclidCoreRandomTools.nextVector3D(random));
      next.getReferenceLinearAcceleration().set(EuclidCoreRandomTools.nextVector3D(random));
      next.setGains(nextPID3DGains(random));
      next.getMomentumRateCommand().set(nextMomentumRateCommand(random, rootBody, possibleFrames));
      return next;
   }

   public static OneDoFJointFeedbackControlCommand nextOneDoFJointFeedbackControlCommand(Random random, RigidBodyBasics rootBody,
                                                                                         ReferenceFrame... possibleFrames)
   {
      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      OneDoFJointBasics joint = allJoints.get(random.nextInt(allJoints.size()));
      return nextOneDoFJointFeedbackControlCommand(random, joint);
   }

   public static OneDoFJointFeedbackControlCommand nextOneDoFJointFeedbackControlCommand(Random random, OneDoFJointBasics joint)
   {
      OneDoFJointFeedbackControlCommand next = new OneDoFJointFeedbackControlCommand();
      next.setJoint(joint);
      next.setControlMode(nextElementIn(random, WholeBodyControllerCoreMode.values()));
      next.setInverseDynamics(random.nextDouble(), random.nextDouble(), random.nextDouble());
      next.setGains(nextPDGains(random));
      next.setWeightForSolver(random.nextDouble());
      return next;
   }

   public static JointspaceFeedbackControlCommand nextJointspaceFeedbackControlCommand(Random random, RigidBodyBasics rootBody,
                                                                                       ReferenceFrame... possibleFrames)
   {
      JointspaceFeedbackControlCommand next = new JointspaceFeedbackControlCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addCommand(nextOneDoFJointFeedbackControlCommand(random, joint));
      }

      return next;
   }

   public static OrientationFeedbackControlCommand nextOrientationFeedbackControlCommand(Random random, RigidBodyBasics rootBody,
                                                                                         ReferenceFrame... possibleFrames)
   {
      OrientationFeedbackControlCommand next = new OrientationFeedbackControlCommand();
      next.setControlMode(nextElementIn(random, WholeBodyControllerCoreMode.values()));
      next.getBodyFixedOrientationToControl().setIncludingFrame(nextFrameQuaternion(random, possibleFrames));
      next.getReferenceOrientation().setIncludingFrame(nextFrameQuaternion(random, possibleFrames));
      next.getReferenceAngularVelocity().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceAngularAcceleration().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceTorque().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getGains().set(nextPID3DGains(random));
      next.setGainsFrame(nextElementIn(random, possibleFrames));
      next.getSpatialAccelerationCommand().set(nextSpatialAccelerationCommand(random, rootBody, possibleFrames));
      next.setControlBaseFrame(nextElementIn(random, possibleFrames));
      return next;
   }

   public static PointFeedbackControlCommand nextPointFeedbackControlCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      PointFeedbackControlCommand next = new PointFeedbackControlCommand();
      next.setControlMode(nextElementIn(random, WholeBodyControllerCoreMode.values()));
      next.getBodyFixedPointToControl().setIncludingFrame(nextFramePoint3D(random, possibleFrames));
      next.getReferencePosition().setIncludingFrame(nextFramePoint3D(random, possibleFrames));
      next.getReferenceLinearVelocity().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceLinearAcceleration().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceForce().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getGains().set(nextPID3DGains(random));
      next.setGainsFrame(nextElementIn(random, possibleFrames));
      next.getSpatialAccelerationCommand().set(nextSpatialAccelerationCommand(random, rootBody, possibleFrames));
      next.setControlBaseFrame(nextElementIn(random, possibleFrames));
      return next;
   }

   public static SpatialFeedbackControlCommand nextSpatialFeedbackControlCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      SpatialFeedbackControlCommand next = new SpatialFeedbackControlCommand();
      next.setControlMode(nextElementIn(random, WholeBodyControllerCoreMode.values()));
      next.getControlFramePose().setIncludingFrame(nextFramePose3D(random, possibleFrames));
      next.getReferenceOrientation().setIncludingFrame(nextFrameQuaternion(random, possibleFrames));
      next.getReferenceAngularVelocity().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceAngularAcceleration().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceTorque().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getGains().set(nextPIDSE3Gains(random));
      next.setGainsFrames(nextElementIn(random, possibleFrames), nextElementIn(random, possibleFrames));
      next.getSpatialAccelerationCommand().set(nextSpatialAccelerationCommand(random, rootBody, possibleFrames));
      next.setControlBaseFrame(nextElementIn(random, possibleFrames));
      next.getReferencePosition().setIncludingFrame(nextFramePoint3D(random, possibleFrames));
      next.getReferenceLinearVelocity().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceLinearAcceleration().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceForce().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getSpatialAccelerationCommand().set(nextSpatialAccelerationCommand(random, rootBody, possibleFrames));
      next.setControlBaseFrame(nextElementIn(random, possibleFrames));
      return next;
   }

   @SuppressWarnings("rawtypes")
   public static InverseDynamicsCommandList nextInverseDynamicsCommandList(Random random,
                                                                           Collection<Class<? extends InverseDynamicsCommand>> commandsToGenerate,
                                                                           RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      InverseDynamicsCommandList next = new InverseDynamicsCommandList();

      List<Class<? extends InverseDynamicsCommand>> commandTypes = new ArrayList<>(commandsToGenerate);
      List<MutableInt> numberOfCommandsToGenerate = new ArrayList<>();
      commandTypes.forEach(c -> numberOfCommandsToGenerate.add(new MutableInt(random.nextInt(10))));

      while (!commandTypes.isEmpty())
      {
         int index = random.nextInt(commandTypes.size());
         Class<? extends InverseDynamicsCommand> commandType = commandTypes.get(index);
         if (numberOfCommandsToGenerate.get(index).getAndDecrement() == 0)
         {
            commandTypes.remove(index);
            numberOfCommandsToGenerate.remove(index);
         }

         Method randomGenerator = CrossRobotCommandResolverTest.class.getDeclaredMethod("next" + commandType.getSimpleName(), Random.class,
                                                                                        RigidBodyBasics.class, ReferenceFrame[].class);
         InverseDynamicsCommand<?> command = (InverseDynamicsCommand<?>) randomGenerator.invoke(null, random, rootBody, possibleFrames);
         next.addCommand(command);
      }
      return next;
   }

   @SuppressWarnings("rawtypes")
   public static InverseKinematicsCommandList nextInverseKinematicsCommandList(Random random,
                                                                               Collection<Class<? extends InverseKinematicsCommand>> commandsToGenerate,
                                                                               RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      InverseKinematicsCommandList next = new InverseKinematicsCommandList();

      List<Class<? extends InverseKinematicsCommand>> commandTypes = new ArrayList<>(commandsToGenerate);
      List<MutableInt> numberOfCommandsToGenerate = new ArrayList<>();
      commandTypes.forEach(c -> numberOfCommandsToGenerate.add(new MutableInt(random.nextInt(10))));

      while (!commandTypes.isEmpty())
      {
         int index = random.nextInt(commandTypes.size());
         Class<? extends InverseKinematicsCommand> commandType = commandTypes.get(index);
         if (numberOfCommandsToGenerate.get(index).getAndDecrement() == 0)
         {
            commandTypes.remove(index);
            numberOfCommandsToGenerate.remove(index);
         }

         Method randomGenerator = CrossRobotCommandResolverTest.class.getDeclaredMethod("next" + commandType.getSimpleName(), Random.class,
                                                                                        RigidBodyBasics.class, ReferenceFrame[].class);
         InverseKinematicsCommand<?> command = (InverseKinematicsCommand<?>) randomGenerator.invoke(null, random, rootBody, possibleFrames);
         next.addCommand(command);
      }
      return next;
   }

   @SuppressWarnings("rawtypes")
   public static VirtualModelControlCommandList nextVirtualModelControlCommandList(Random random,
                                                                                   Collection<Class<? extends VirtualModelControlCommand>> commandsToGenerate,
                                                                                   RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      VirtualModelControlCommandList next = new VirtualModelControlCommandList();

      List<Class<? extends VirtualModelControlCommand>> commandTypes = new ArrayList<>(commandsToGenerate);
      List<MutableInt> numberOfCommandsToGenerate = new ArrayList<>();
      commandTypes.forEach(c -> numberOfCommandsToGenerate.add(new MutableInt(random.nextInt(10))));

      while (!commandTypes.isEmpty())
      {
         int index = random.nextInt(commandTypes.size());
         Class<? extends VirtualModelControlCommand> commandType = commandTypes.get(index);
         if (numberOfCommandsToGenerate.get(index).getAndDecrement() == 0)
         {
            commandTypes.remove(index);
            numberOfCommandsToGenerate.remove(index);
         }

         Method randomGenerator = CrossRobotCommandResolverTest.class.getDeclaredMethod("next" + commandType.getSimpleName(), Random.class,
                                                                                        RigidBodyBasics.class, ReferenceFrame[].class);
         VirtualModelControlCommand<?> command = (VirtualModelControlCommand<?>) randomGenerator.invoke(null, random, rootBody, possibleFrames);
         next.addCommand(command);
      }
      return next;
   }

   @SuppressWarnings("rawtypes")
   public static FeedbackControlCommandList nextFeedbackControlCommandList(Random random,
                                                                           Collection<Class<? extends FeedbackControlCommand>> commandsToGenerate,
                                                                           RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      FeedbackControlCommandList next = new FeedbackControlCommandList();

      List<Class<? extends FeedbackControlCommand>> commandTypes = new ArrayList<>(commandsToGenerate);
      List<MutableInt> numberOfCommandsToGenerate = new ArrayList<>();
      commandTypes.forEach(c -> numberOfCommandsToGenerate.add(new MutableInt(random.nextInt(10))));

      while (!commandTypes.isEmpty())
      {
         int index = random.nextInt(commandTypes.size());
         Class<? extends FeedbackControlCommand> commandType = commandTypes.get(index);
         if (numberOfCommandsToGenerate.get(index).getAndDecrement() == 0)
         {
            commandTypes.remove(index);
            numberOfCommandsToGenerate.remove(index);
         }

         Method randomGenerator = CrossRobotCommandResolverTest.class.getDeclaredMethod("next" + commandType.getSimpleName(), Random.class,
                                                                                        RigidBodyBasics.class, ReferenceFrame[].class);
         FeedbackControlCommand<?> command = (FeedbackControlCommand<?>) randomGenerator.invoke(null, random, rootBody, possibleFrames);
         next.addCommand(command);
      }
      return next;
   }

   public static LowLevelOneDoFJointDesiredDataHolder nextLowLevelOneDoFJointDesiredDataHolder(Random random, RigidBodyBasics rootBody,
                                                                                               ReferenceFrame... possibleFrames)
   {
      LowLevelOneDoFJointDesiredDataHolder next = new LowLevelOneDoFJointDesiredDataHolder();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.registerJointWithEmptyData(joint).set(nextJointDesiredOutput(random));
      }

      return next;
   }

   @SuppressWarnings("rawtypes")
   public static ControllerCoreCommand nextControllerCoreCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      Reflections reflections = new Reflections(CONTROLLER_CORE_COMMANDS_PACKAGE);
      Set<Class<? extends InverseDynamicsCommand>> inverseDynamicsCommandsToGenerate = reflections.getSubTypesOf(InverseDynamicsCommand.class);
      inverseDynamicsCommandsToGenerate.remove(InverseDynamicsCommandList.class);
      inverseDynamicsCommandsToGenerate.remove(InverseDynamicsCommandBuffer.class);
      Set<Class<? extends InverseKinematicsCommand>> inverseKinematicsCommandsToGenerate = reflections.getSubTypesOf(InverseKinematicsCommand.class);
      inverseKinematicsCommandsToGenerate.remove(InverseKinematicsCommandList.class);
      inverseKinematicsCommandsToGenerate.remove(InverseKinematicsCommandBuffer.class);
      Set<Class<? extends VirtualModelControlCommand>> virtualModelControlCommandsToGenerate = reflections.getSubTypesOf(VirtualModelControlCommand.class);
      virtualModelControlCommandsToGenerate.remove(VirtualModelControlCommandList.class);
      virtualModelControlCommandsToGenerate.remove(VirtualModelControlCommandBuffer.class);
      virtualModelControlCommandsToGenerate.remove(VirtualEffortCommand.class);
      Set<Class<? extends FeedbackControlCommand>> feedbackControlCommandsToGenerate = reflections.getSubTypesOf(FeedbackControlCommand.class);
      feedbackControlCommandsToGenerate.remove(FeedbackControlCommandList.class);
      feedbackControlCommandsToGenerate.remove(FeedbackControlCommandBuffer.class);

      return nextControllerCoreCommand(random, inverseDynamicsCommandsToGenerate, inverseKinematicsCommandsToGenerate, virtualModelControlCommandsToGenerate,
                                       feedbackControlCommandsToGenerate, rootBody, possibleFrames);
   }

   @SuppressWarnings("rawtypes")
   public static ControllerCoreCommand nextControllerCoreCommand(Random random,
                                                                 Collection<Class<? extends InverseDynamicsCommand>> inverseDynamicsCommandsToGenerate,
                                                                 Collection<Class<? extends InverseKinematicsCommand>> inverseKinematicsCommandsToGenerate,
                                                                 Collection<Class<? extends VirtualModelControlCommand>> virtualModelControlCommandsToGenerate,
                                                                 Collection<Class<? extends FeedbackControlCommand>> feedbackControlCommandsToGenerate,
                                                                 RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, IllegalAccessException, InvocationTargetException
   {
      ControllerCoreCommand next = new ControllerCoreCommand(nextElementIn(random, WholeBodyControllerCoreMode.values()));

      next.getInverseDynamicsCommandList().set(nextInverseDynamicsCommandList(random, inverseDynamicsCommandsToGenerate, rootBody, possibleFrames));
      next.getInverseKinematicsCommandList().set(nextInverseKinematicsCommandList(random, inverseKinematicsCommandsToGenerate, rootBody, possibleFrames));
      next.getVirtualModelControlCommandList().set(nextVirtualModelControlCommandList(random, virtualModelControlCommandsToGenerate, rootBody, possibleFrames));
      next.getFeedbackControlCommandList().set(nextFeedbackControlCommandList(random, feedbackControlCommandsToGenerate, rootBody, possibleFrames));
      next.getLowLevelOneDoFJointDesiredDataHolder().overwriteWith(nextLowLevelOneDoFJointDesiredDataHolder(random, rootBody, possibleFrames));
      if (random.nextBoolean())
         next.requestReinitialization();
      return next;
   }
}
