package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.List;

/**
 * Live-switchable joint-level source (2026-07-16, 12 Hz anti-damping investigation): runs BOTH the
 * JointLevelKF and the AlphaComplementary pre-filters every tick and selects, via the
 * {@code jointLevelSourceSelection} YoEnum, whose joint positions/velocities feed the controller.
 * This is the H5 discriminator experiment as a one-flag hardware A/B: same InEKF base estimate,
 * swap the joint q/qd source mid-session.
 * <p>
 * The {@link us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider} side
 * is PINNED for the life of this instance to whichever source was selected at BOOT (construction),
 * regardless of later live q/qd switches: the InEKF integrates the provided gyro bias into base
 * orientation, and two independently-running filters estimating the same physical bias will not agree
 * tick-to-tick, so retargeting the bias source on a LIVE switch would inject an orientation-rate step.
 * Boot-time selection carries no such risk -- nothing has integrated anything yet -- so it is free to
 * pick either source's bias, which is what lets {@code ALPHA_COMPLEMENTARY} be a genuinely complete
 * old-pipeline option (q, qd, AND bias) rather than a hybrid with the JointKF's bias underneath. Both
 * filters stay warm regardless, so the q/qd switch itself is always bumpless.
 */
public class SwitchableJointLevelSource implements ProprioceptivePreFilter
{
   public enum JointLevelSource { JOINT_KF, ALPHA_COMPLEMENTARY }

   private final ProprioceptivePreFilter jointKF;
   private final ProprioceptivePreFilter alphaComplementary;
   private final ProprioceptivePreFilter biasSource;
   private final YoEnum<JointLevelSource> selection;

   /** Defaults the live selection, and therefore the pinned bias source, to {@link JointLevelSource#JOINT_KF}. */
   public SwitchableJointLevelSource(ProprioceptivePreFilter jointKF, ProprioceptivePreFilter alphaComplementary, YoRegistry registry)
   {
      this(jointKF, alphaComplementary, JointLevelSource.JOINT_KF, registry);
   }

   /**
    * @param initialSelection which source is active before anything switches it, AND which source's
    *                         IMU bias is pinned for the life of this instance (see class javadoc).
    *                         Symmetric with the
    *                         {@link us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters.JointLevelEstimatorType}
    *                         that was actually configured: whichever one was chosen at boot is the one
    *                         live the moment this estimator starts ticking, and the other stays warm in
    *                         the background ready to switch the q/qd source to -- not always JOINT_KF
    *                         regardless of configuration.
    */
   public SwitchableJointLevelSource(ProprioceptivePreFilter jointKF, ProprioceptivePreFilter alphaComplementary, JointLevelSource initialSelection,
                                      YoRegistry registry)
   {
      this.jointKF = jointKF;
      this.alphaComplementary = alphaComplementary;
      this.biasSource = initialSelection == JointLevelSource.JOINT_KF ? jointKF : alphaComplementary;
      selection = new YoEnum<>("jointLevelSourceSelection", registry, JointLevelSource.class);
      selection.set(initialSelection);
   }

   /** The wrapped JointKF, e.g. for wiring its initialization gate. */
   public ProprioceptivePreFilter getJointKF()
   {
      return jointKF;
   }

   private ProprioceptivePreFilter active()
   {
      return selection.getEnumValue() == JointLevelSource.JOINT_KF ? jointKF : alphaComplementary;
   }

   @Override
   public void initialize()
   {
      jointKF.initialize();
      alphaComplementary.initialize();
   }

   @Override
   public void computeJointState()
   {
      jointKF.computeJointState();
      alphaComplementary.computeJointState();
   }

   @Override
   public void computeImuBiases(List<RigidBodyBasics> trustedFeet)
   {
      jointKF.computeImuBiases(trustedFeet);
      alphaComplementary.computeImuBiases(trustedFeet);
   }

   @Override
   public boolean containsJoint(OneDoFJointBasics joint)
   {
      return active().containsJoint(joint);
   }

   @Override
   public double getEstimatedJointPosition(OneDoFJointBasics joint)
   {
      return active().getEstimatedJointPosition(joint);
   }

   @Override
   public double getEstimatedJointVelocity(OneDoFJointBasics joint)
   {
      return active().getEstimatedJointVelocity(joint);
   }

   @Override
   public boolean hasCovariance()
   {
      return active().hasCovariance();
   }

   @Override
   public void packPositionCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack)
   {
      active().packPositionCovariance(joints, fallbackVariance, toPack);
   }

   @Override
   public void packVelocityCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack)
   {
      active().packVelocityCovariance(joints, fallbackVariance, toPack);
   }

   // IMU biases: PINNED to whichever source was selected at boot (see class javadoc) -- NOT `active()`,
   // which tracks the live q/qd selection and would retarget the bias source on every live switch.

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      return biasSource.getAngularVelocityBiasInIMUFrame(imu);
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      return biasSource.getAngularVelocityBiasInWorldFrame(imu);
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      return biasSource.getLinearAccelerationBiasInIMUFrame(imu);
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      return biasSource.getLinearAccelerationBiasInWorldFrame(imu);
   }
}
