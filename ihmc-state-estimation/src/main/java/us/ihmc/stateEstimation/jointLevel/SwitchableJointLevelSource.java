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
 * is PINNED to the JointKF regardless of selection: the InEKF integrates the provided gyro bias
 * into base orientation, so switching the bias source would inject an orientation-rate step. Both
 * filters stay warm, so the q/qd switch itself is bumpless.
 */
public class SwitchableJointLevelSource implements ProprioceptivePreFilter
{
   public enum JointLevelSource { JOINT_KF, ALPHA_COMPLEMENTARY }

   private final ProprioceptivePreFilter jointKF;
   private final ProprioceptivePreFilter alphaComplementary;
   private final YoEnum<JointLevelSource> selection;

   public SwitchableJointLevelSource(ProprioceptivePreFilter jointKF, ProprioceptivePreFilter alphaComplementary, YoRegistry registry)
   {
      this.jointKF = jointKF;
      this.alphaComplementary = alphaComplementary;
      selection = new YoEnum<>("jointLevelSourceSelection", registry, JointLevelSource.class);
      selection.set(JointLevelSource.JOINT_KF);
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

   // IMU biases: PINNED to the JointKF in both modes (see class javadoc).

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      return jointKF.getAngularVelocityBiasInIMUFrame(imu);
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      return jointKF.getAngularVelocityBiasInWorldFrame(imu);
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      return jointKF.getLinearAccelerationBiasInIMUFrame(imu);
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      return jointKF.getLinearAccelerationBiasInWorldFrame(imu);
   }
}
