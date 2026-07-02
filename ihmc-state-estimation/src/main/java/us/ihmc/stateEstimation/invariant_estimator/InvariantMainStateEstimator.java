package us.ihmc.stateEstimation.invariant_estimator;

import gnu.trove.map.TObjectDoubleMap;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Runs the contact-aided right-invariant InEKF as the <em>main</em> floating-base estimator.
 *
 * <p>Unlike {@link InvariantEKFStateEstimator} (which runs as a secondary/evaluation estimator and only
 * logs), this owns the responsibilities a main {@link StateEstimatorController} must fulfil for the
 * controller:
 * <ol>
 *   <li><b>Joints</b> — sets every one-DoF joint's q/q̇/τ directly from the processed sensor output and
 *       updates the model frames. This is the "good-enough FK" path: the same thing the DRC estimator's
 *       {@code JointStateUpdater} does, with no joint-space KF (that is the later P-A pre-filter). The
 *       InEKF's contact FK measurement noise stays the constant {@code contactMeasurementVariance} until
 *       that pre-filter exists.</li>
 *   <li><b>Floating base</b> — runs the InEKF predict + soft-contact update via a wrapped
 *       {@link InvariantEKFStateEstimator}.</li>
 *   <li><b>Yaw seeding</b> — optional {@link FootReferencedYawCorrector}, since heading is unobservable.</li>
 *   <li><b>Output</b> — writes the estimated root-joint pose and twist, mirroring how the DRC pelvis
 *       updaters write {@code rootJoint} (orientation/position setters, angular twist in body, linear twist
 *       via {@code setMatchingFrame} from world). The published context then carries the InEKF base to the
 *       controller.</li>
 *   <li><b>Center of mass</b> — runs an {@link InvariantCenterOfMassUpdater} (the {@code MomentumStateUpdater}
 *       seam) <em>after</em> the root-joint write, so CoM position/velocity are derived purely from the InEKF
 *       base and published to the controller via the shared {@link CenterOfMassDataHolder}. Velocity is
 *       kinematic (no GRF/centroidal fusion yet); see that class for the caveat and the upgrade seam.</li>
 * </ol>
 * The force-sensor data holder is intentionally <em>not</em> updated here: the sensor reader fills the
 * force-sensor holder the controller reads, mirroring the division of labour the EKF main estimator
 * ({@code HumanoidRobotEKFWithSimpleJoints}) relies on.</p>
 */
public class InvariantMainStateEstimator implements StateEstimatorController
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final FullHumanoidRobotModel fullRobotModel;
   private final SensorOutputMapReadOnly processedSensorOutput;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final FloatingJointBasics rootJoint;

   private final HumanoidReferenceFrames referenceFrames;

   private final InvariantEKFStateEstimator invariantEstimator;
   private final FootReferencedYawCorrector yawCorrector; // null when yaw seeding is disabled
   private final InvariantCenterOfMassUpdater centerOfMassUpdater;

   // Root-joint write temporaries.
   private final RotationMatrix rootOrientation = new RotationMatrix();
   private final Vector3D rootPosition = new Vector3D();
   private final Vector3D baseVelocityWorld = new Vector3D();
   private final Vector3D angularVelocityBody = new Vector3D();
   private final FrameVector3D rootLinearVelocityWorld = new FrameVector3D();

   /**
    * @param fullRobotModel             the estimator's full robot model (joints set here, root written here).
    * @param processedSensorOutput      processed sensor outputs (IMU + joints).
    * @param primaryImuName             sensor name of the pelvis (base) IMU to use, e.g.
    *                                   {@code sensorInformation.getPrimaryBodyImu()}.
    * @param centerOfMassDataHolder     shared CoM holder, written each tick from the InEKF base estimate and
    *                                   published to the controller; may be {@code null} to skip CoM output.
    * @param dt                         estimator timestep Δt (s).
    * @param gyroVariance               continuous gyro noise variance σ_ω².
    * @param accelVariance              continuous accel noise variance σ_a².
    * @param contactVariance            continuous contact-slip noise variance σ_c² (base value at full contact).
    * @param contactMeasurementVariance per-axis body-frame contact FK measurement variance (m²).
    * @param initialCovariance          scalar for the initial P = initialCovariance · I.
    * @param enableYawSeeding           if true, install the foot-referenced yaw corrector.
    */
   public InvariantMainStateEstimator(FullHumanoidRobotModel fullRobotModel,
                                      SensorOutputMapReadOnly processedSensorOutput,
                                      String primaryImuName,
                                      CenterOfMassDataHolder centerOfMassDataHolder,
                                      double dt,
                                      double gyroVariance,
                                      double accelVariance,
                                      double contactVariance,
                                      double contactMeasurementVariance,
                                      double initialCovariance,
                                      boolean enableYawSeeding)
   {
      this.fullRobotModel = fullRobotModel;
      this.processedSensorOutput = processedSensorOutput;
      this.oneDoFJoints = fullRobotModel.getOneDoFJoints();
      this.rootJoint = fullRobotModel.getRootJoint();

      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      invariantEstimator = new InvariantEKFStateEstimator(fullRobotModel,
                                                          processedSensorOutput,
                                                          primaryImuName,
                                                          dt,
                                                          gyroVariance,
                                                          accelVariance,
                                                          contactVariance,
                                                          contactMeasurementVariance,
                                                          initialCovariance);
      invariantEstimator.setRunningAsMain(true); // disables the invariantMinusMain* self-comparisons
      registry.addChild(invariantEstimator.getYoRegistry());

      if (enableYawSeeding)
      {
         SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();
         for (RobotSide side : RobotSide.values)
            soleFrames.put(side, referenceFrames.getSoleFrame(side));

         yawCorrector = new FootReferencedYawCorrector(invariantEstimator.getInvariantEKF(),
                                                       referenceFrames.getPelvisFrame(),
                                                       soleFrames,
                                                       invariantEstimator::getContactProbability,
                                                       registry);
      }
      else
      {
         yawCorrector = null;
      }

      centerOfMassUpdater = new InvariantCenterOfMassUpdater(rootJoint, centerOfMassDataHolder);
      registry.addChild(centerOfMassUpdater.getRegistry());
   }

   @Override
   public void doControl()
   {
      updateJoints();
      referenceFrames.updateFrames();

      invariantEstimator.doControl();

      if (yawCorrector != null)
         yawCorrector.correct();

      writeRootJoint();

      // CoM position/velocity for the controller, derived from the InEKF base just written to the root joint.
      centerOfMassUpdater.update();
   }

   /** Sets q/q̇/τ for every one-DoF joint from the processed sensor output and refreshes the model frames. */
   private void updateJoints()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];
         OneDoFJointStateReadOnly jointOutput = processedSensorOutput.getOneDoFJointOutput(joint);
         joint.setQ(jointOutput.getPosition());
         joint.setQd(jointOutput.getVelocity());
         joint.setTau(jointOutput.getEffort());
      }
      fullRobotModel.getElevator().updateFramesRecursively();
   }

   /** Publishes the InEKF base estimate to the root joint (mirrors the DRC pelvis updaters). */
   private void writeRootJoint()
   {
      InvariantEKF ekf = invariantEstimator.getInvariantEKF();
      ekf.getRotation(rootOrientation);
      ekf.getBasePosition(rootPosition);
      ekf.getBaseVelocity(baseVelocityWorld);
      invariantEstimator.getMeasuredAngularVelocityInBody(angularVelocityBody);

      rootJoint.setJointOrientation(rootOrientation);
      rootJoint.setJointPosition(rootPosition);
      rootJoint.updateFrame(); // refresh the after-joint frame so the world→body twist conversion is correct

      // Angular part: body-frame gyro (same physical frame as the joint's after-joint/pelvis frame).
      rootJoint.getJointTwist().getAngularPart().set(angularVelocityBody);
      // Linear part: world-frame base velocity, converted into the twist's (body) frame.
      rootLinearVelocityWorld.setIncludingFrame(ReferenceFrame.getWorldFrame(), baseVelocityWorld);
      rootJoint.getJointTwist().getLinearPart().setMatchingFrame(rootLinearVelocityWorld);

      rootJoint.updateFramesRecursively();
   }

   @Override
   public void initialize()
   {
      updateJoints();
      referenceFrames.updateFrames();
      invariantEstimator.initialize();
      centerOfMassUpdater.initialize();
   }

   @Override
   public void initializeEstimator(RigidBodyTransformReadOnly rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      updateJoints();
      rootJoint.setJointOrientation(rootJointTransform.getRotation());
      rootJoint.setJointPosition(rootJointTransform.getTranslation());
      rootJoint.updateFramesRecursively();
      referenceFrames.updateFrames();

      invariantEstimator.initializeEstimator(rootJointTransform, jointPositions);

      if (yawCorrector != null)
         yawCorrector.reset();

      centerOfMassUpdater.initialize();
   }

   @Override
   public void requestStateEstimatorMode(StateEstimatorMode operatingMode)
   {
      invariantEstimator.requestStateEstimatorMode(operatingMode);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(name);
      group.addChild(invariantEstimator.getSCS2YoGraphics());
      return group;
   }

   /** @return the wrapped InEKF evaluation estimator (base estimate + soft contact + YoVariables). */
   public InvariantEKFStateEstimator getInvariantEKFStateEstimator()
   {
      return invariantEstimator;
   }
}
