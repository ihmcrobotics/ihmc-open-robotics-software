package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PID3DConfiguration;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class AtlasJumpControllerParameters extends JumpControllerParameters
{
   private final RobotTarget target;
   private final boolean runningOnRealRobot;
   private final SideDependentList<RigidBodyTransform> handPosesWithRespectToChestFrame = new SideDependentList<RigidBodyTransform>();

   private final AtlasJointMap jointMap;
   private final AtlasMomentumOptimizationSettings momentumOptimizationSettings;
   private final double massScale;

   private TObjectDoubleHashMap<String> jointHomeConfiguration = null;
   private Map<String, Pose3D> bodyHomeConfiguration = null;

   private final JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters;
   private final Vector3D maxForce = new Vector3D(10.0, 10.0, 1000.0);
   private final Vector3D minForce = new Vector3D(-10.0, -10.0, 0.0);
   private final Vector3D maxForceRate = new Vector3D(0.2, 0.2, 100.0);
   private final Vector3D minForceRate = new Vector3D(-0.2, -0.2, -100.0);

   public AtlasJumpControllerParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
   {
      this.target = target;
      this.jointMap = jointMap;
      this.massScale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());

      momentumOptimizationSettings = new AtlasMomentumOptimizationSettings(jointMap, contactPointParameters.getNumberOfContactableBodies());

      runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      jointPrivilegedConfigurationParameters = new AtlasJointPrivilegedConfigurationParameters(runningOnRealRobot);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform transform = new RigidBodyTransform();

         double x = 0.20;
         double y = robotSide.negateIfRightSide(0.35); // 0.30);
         double z = -0.40;
         Vector3D translation = new Vector3D(x, y, z);
         translation.scale(jointMap.getModelScale());
         transform.setTranslation(translation);

         RotationMatrix rotation = new RotationMatrix();
         double yaw = 0.0; // robotSide.negateIfRightSide(-1.7);
         double pitch = 0.7;
         double roll = 0.0; // robotSide.negateIfRightSide(-0.8);
         rotation.setYawPitchRoll(yaw, pitch, roll);
         transform.setRotation(rotation);

         handPosesWithRespectToChestFrame.put(robotSide, transform);
      }
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PIDGainsReadOnly>> getJointSpaceControlGains()
   {
      List<GroupParameter<PIDGainsReadOnly>> jointspaceGains = new ArrayList<>();
      jointspaceGains.add(new GroupParameter<>("SpineJoints", jointMap.getSpineJointNamesAsStrings()));
      jointspaceGains.add(new GroupParameter<>("NeckJoints", jointMap.getNeckJointNamesAsStrings()));
      jointspaceGains.add(new GroupParameter<>("ArmJoints", jointMap.getArmJointNamesAsStrings()));
      return jointspaceGains;
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PID3DConfiguration>> getTaskspaceOrientationControlGains()
   {
      List<GroupParameter<PID3DConfiguration>> taskspaceAngularGains = new ArrayList<>();

      PID3DConfiguration chestAngularGainConfiguration = createChestOrientationControlGains();
      taskspaceAngularGains.add(new GroupParameter<>("Chest", chestAngularGainConfiguration, jointMap.getChestName()));

      PID3DConfiguration headAngularGainConfiguration = createHeadOrientationControlGains();
      taskspaceAngularGains.add(new GroupParameter<>("Head", headAngularGainConfiguration, jointMap.getHeadName()));

      PID3DConfiguration handAngularGainConfiguration = createHandOrientationControlGains();
      taskspaceAngularGains.add(new GroupParameter<>("Hand", handAngularGainConfiguration, jointMap.getHandNames()));

      PID3DConfiguration pelvisAngularGainConfiguration = createPelvisOrientationControlGains();
      taskspaceAngularGains.add(new GroupParameter<>("Pelvis", pelvisAngularGainConfiguration, jointMap.getPelvisName()));

      return taskspaceAngularGains;
   }

   private PID3DConfiguration createPelvisOrientationControlGains()
   {
      double kpXY = 200.0;
      double kpZ = 200.0;
      double zeta = 1.0;
      double maxAccel = runningOnRealRobot ? 12.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 180.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kpXY, kpXY, kpZ);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return new PID3DConfiguration(GainCoupling.XY, false, gains);
   }

   private PID3DConfiguration createHeadOrientationControlGains()
   {
      double kp = 100.0;
      double zeta = 1.0;
      double maxAccel = runningOnRealRobot ? 6.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 60.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return new PID3DConfiguration(GainCoupling.XYZ, false, gains);
   }

   private PID3DConfiguration createChestOrientationControlGains()
   {
      double kpXY = 500.0;
      double kpZ = 500.0;
      double zetaXY = 1.0;
      double zetaZ = 1.0;
      double maxAccel = runningOnRealRobot ? 6.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 60.0 : Double.POSITIVE_INFINITY;
      double maxProportionalError = 10.0 * Math.PI / 180.0;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kpXY, kpXY, kpZ);
      gains.setDampingRatios(zetaXY, zetaXY, zetaZ);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);
      gains.setMaxProportionalError(maxProportionalError);

      return new PID3DConfiguration(GainCoupling.XY, false, gains);
   }

   private PID3DConfiguration createHandOrientationControlGains()
   {
      double kp = 200.0;
      double zeta = 1.0;
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return new PID3DConfiguration(GainCoupling.XYZ, false, gains);
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PID3DConfiguration>> getTaskspacePositionControlGains()
   {
      List<GroupParameter<PID3DConfiguration>> taskspaceLinearGains = new ArrayList<>();

      PID3DConfiguration handLinearGainConfiguration = createHandPositionControlGains();
      taskspaceLinearGains.add(new GroupParameter<>("Hand", handLinearGainConfiguration, jointMap.getHandNames()));

      PID3DConfiguration footLinearGains = createFootPositionControlGains();
      List<String> footGainBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         footGainBodies.add(jointMap.getFootName(robotSide));
      }
      taskspaceLinearGains.add(new GroupParameter<>("Foot", footLinearGains, footGainBodies));

      PID3DConfiguration chestLinearGains = createChestPositionControlGains();
      List<String> chestGainBodies = new ArrayList<>();
      chestGainBodies.add(jointMap.getChestName());
      taskspaceLinearGains.add(new GroupParameter<>("Chest", chestLinearGains, chestGainBodies));

      PID3DConfiguration headLinearGains = createHeadPositionControlGains();
      List<String> headGainBodies = new ArrayList<>();
      headGainBodies.add(jointMap.getHeadName());
      taskspaceLinearGains.add(new GroupParameter<>("Head", headLinearGains, headGainBodies));

      PID3DConfiguration pelvisLinearGains = createPelvisPositionControlGains();
      List<String> pelvisGainBodies = new ArrayList<>();
      pelvisGainBodies.add(jointMap.getPelvisName());
      taskspaceLinearGains.add(new GroupParameter<>("Pelvis", pelvisLinearGains, pelvisGainBodies));

      return taskspaceLinearGains;
   }

   public PID3DConfiguration createFootPositionControlGains()
   {
      double kpXY = 200.0;
      double zetaXYZ = 1.0;
      double kpZ = 200.0;

      double maxAccel = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 300.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kpXY, kpXY, kpZ);
      gains.setDampingRatios(zetaXYZ, zetaXYZ, zetaXYZ);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return new PID3DConfiguration(GainCoupling.XYZ, false, gains);
   }

   public PID3DConfiguration createHeadPositionControlGains()
   {
      double kpXY = 100.0;
      double zetaXYZ = 1.0;
      double kpZ = 100.0;

      double maxAccel = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 300.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kpXY, kpXY, kpZ);
      gains.setDampingRatios(zetaXYZ, zetaXYZ, zetaXYZ);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return new PID3DConfiguration(GainCoupling.XYZ, false, gains);
   }

   public PID3DConfiguration createChestPositionControlGains()
   {
      double kpXY = 100.0;
      double zetaXYZ = 1.0;
      double kpZ = 100.0;

      double maxAccel = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 300.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kpXY, kpXY, kpZ);
      gains.setDampingRatios(zetaXYZ, zetaXYZ, zetaXYZ);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return new PID3DConfiguration(GainCoupling.XYZ, false, gains);
   }

   private PID3DConfiguration createHandPositionControlGains()
   {
      double kp = 200.0;
      double zeta = 1.0;
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return new PID3DConfiguration(GainCoupling.XYZ, false, gains);
   }

   private PID3DConfiguration createPelvisPositionControlGains()
   {
      double kp = 200.0;
      double zeta = 1.0;
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return new PID3DConfiguration(GainCoupling.XYZ, false, gains);
   }

   /** {@inheritDoc} */
   @Override
   public Map<String, RigidBodyControlMode> getDefaultControlModesForRigidBodies()
   {
      Map<String, RigidBodyControlMode> defaultControlModes = new HashMap<>();
      defaultControlModes.put(jointMap.getChestName(), RigidBodyControlMode.TASKSPACE);
      return defaultControlModes;
   }

   /** {@inheritDoc} */
   @Override
   public TObjectDoubleHashMap<String> getOrCreateJointHomeConfiguration()
   {
      if (jointHomeConfiguration != null)
         return jointHomeConfiguration;

      jointHomeConfiguration = new TObjectDoubleHashMap<String>();

      jointHomeConfiguration.put(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 0.0);
      jointHomeConfiguration.put(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 0.0);
      jointHomeConfiguration.put(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 0.0);

      jointHomeConfiguration.put(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH), 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), robotSide.negateIfRightSide(0.785398));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(-0.52379));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), 2.33708);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), robotSide.negateIfRightSide(2.35619));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), -0.337807);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), robotSide.negateIfRightSide(0.207730));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SECOND_WRIST_PITCH), -0.026599);

         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 0.0);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 0.0);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), 0.0);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 0.0);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), 0.0);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), 0.0);
      }

      return jointHomeConfiguration;
   }

   /** {@inheritDoc} */
   @Override
   public Map<String, Pose3D> getOrCreateBodyHomeConfiguration()
   {
      if (bodyHomeConfiguration != null)
         return bodyHomeConfiguration;

      bodyHomeConfiguration = new HashMap<String, Pose3D>();

      Pose3D homeChestPoseInPelvisZUpFrame = new Pose3D();
      bodyHomeConfiguration.put(jointMap.getChestName(), homeChestPoseInPelvisZUpFrame);

      return bodyHomeConfiguration;
   }

   @Override
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return momentumOptimizationSettings;
   }

   /** {@inheritDoc} */
   @Override
   public JointPrivilegedConfigurationParameters getJointPrivilegedConfigurationParameters()
   {
      return jointPrivilegedConfigurationParameters;
   }

   @Override
   public double getMotionPlanningNodeTime()
   {
      return 0.05;
   }

   @Override
   public double getForceRegularizationWeight()
   {
      return 1e-5;
   }

   @Override
   public double getForceRateRegularizationWeight()
   {
      return 1e-5;
   }

   @Override
   public Vector3DReadOnly getMaxForce()
   {
      return maxForce;
   }

   @Override
   public Vector3DReadOnly getMinForce()
   {
      return minForce;
   }

   @Override
   public Vector3DReadOnly getMaxForceRate()
   {
      return maxForceRate;
   }

   @Override
   public Vector3DReadOnly getMinForceRate()
   {
      return minForceRate;
   }

   @Override
   public Vector3DReadOnly getPositionErrorGain()
   {
      return new Vector3D(100.0, 100.0, 200.0);
   }

   @Override
   public Vector3DReadOnly getVelocityErrorGain()
   {
      return new Vector3D(20.0, 20.0, 28.28);
   }

   @Override
   public double getNominalInertiaForTwistPlanning()
   {
      return 0.1;
   }
}
