package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import java.util.ArrayList;
import java.util.Random;

import org.apache.commons.lang3.StringUtils;
import org.ejml.data.DenseMatrix64F;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.StampedPosePacket;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.DiagnosticBehavior.DiagnosticTask;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ArmTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.GoHomeBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisHeightTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisOrientationTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.TurnInPlaceBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.taskExecutor.ArmTrajectoryTask;
import us.ihmc.humanoidBehaviors.taskExecutor.ChestOrientationTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FootTrajectoryTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FootstepListTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FootstepTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GoHomeTask;
import us.ihmc.humanoidBehaviors.taskExecutor.PelvisHeightTrajectoryTask;
import us.ihmc.humanoidBehaviors.taskExecutor.PelvisOrientationTrajectoryTask;
import us.ihmc.humanoidBehaviors.taskExecutor.PelvisTrajectoryTask;
import us.ihmc.humanoidBehaviors.taskExecutor.SleepTask;
import us.ihmc.humanoidBehaviors.taskExecutor.TurnInPlaceTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WalkToLocationTask;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.kinematics.NumericalInverseKinematicsCalculator;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.math.trajectories.generators.OneDoFTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.OneDoFTrajectoryPointList;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.taskExecutor.NullState;
import us.ihmc.robotics.taskExecutor.PipeLine;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.diagnostics.HumanoidArmPose;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class DiagnosticBehavior extends AbstractBehavior
{
   private static final boolean FAST_MOTION = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean DEBUG = false;
   private boolean initialized = false;

   private final PipeLine<AbstractBehavior> pipeLine;

   /**
    * FIXME Should have a packet from the controller to let know when it is ready to execute
    * commands.
    */
   private final ConcurrentListeningQueue<CapturabilityBasedStatus> inputListeningQueue = new ConcurrentListeningQueue<CapturabilityBasedStatus>(40);
   private final YoBoolean diagnosticBehaviorEnabled;
   private final YoBoolean hasControllerWakenUp;
   private final YoBoolean automaticDiagnosticRoutineRequested;
   private final YoBoolean automaticDiagnosticRoutineHasStarted;
   private final YoDouble timeWhenControllerWokeUp;
   private final YoDouble timeToWaitBeforeEnable;
   private final YoBoolean enableHandOrientation;
   private final YoBoolean canArmsReachFarBehind;

   private final SideDependentList<ArmTrajectoryBehavior> armTrajectoryBehaviors = new SideDependentList<>();
   private final SideDependentList<HandTrajectoryBehavior> handTrajectoryBehaviors = new SideDependentList<>();
   private final SideDependentList<GoHomeBehavior> armGoHomeBehaviors = new SideDependentList<>();
   private final FootTrajectoryBehavior footPoseBehavior;
   private final ChestTrajectoryBehavior chestTrajectoryBehavior;
   private final GoHomeBehavior chestGoHomeBehavior;
   private final PelvisTrajectoryBehavior pelvisTrajectoryBehavior;
   private final PelvisOrientationTrajectoryBehavior pelvisOrientationTrajectoryBehavior;
   private final GoHomeBehavior pelvisGoHomeBehavior;
   private final FootstepListBehavior footstepListBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final PelvisHeightTrajectoryBehavior pelvisHeightTrajectoryBehavior;
   private final TurnInPlaceBehavior turnInPlaceBehavior;

   private final YoDouble yoTime;
   private final YoDouble trajectoryTime, flyingTrajectoryTime;
   private final YoDouble sleepTimeBetweenPoses;
   private final YoBoolean doPelvisAndChestYaw;
   private final YoInteger numberOfCyclesToRun;
   private final YoDouble minCoMHeightOffset, maxCoMHeightOffset;
   private final int numberOfArmJoints;
   private final FullHumanoidRobotModel fullRobotModel;

   //Icp Offset generator variables
   private final YoBoolean isIcpOffsetSenderEnabled;
   private final YoDouble minMaxIcpAngularOffset;
   private final YoDouble minMaxIcpTranslationOffset;
   private final YoDouble previousIcpPacketSentTime;
   private final TimeStampedTransformBuffer stateEstimatorPelvisPoseBuffer;
   private final YoDouble icpTimeDelay;

   private final YoFrameConvexPolygon2D yoSupportPolygon;

   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<MovingReferenceFrame> ankleZUpFrames;
   private final SideDependentList<MovingReferenceFrame> soleZUpFrames;

   private final YoFrameVector2D pelvisShiftScaleFactor;

   private final SideDependentList<OneDoFJointBasics[]> upperArmJoints = new SideDependentList<OneDoFJointBasics[]>();
   private final SideDependentList<OneDoFJointBasics[]> lowerArmJoints = new SideDependentList<OneDoFJointBasics[]>();

   private final SideDependentList<OneDoFJointBasics[]> upperArmJointsClone = new SideDependentList<OneDoFJointBasics[]>();
   private final SideDependentList<OneDoFJointBasics[]> lowerArmJointsClone = new SideDependentList<OneDoFJointBasics[]>();

   private final SideDependentList<Double> elbowJointSign = new SideDependentList<>();

   private final WalkingControllerParameters walkingControllerParameters;

   public enum DiagnosticTask
   {
      CHEST_ROTATIONS,
      PELVIS_ROTATIONS,
      BOOTY_SHAKE,
      SHIFT_WEIGHT,
      COMBINED_CHEST_PELVIS,
      ARM_MOTIONS,
      ARM_SHAKE,
      UPPER_BODY,
      FOOT_LIFT,
      FOOT_POSES_SHORT,
      FOOT_POSES_LONG,
      RUNNING_MAN,
      BOW,
      KARATE_KID,
      WHOLE_SCHEBANG,
      SQUATS,
      SQUATATHON,
      SIMPLE_WARMUP,
      MEDIUM_WARMUP,
      HARD_WARMUP,
      STEPS_FORWARD_BACKWARD,
      STEPS_SHORT,
      STEPS_LONG,
      STEPS_IN_PLACE,
      TURN_IN_PLACE_SEQUENCE,
      TURN_IN_PLACE_ANGLE,
      FEET_SQUARE_UP,
      CUTE_WAVE,
      HAND_SHAKE_PREP,
      HAND_SHAKE_SHAKE,
      GO_HOME,
      FLEX_UP,
      FLEX_DOWN,
      FLEX_UP_FLEX_DOWN,
      KRANE_KICK,
      REDO_LAST_TASK // Keep that one at the end.
   };

   private final YoEnum<DiagnosticTask> lastDiagnosticTask;

   private final YoEnum<DiagnosticTask> requestedDiagnostic;
   private final YoEnum<HumanoidArmPose> requestedSymmetricArmPose;
   private final YoEnum<HumanoidArmPose> requestedSingleArmPose;
   private final YoEnum<RobotSide> activeSideForHandControl;
   private final YoEnum<RobotSide> activeSideForFootControl;
   private final YoEnum<RobotSide> supportLeg;

   private final double maxPitchBackward = Math.toRadians(-5.0);
   private final double maxPitchForward = Math.toRadians(40.0);
   private final double minMaxRoll = Math.toRadians(15.0);
   private final double minMaxYaw = Math.toRadians(30.0);

   private final YoDouble footstepLength;
   private final YoDouble swingTime;
   private final YoDouble transferTime;

   private final YoDouble maxFootPoseHeight;
   private final YoDouble maxFootPoseDisplacement;

   private final YoDouble angleToTurnInDegrees;

   private final SideDependentList<ReferenceFrame> upperArmsFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> lowerArmsFrames = new SideDependentList<>();
   private final SideDependentList<NumericalInverseKinematicsCalculator> inverseKinematicsForUpperArms = new SideDependentList<>();
   private final SideDependentList<NumericalInverseKinematicsCalculator> inverseKinematicsForLowerArms = new SideDependentList<>();

   private final SideDependentList<YoFrameYawPitchRoll> currentUpperArmOrientations = new SideDependentList<YoFrameYawPitchRoll>();
   private final SideDependentList<YoFrameYawPitchRoll> currentHandOrientations = new SideDependentList<YoFrameYawPitchRoll>();

   private final SideDependentList<RigidBodyTransform> armZeroJointAngleConfigurationOffsets = new SideDependentList<>();

   private final YoDouble pelvisOrientationScaleFactor = new YoDouble("diagnosticBehaviorPelvisOrientationScaleFactor", registry);
   private final YoDouble bootyShakeTime = new YoDouble("diagnosticBehaviorButtyShakeTime", registry);

   private final IHMCROS2Publisher<StampedPosePacket> stampedPosePublisher;

   public DiagnosticBehavior(String robotName, FullHumanoidRobotModel fullRobotModel, YoEnum<RobotSide> supportLeg, HumanoidReferenceFrames referenceFrames,
                             YoDouble yoTime, YoBoolean yoDoubleSupport, Ros2Node ros2Node, WholeBodyControllerParameters wholeBodyControllerParameters, FootstepPlannerParametersBasics footstepPlannerParameters,
                             YoFrameConvexPolygon2D yoSupportPolygon, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, ros2Node);
      pipeLine = new PipeLine<>(yoTime);
      this.supportLeg = supportLeg;
      this.fullRobotModel = fullRobotModel;
      this.yoSupportPolygon = yoSupportPolygon;
      this.walkingControllerParameters = wholeBodyControllerParameters.getWalkingControllerParameters();

      diagnosticBehaviorEnabled = new YoBoolean("diagnosticBehaviorEnabled", registry);
      hasControllerWakenUp = new YoBoolean("diagnostBehaviorHasControllerWakenUp", registry);
      automaticDiagnosticRoutineRequested = new YoBoolean("diagnosticBehaviorAutomaticDiagnosticRoutineRequested", registry);
      automaticDiagnosticRoutineHasStarted = new YoBoolean("diagnosticBehaviorAutomaticDiagnosticRoutineHasStarted", registry);
      timeWhenControllerWokeUp = new YoDouble("diagnosticBehaviorTimeWhenControllerWokeUp", registry);
      timeToWaitBeforeEnable = new YoDouble("diagnosticBehaviorTimeToWaitBeforeEnable", registry);
      enableHandOrientation = new YoBoolean("diagnosticEnableHandOrientation", registry);
      canArmsReachFarBehind = new YoBoolean("diagnosticCanArmsReachFarBehind", registry);

      numberOfArmJoints = fullRobotModel.getRobotSpecificJointNames().getArmJointNames().length;
      this.yoTime = yoTime;
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      soleZUpFrames = referenceFrames.getSoleZUpFrames();

      //icp variables
      isIcpOffsetSenderEnabled = new YoBoolean("DiagnosticBehaviorIcpOffsetSenderEnabled", registry);
      isIcpOffsetSenderEnabled.set(false);
      isIcpOffsetSenderEnabled.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            DiagnosticBehavior.this.previousIcpPacketSentTime.set(DiagnosticBehavior.this.yoTime.getDoubleValue());

         }
      });
      minMaxIcpAngularOffset = new YoDouble(getName() + "MinMaxIcpAngularOffset", registry);
      minMaxIcpAngularOffset.set(0.0);
      minMaxIcpTranslationOffset = new YoDouble(getName() + "MinMaxIcpTranslationOffset", registry);
      minMaxIcpTranslationOffset.set(0.06);
      previousIcpPacketSentTime = new YoDouble("DiagnosticBehaviorPreviousIcpPacketSentTime", registry);
      stateEstimatorPelvisPoseBuffer = new TimeStampedTransformBuffer(10000);
      icpTimeDelay = new YoDouble(getName() + "IcpTimeDelay", registry);
      icpTimeDelay.set(0.99);
      ///////////////////

      String behaviorNameFirstLowerCase = StringUtils.uncapitalize(getName());
      trajectoryTime = new YoDouble(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      flyingTrajectoryTime = new YoDouble(behaviorNameFirstLowerCase + "flyingTrajectoryTime", registry);

      swingTime = new YoDouble(behaviorNameFirstLowerCase + "SwingTime", registry);
      swingTime.set(walkingControllerParameters.getDefaultSwingTime());
      transferTime = new YoDouble(behaviorNameFirstLowerCase + "TransferTime", registry);
      transferTime.set(walkingControllerParameters.getDefaultTransferTime());

      maxFootPoseHeight = new YoDouble(behaviorNameFirstLowerCase + "MaxFootPoseHeight", registry);
      maxFootPoseHeight.set(0.1);
      maxFootPoseDisplacement = new YoDouble(behaviorNameFirstLowerCase + "maxFootPoseDisplacement", registry);
      maxFootPoseDisplacement.set(0.2);
      angleToTurnInDegrees = new YoDouble(behaviorNameFirstLowerCase + "AngleToTurnInDegrees", registry);
      angleToTurnInDegrees.set(0.0);

      trajectoryTime.set(FAST_MOTION ? 0.5 : 3.0);
      flyingTrajectoryTime.set(FAST_MOTION ? 0.5 : 10.0);
      sleepTimeBetweenPoses = new YoDouble(behaviorNameFirstLowerCase + "SleepTimeBetweenPoses", registry);
      sleepTimeBetweenPoses.set(FAST_MOTION ? 0.0 : 0.5);

      minCoMHeightOffset = new YoDouble(behaviorNameFirstLowerCase + "MinCoMHeightOffset", registry);
      minCoMHeightOffset.set(-0.15);
      maxCoMHeightOffset = new YoDouble(behaviorNameFirstLowerCase + "MaxCoMHeightOffset", registry);
      maxCoMHeightOffset.set(0.05);

      footstepLength = new YoDouble(behaviorNameFirstLowerCase + "FootstepLength", registry);
      footstepLength.set(0.3);

      bootyShakeTime.set(1.0);

      walkToLocationBehavior = new WalkToLocationBehavior(robotName, ros2Node, fullRobotModel, referenceFrames, walkingControllerParameters);
      registry.addChild(walkToLocationBehavior.getYoVariableRegistry());

      chestTrajectoryBehavior = new ChestTrajectoryBehavior(robotName, ros2Node, yoTime);
      registry.addChild(chestTrajectoryBehavior.getYoVariableRegistry());

      chestGoHomeBehavior = new GoHomeBehavior(robotName, "chest", ros2Node, yoTime);
      registry.addChild(chestGoHomeBehavior.getYoVariableRegistry());

      pelvisTrajectoryBehavior = new PelvisTrajectoryBehavior(robotName, ros2Node, yoTime);
      registry.addChild(pelvisTrajectoryBehavior.getYoVariableRegistry());

      pelvisOrientationTrajectoryBehavior = new PelvisOrientationTrajectoryBehavior(robotName, ros2Node, yoTime);
      registry.addChild(pelvisOrientationTrajectoryBehavior.getYoVariableRegistry());

      pelvisGoHomeBehavior = new GoHomeBehavior(robotName, "pelvis", ros2Node, yoTime);
      registry.addChild(pelvisGoHomeBehavior.getYoVariableRegistry());

      footPoseBehavior = new FootTrajectoryBehavior(robotName, ros2Node, yoTime, yoDoubleSupport);
      registry.addChild(footPoseBehavior.getYoVariableRegistry());

      footstepListBehavior = new FootstepListBehavior(robotName, ros2Node, walkingControllerParameters);
      registry.addChild(footstepListBehavior.getYoVariableRegistry());

      pelvisHeightTrajectoryBehavior = new PelvisHeightTrajectoryBehavior(robotName, ros2Node, yoTime);
      registry.addChild(pelvisHeightTrajectoryBehavior.getYoVariableRegistry());

      turnInPlaceBehavior = new TurnInPlaceBehavior(robotName, ros2Node, fullRobotModel, referenceFrames, walkingControllerParameters,footstepPlannerParameters, yoTime);
      registry.addChild(turnInPlaceBehavior.getYoVariableRegistry());

      for (RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = robotSide.getCamelCaseNameForMiddleOfExpression();
         ArmTrajectoryBehavior armTrajectoryBehavior = new ArmTrajectoryBehavior(robotName, namePrefix, ros2Node, yoTime);
         registry.addChild(armTrajectoryBehavior.getYoVariableRegistry());
         armTrajectoryBehaviors.put(robotSide, armTrajectoryBehavior);

         HandTrajectoryBehavior handTrajectoryBehavior = new HandTrajectoryBehavior(robotName, namePrefix, ros2Node, yoTime);
         registry.addChild(handTrajectoryBehavior.getYoVariableRegistry());
         handTrajectoryBehaviors.put(robotSide, handTrajectoryBehavior);

         GoHomeBehavior armGoHomeBehavior = new GoHomeBehavior(robotName, namePrefix + "Arm", ros2Node, yoTime);
         registry.addChild(armGoHomeBehavior.getYoVariableRegistry());
         armGoHomeBehaviors.put(robotSide, armGoHomeBehavior);
      }

      requestedDiagnostic = new YoEnum<>("requestedDiagnostic", registry, DiagnosticTask.class, true);
      requestedDiagnostic.set(null);

      lastDiagnosticTask = new YoEnum<>("lastDiagnosticTask", registry, DiagnosticTask.class, true);
      lastDiagnosticTask.set(null);

      requestedSymmetricArmPose = new YoEnum<>("requestedSymmetricArmPose", registry, HumanoidArmPose.class, true);
      requestedSymmetricArmPose.set(null);

      requestedSingleArmPose = new YoEnum<>("requestedSingleArmPose", registry, HumanoidArmPose.class, true);
      requestedSingleArmPose.set(null);

      activeSideForFootControl = new YoEnum<>("activeSideForFootControl", registry, RobotSide.class, true);
      activeSideForFootControl.set(RobotSide.LEFT);

      activeSideForHandControl = new YoEnum<>("activeSideForHandControl", registry, RobotSide.class, true);
      activeSideForHandControl.set(RobotSide.LEFT);

      numberOfCyclesToRun = new YoInteger("numberOfDiagnosticCyclesToRun", registry);
      numberOfCyclesToRun.set(1);

      doPelvisAndChestYaw = new YoBoolean("diagnosticDoPelvisAndChestYaw", registry);
      doPelvisAndChestYaw.set(true);

      pelvisShiftScaleFactor = new YoFrameVector2D("DiagnosticPelvisShiftScaleFactor", null, registry);
      pelvisShiftScaleFactor.set(0.4, 0.7);

      pelvisOrientationScaleFactor.set(0.1);

      setupArmsInverseKinematics(fullRobotModel);

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

         YoFrameYawPitchRoll currentUpperArmOrientation = new YoFrameYawPitchRoll(sidePrefix + "CurrentUpperArm", chestFrame, registry);
         currentUpperArmOrientations.put(robotSide, currentUpperArmOrientation);

         YoFrameYawPitchRoll currentHandOrientation = new YoFrameYawPitchRoll(sidePrefix + "CurrentHand", lowerArmsFrames.get(robotSide), registry);
         currentHandOrientations.put(robotSide, currentHandOrientation);
      }

      createSubscriberFromController(CapturabilityBasedStatus.class, inputListeningQueue::put);
      new SleepBehavior(robotName, ros2Node, yoTime);

      stampedPosePublisher = createPublisherForController(StampedPosePacket.class);
   }

   private void setupArmsInverseKinematics(FullHumanoidRobotModel fullRobotModel)
   {
      // These values were tuned by Jerry Pratt on February 24, 2015 to match Atlas the best.
      int maxIterations = 1000; // 60 Seems to be a bit too low, 100 seems to be enough, just set it to 200 to make sure (Sylvain)
      double lambdaLeastSquares = 0.0009;
      double tolerance = 0.0025;
      double maxStepSize = 0.2;
      double minRandomSearchScalar = 0.01;
      double maxRandomSearchScalar = 0.8;

      DenseMatrix64F angularSelectionMatrix = new DenseMatrix64F(3, SpatialVector.SIZE);
      angularSelectionMatrix.set(0, 0, 1.0);
      angularSelectionMatrix.set(1, 1, 1.0);
      angularSelectionMatrix.set(2, 2, 1.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

         if (hand == null)
            continue;

         // The following one works for Valkyrie but doesn't work for Atlas
         //         RigidBody upperArmBody = fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH).getPredecessor();
         // Pretty hackish but will work for now: Consider the elbow joint to be the fourth joint of the chain
         OneDoFJointBasics[] armJoints = MultiBodySystemTools.filterJoints(MultiBodySystemTools.createJointPath(chest, hand), OneDoFJointBasics.class);
         OneDoFJointBasics elbowJoint = armJoints[3];
         double jointSign = -Math.signum(elbowJoint.getJointLimitLower() + elbowJoint.getJointLimitUpper());
         elbowJointSign.put(robotSide, jointSign);

         RigidBodyBasics upperArmBody = elbowJoint.getPredecessor();
         RigidBodyBasics lowerArmBody = elbowJoint.getSuccessor();

         upperArmsFrames.put(robotSide, upperArmBody.getBodyFixedFrame());
         lowerArmsFrames.put(robotSide, lowerArmBody.getBodyFixedFrame());

         FramePoint3D tempPoint = new FramePoint3D(hand.getParentJoint().getFrameAfterJoint());
         tempPoint.changeFrame(armJoints[1].getFrameAfterJoint());
         FrameVector3D tempVector = new FrameVector3D(tempPoint);
         EuclidCoreMissingTools.floorToGivenPrecision(tempVector, 1.0e-2);
         tempVector.normalize();

         Vector3D expectedArmZeroConfiguration = new Vector3D(0.0, robotSide.negateIfRightSide(1.0), 0.0);
         RigidBodyTransform armZeroJointAngleConfigurationOffset = new RigidBodyTransform();
         if (tempVector.dot(expectedArmZeroConfiguration) > 1.0 - 1e-5)
         {
            armZeroJointAngleConfigurationOffset.setIdentity();
         }
         else
         {
            AxisAngle rotation = new AxisAngle();
            EuclidGeometryTools.axisAngleFromFirstToSecondVector3D(expectedArmZeroConfiguration, tempVector, rotation);
            armZeroJointAngleConfigurationOffset.setRotation(rotation);
         }

         Vector3D expectedElbowAxis = new Vector3D(0.0, 0.0, 1.0);
         RigidBodyTransform zRotationDueToAccountForElbowAxis = new RigidBodyTransform();
         FrameVector3D elbowJointAxis = new FrameVector3D(elbowJoint.getJointAxis());
         zRotationDueToAccountForElbowAxis.setRotationPitchAndZeroTranslation(-elbowJointAxis.angle(expectedElbowAxis));
         armZeroJointAngleConfigurationOffset.multiply(zRotationDueToAccountForElbowAxis);

         armZeroJointAngleConfigurationOffset.invert();
         armZeroJointAngleConfigurationOffsets.put(robotSide, armZeroJointAngleConfigurationOffset);

         upperArmJoints.put(robotSide, MultiBodySystemTools.filterJoints(MultiBodySystemTools.createJointPath(chest, upperArmBody), OneDoFJointBasics.class));
         upperArmJointsClone.put(robotSide, MultiBodySystemTools.filterJoints(MultiBodySystemFactories.cloneKinematicChain(upperArmJoints.get(robotSide)),
                                                                              OneDoFJointBasics.class));
         GeometricJacobian upperArmJacobian = new GeometricJacobian(upperArmJointsClone.get(robotSide),
                                                                    upperArmJointsClone.get(robotSide)[upperArmJointsClone.get(robotSide).length
                                                                          - 1].getSuccessor().getBodyFixedFrame());
         NumericalInverseKinematicsCalculator inverseKinematicsForUpperArm = new NumericalInverseKinematicsCalculator(upperArmJacobian, lambdaLeastSquares,
                                                                                                                      tolerance, maxIterations, maxStepSize,
                                                                                                                      minRandomSearchScalar,
                                                                                                                      maxRandomSearchScalar);
         inverseKinematicsForUpperArm.setSelectionMatrix(angularSelectionMatrix);
         inverseKinematicsForUpperArms.put(robotSide, inverseKinematicsForUpperArm);

         lowerArmJoints.put(robotSide, MultiBodySystemTools.filterJoints(MultiBodySystemTools.createJointPath(lowerArmBody, hand), OneDoFJointBasics.class));
         lowerArmJointsClone.put(robotSide, MultiBodySystemTools.filterJoints(MultiBodySystemFactories.cloneKinematicChain(lowerArmJoints.get(robotSide)),
                                                                              OneDoFJointBasics.class));
         GeometricJacobian lowerArmJacobian = new GeometricJacobian(lowerArmJointsClone.get(robotSide),
                                                                    lowerArmJointsClone.get(robotSide)[lowerArmJointsClone.get(robotSide).length
                                                                          - 1].getSuccessor().getBodyFixedFrame());
         NumericalInverseKinematicsCalculator inverseKinematicsForLowerArm = new NumericalInverseKinematicsCalculator(lowerArmJacobian, lambdaLeastSquares,
                                                                                                                      tolerance, maxIterations, maxStepSize,
                                                                                                                      minRandomSearchScalar,
                                                                                                                      maxRandomSearchScalar);
         inverseKinematicsForLowerArm.setSelectionMatrix(angularSelectionMatrix);
         inverseKinematicsForLowerArms.put(robotSide, inverseKinematicsForLowerArm);
      }
   }

   public void setCanArmsReachFarBehind(boolean value)
   {
      canArmsReachFarBehind.set(value);
   }

   public void setupForAutomaticDiagnostic(double timeToWait)
   {
      automaticDiagnosticRoutineRequested.set(true);
      timeToWaitBeforeEnable.set(timeToWait); // To make sure that the transition from strand prep to walking is done.
      System.out.println("\n");
      System.out.println("///////////////////////////////////////////////////////////");
      System.out.println("//       Initializing automatic diagnostic routine       //");
      System.out.println("//        Waiting for walking controller to start        //");
      System.out.println("///////////////////////////////////////////////////////////");
      System.out.println("");

   }

   private void automaticDiagnosticRoutine()
   {
      sequenceSimpleWarmup();
   }

   private void sequenceSimpleWarmup()
   {
      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
         sequenceSquats();
      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
         sequenceChestRotations(0.35); //55);
      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
         sequencePelvisRotations(0.2); //3);
      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
         sequenceShiftWeight();
   }

   private void sequenceMediumWarmup()
   {
      FramePoint2D center = new FramePoint2D(midFeetZUpFrame);
      FrameVector2D shiftScaleVector = new FrameVector2D(midFeetZUpFrame, 0.1, 0.7);

      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D(yoSupportPolygon);
      supportPolygon.changeFrameAndProjectToXYPlane(midFeetZUpFrame);

      FramePoint2D desiredPelvisOffset = new FramePoint2D(midFeetZUpFrame);

      for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
      {
         desiredPelvisOffset.setIncludingFrame(supportPolygon.getVertex(i));
         desiredPelvisOffset.sub(center);
         submitDesiredPelvisPositionOffset(false, shiftScaleVector.getX() * desiredPelvisOffset.getX(), shiftScaleVector.getY() * desiredPelvisOffset.getY(),
                                           0.0);
         sequenceSquats();
         sequenceChestRotations(0.55); //TODO increase/decrease limit?
         sequencePelvisRotations(0.3); //TODO increase/decrease limit?
      }
      // Get back to the first vertex again
      desiredPelvisOffset.setIncludingFrame(supportPolygon.getVertex(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(false, pelvisShiftScaleFactor.getX() * desiredPelvisOffset.getX(),
                                        pelvisShiftScaleFactor.getY() * desiredPelvisOffset.getY(), 0.0);

      submitChestHomeCommand(false);
      submitPelvisHomeCommand(false);
   }

   private final FramePoint2D frameVertexBefore = new FramePoint2D();
   private final FramePoint2D frameVertexCurrentlyChecked = new FramePoint2D();
   private final FramePoint2D frameVertexAfter = new FramePoint2D();

   private void sequenceHardWarmup()
   {
      //chest rotation closer to the limits
      sequenceChestRotations(0.80);

      //pelvis rotations closer to the limits
      sequencePelvisRotations(0.55);

      //get the 4 corners of the double support polygon (the feet are supposedly aligned)
      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D(yoSupportPolygon);
      supportPolygon.changeFrameAndProjectToXYPlane(midFeetZUpFrame);
      int numberOfVertices = supportPolygon.getNumberOfVertices();
      ArrayList<FramePoint2D> supportCornerPoints = new ArrayList<>();

      for (int i = 0; i < numberOfVertices; i++)
      {
         frameVertexBefore.setIncludingFrame(supportPolygon.getVertex(i));
         frameVertexCurrentlyChecked.setIncludingFrame(supportPolygon.getVertex((i + 1) % numberOfVertices));
         frameVertexAfter.setIncludingFrame(supportPolygon.getVertex((i + 2) % numberOfVertices));

         FrameVector2D frameVector1 = new FrameVector2D(midFeetZUpFrame);
         frameVector1.sub(frameVertexCurrentlyChecked, frameVertexBefore);
         frameVector1.normalize();

         FrameVector2D frameVector2 = new FrameVector2D(midFeetZUpFrame);
         frameVector2.sub(frameVertexAfter, frameVertexCurrentlyChecked);
         frameVector2.normalize();

         if (Math.abs(frameVector1.angle(frameVector2)) > Math.PI / 2.0 - 0.2 && Math.abs(frameVector1.angle(frameVector2)) < Math.PI / 2.0 + 0.2)
            supportCornerPoints.add(frameVertexCurrentlyChecked);
      }

      // scale the rectangle so that the center of pressure does not go too far on the support polygon sides
      FrameVector2D shiftScaleVector = new FrameVector2D(midFeetZUpFrame, 0.1, 0.7);
      for (int i = 0; i < supportCornerPoints.size(); i++)
         supportCornerPoints.get(i).scale(shiftScaleVector.getX(), shiftScaleVector.getY());

      ///////////   combinations of doom   ////////////
      //shiftWeight + pelvisOrientation
      FramePoint3D currentPelvisHeight = new FramePoint3D(pelvisZUpFrame);
      currentPelvisHeight.changeFrame(worldFrame);
      FrameVector2D desiredPelvisOffset = new FrameVector2D(midFeetZUpFrame);
      FramePoint2D center = new FramePoint2D(midFeetZUpFrame);

      double yawPercentage = 0.3;
      double pitchPercentage = 0.3;
      double rollPercentage = 0.3;

      desiredPelvisOffset.set(supportCornerPoints.get(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(false, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchBackward, rollPercentage * minMaxRoll);

      desiredPelvisOffset.set(supportCornerPoints.get(1));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(false, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, -yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchBackward, rollPercentage * minMaxRoll);

      desiredPelvisOffset.set(supportCornerPoints.get(2));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(false, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);

      desiredPelvisOffset.set(supportCornerPoints.get(3));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(false, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, -yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);

      submitPelvisHomeCommand(false);

      //shiftWeight + CoMHeight
      desiredPelvisOffset.set(supportCornerPoints.get(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitDesiredPelvisHeight(true, minCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(1));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitDesiredPelvisHeight(true, maxCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(2));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitDesiredPelvisHeight(true, minCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(3));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitDesiredPelvisHeight(true, maxCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitDesiredPelvisHeight(true, minCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(2));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitDesiredPelvisHeight(true, maxCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(1));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitDesiredPelvisHeight(true, minCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      submitPelvisHomeCommand(true);

      //shiftWeight + chestOrientation
      desiredPelvisOffset.set(supportCornerPoints.get(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitDesiredChestOrientation(true, yawPercentage * minMaxYaw, pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(2));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitDesiredChestOrientation(true, -yawPercentage * minMaxYaw, pitchPercentage * maxPitchForward, rollPercentage * minMaxRoll);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(1));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitDesiredChestOrientation(true, yawPercentage * minMaxYaw, pitchPercentage * maxPitchBackward, -rollPercentage * minMaxRoll);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(3));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitDesiredChestOrientation(true, -yawPercentage * minMaxYaw, pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(2));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitDesiredChestOrientation(true, -yawPercentage * minMaxYaw, pitchPercentage * maxPitchBackward, rollPercentage * minMaxRoll);
      pipeLine.requestNewStage();

      submitChestHomeCommand(true);
      submitPelvisHomeCommand(true);

      //shiftWeight + arms
      desiredPelvisOffset.set(supportCornerPoints.get(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitSymmetricHumanoidArmPose(HumanoidArmPose.LARGE_CHICKEN_WINGS);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(2));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitSymmetricHumanoidArmPose(HumanoidArmPose.ARM_FORTFIVE_ELBOW_DOWN);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(1));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitSymmetricHumanoidArmPose(HumanoidArmPose.ARMS_OUT_EXTENDED);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(3));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitSymmetricHumanoidArmPose(HumanoidArmPose.FLYING);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(2));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitSymmetricHumanoidArmPose(HumanoidArmPose.SMALL_CHICKEN_WINGS);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(1));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitSymmetricHumanoidArmPose(HumanoidArmPose.ARM_NINETY_ELBOW_UP);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(3));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0);
      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);
      pipeLine.requestNewStage();

      submitPelvisHomeCommand(true);
      submitArmGoHomeCommand(true);

      //Mean stuff  (shiftWeight + CoM + chestOrientation + PelvisOrientation)
      desiredPelvisOffset.set(supportCornerPoints.get(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchForward, rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, yawPercentage * minMaxYaw, pitchPercentage * maxPitchBackward, -rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, maxCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(2));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, -yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchBackward, -rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, yawPercentage * minMaxYaw, pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, minCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(1));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, -yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, -yawPercentage * minMaxYaw, pitchPercentage * maxPitchForward, rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, maxCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(3));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, -yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, -yawPercentage * minMaxYaw, pitchPercentage * maxPitchBackward, -rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, minCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(2));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchBackward, rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, yawPercentage * minMaxYaw, pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, maxCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, -yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchForward, rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, yawPercentage * minMaxYaw, pitchPercentage * maxPitchBackward, rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, minCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(3));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchBackward, -rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, -yawPercentage * minMaxYaw, pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, maxCoMHeightOffset.getDoubleValue());
      pipeLine.requestNewStage();

      submitChestHomeCommand(true);
      submitPelvisHomeCommand(true);
      submitDesiredPelvisHeight(true, 0.0);

      //really mean stuff (arms + CoM + shiftWeight + chestOrientation + PelvisOrientation)
      desiredPelvisOffset.set(supportCornerPoints.get(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, yawPercentage * minMaxYaw, pitchPercentage * maxPitchBackward, -rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, maxCoMHeightOffset.getDoubleValue());
      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(2));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, -yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchBackward, rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, yawPercentage * minMaxYaw, pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, minCoMHeightOffset.getDoubleValue());
      submitSymmetricHumanoidArmPose(HumanoidArmPose.ARM_STRAIGHT_DOWN);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(1));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, -yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, -yawPercentage * minMaxYaw, pitchPercentage * maxPitchForward, rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, maxCoMHeightOffset.getDoubleValue());
      submitSymmetricHumanoidArmPose(HumanoidArmPose.FLYING);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(3));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, -yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, -yawPercentage * minMaxYaw, pitchPercentage * maxPitchBackward, -rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, minCoMHeightOffset.getDoubleValue());
      submitSymmetricHumanoidArmPose(HumanoidArmPose.SUPER_CHICKEN_WINGS);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(2));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchBackward, rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, yawPercentage * minMaxYaw, pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, maxCoMHeightOffset.getDoubleValue());
      submitSymmetricHumanoidArmPose(HumanoidArmPose.LARGER_CHICKEN_WINGS);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, -yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchForward, rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, yawPercentage * minMaxYaw, pitchPercentage * maxPitchBackward, rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, minCoMHeightOffset.getDoubleValue());
      submitSymmetricHumanoidArmPose(HumanoidArmPose.ARM_FORTFIVE_ELBOW_UP3);
      pipeLine.requestNewStage();

      desiredPelvisOffset.set(supportCornerPoints.get(3));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffsetAndOrientation(true, desiredPelvisOffset.getX(), desiredPelvisOffset.getY(), 0.0, yawPercentage * minMaxYaw,
                                                      pitchPercentage * maxPitchBackward, -rollPercentage * minMaxRoll);
      submitDesiredChestOrientation(true, -yawPercentage * minMaxYaw, pitchPercentage * maxPitchForward, -rollPercentage * minMaxRoll);
      submitDesiredPelvisHeight(true, maxCoMHeightOffset.getDoubleValue());
      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);
      pipeLine.requestNewStage();

      submitChestHomeCommand(true);
      submitPelvisHomeCommand(true);
      submitArmGoHomeCommand(true);
   }

   private void sequenceUpperBody()
   {
      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
      {
         sequenceArmPose(activeSideForHandControl.getEnumValue());

         FrameQuaternion desiredUpperArmOrientation = new FrameQuaternion(fullRobotModel.getChest().getBodyFixedFrame());

         submitSymmetricHumanoidArmPose(HumanoidArmPose.LARGE_CHICKEN_WINGS);
         sequenceChestRotations(0.55);
         sequencePelvisRotations(0.3);
         sequenceMovingChestAndPelvisOnly();

         submitSymmetricHumanoidArmPose(HumanoidArmPose.REACH_FAR_FORWARD);
         sequenceChestRotations(0.55);
         sequencePelvisRotations(0.3);
         sequenceMovingChestAndPelvisOnly();

         submitSymmetricHumanoidArmPose(HumanoidArmPose.REACH_FAR_BACK);
         sequenceChestRotations(0.55);
         sequencePelvisRotations(0.3);
         sequenceMovingChestAndPelvisOnly();

         desiredUpperArmOrientation.setYawPitchRoll(0.0, 0.0, 0.0);
         submitHandPose(RobotSide.LEFT, desiredUpperArmOrientation, 0.0, null, true);
         submitHumanoidArmPose(RobotSide.RIGHT, HumanoidArmPose.ARM_STRAIGHT_DOWN);
         sequenceChestRotations(0.55);
         sequencePelvisRotations(0.3);
         sequenceMovingChestAndPelvisOnly();

         submitHumanoidArmPose(RobotSide.LEFT, HumanoidArmPose.ARM_STRAIGHT_DOWN);
         desiredUpperArmOrientation.setYawPitchRoll(0.0, 0.0, 0.0);
         submitHandPose(RobotSide.RIGHT, desiredUpperArmOrientation, 0.0, null, true);
         sequenceChestRotations(0.55);
         sequencePelvisRotations(0.3);
         sequenceMovingChestAndPelvisOnly();

         submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);
      }
   }

   private void sequenceGoHome()
   {
      submitPelvisHomeCommand(true);
      submitArmGoHomeCommand(true);
      submitChestHomeCommand(true);
   }

   private void sequenceChestRotations(double percentOfJointLimit)
   {
      double roll = percentOfJointLimit * minMaxRoll;
      submitDesiredChestOrientation(false, 0.0, percentOfJointLimit * maxPitchForward, 0.0);
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredChestOrientation(false, minMaxYaw, percentOfJointLimit * maxPitchForward, 0.0);
         submitDesiredChestOrientation(false, -minMaxYaw, percentOfJointLimit * maxPitchForward, 0.0);
      }
      submitDesiredChestOrientation(false, 0.0, 0.0, roll);
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredChestOrientation(false, minMaxYaw, 0.0, roll);
         submitDesiredChestOrientation(false, -minMaxYaw, 0.0, roll);
      }
      submitDesiredChestOrientation(false, 0.0, 0.0, -roll);
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredChestOrientation(false, minMaxYaw, 0.0, -roll);
         submitDesiredChestOrientation(false, -minMaxYaw, 0.0, -roll);
      }
      submitDesiredChestOrientation(false, 0.0, percentOfJointLimit * maxPitchForward, roll);
      submitDesiredChestOrientation(false, 0.0, percentOfJointLimit * maxPitchForward, -roll);

      submitDesiredChestOrientation(false, 0.0, 0.0, 0.0);
   }

   private void sequencePelvisRotations(double percentOfJointLimit)
   {
      double roll = percentOfJointLimit * minMaxRoll;
      double yaw = percentOfJointLimit * minMaxYaw;
      submitDesiredPelvisOrientation(false, 0.0, percentOfJointLimit * maxPitchForward, 0.0);
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredPelvisOrientation(false, yaw, percentOfJointLimit * maxPitchForward, 0.0);
         submitDesiredPelvisOrientation(false, -yaw, percentOfJointLimit * maxPitchForward, 0.0);
      }
      submitDesiredPelvisOrientation(false, 0.0, 0.0, roll);
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredPelvisOrientation(false, yaw, 0.0, roll);
         submitDesiredPelvisOrientation(false, -yaw, 0.0, roll);
      }
      submitDesiredPelvisOrientation(false, 0.0, 0.0, -roll);
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredPelvisOrientation(false, yaw, 0.0, -roll);
         submitDesiredPelvisOrientation(false, -yaw, 0.0, -roll);
      }
      submitDesiredPelvisOrientation(false, 0.0, percentOfJointLimit * maxPitchForward, roll);
      submitDesiredPelvisOrientation(false, 0.0, percentOfJointLimit * maxPitchForward, -roll);

      submitPelvisHomeCommand(false);
   }

   private void sequenceShiftWeight()
   {
      FramePoint2D center = new FramePoint2D(midFeetZUpFrame);

      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D(yoSupportPolygon);
      supportPolygon.changeFrameAndProjectToXYPlane(midFeetZUpFrame);

      FramePoint2D desiredPelvisOffset = new FramePoint2D(midFeetZUpFrame);

      for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
      {
         desiredPelvisOffset.setIncludingFrame(supportPolygon.getVertex(i));
         desiredPelvisOffset.sub(center);
         submitDesiredPelvisPositionOffset(false, pelvisShiftScaleFactor.getX() * desiredPelvisOffset.getX(),
                                           pelvisShiftScaleFactor.getY() * desiredPelvisOffset.getY(), 0.0);
      }
      // Get back to the first vertex again
      desiredPelvisOffset.setIncludingFrame(supportPolygon.getVertex(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(false, pelvisShiftScaleFactor.getX() * desiredPelvisOffset.getX(),
                                        pelvisShiftScaleFactor.getY() * desiredPelvisOffset.getY(), 0.0);

      submitPelvisHomeCommand(false);
   }

   private void sequenceMovingChestAndPelvisOnly()
   {
      double percentOfJointLimit = 0.8;
      double percentOfJointLimitForPelvis = 0.5;
      submitDesiredChestOrientation(true, 0.0, percentOfJointLimit * maxPitchForward, 0.0);
      submitDesiredPelvisOrientation(true, 0.0, percentOfJointLimitForPelvis * maxPitchForward, 0.0);

      submitDesiredChestOrientation(true, 0.0, percentOfJointLimit * maxPitchBackward, 0.0);
      submitDesiredPelvisOrientation(true, 0.0, percentOfJointLimitForPelvis * maxPitchBackward, 0.0);

      submitDesiredChestOrientation(true, 0.0, 0.0, percentOfJointLimit * minMaxRoll);
      submitDesiredPelvisOrientation(true, 0.0, 0.0, percentOfJointLimitForPelvis * minMaxRoll);

      submitDesiredChestOrientation(true, 0.0, 0.0, -percentOfJointLimit * minMaxRoll);
      submitDesiredPelvisOrientation(true, 0.0, 0.0, -percentOfJointLimitForPelvis * minMaxRoll);

      submitDesiredChestOrientation(true, 0.0, percentOfJointLimit * maxPitchForward, 0.0);
      submitDesiredPelvisOrientation(true, 0.0, percentOfJointLimitForPelvis * maxPitchForward, percentOfJointLimitForPelvis * minMaxRoll);

      submitDesiredChestOrientation(true, 0.0, percentOfJointLimit * maxPitchForward, 0.0);
      submitDesiredPelvisOrientation(true, 0.0, percentOfJointLimitForPelvis * maxPitchForward, -percentOfJointLimitForPelvis * minMaxRoll);

      submitDesiredChestOrientation(true, 0.0, percentOfJointLimit * maxPitchBackward, 0.0);
      submitDesiredPelvisOrientation(true, 0.0, percentOfJointLimitForPelvis * maxPitchBackward, -percentOfJointLimitForPelvis * minMaxRoll);

      submitDesiredChestOrientation(true, 0.0, percentOfJointLimit * maxPitchBackward, 0.0);
      submitDesiredPelvisOrientation(true, 0.0, percentOfJointLimitForPelvis * maxPitchBackward, percentOfJointLimitForPelvis * minMaxRoll);

      submitChestHomeCommand(true);
      submitPelvisHomeCommand(true);
   }

   private void sequenceArmPose(RobotSide robotSide)
   {
      if (robotSide == null)
      {
         submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.REACH_BACK);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.REACH_WAY_BACK);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.ARMS_03);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.REACH_FORWARD);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.SMALL_CHICKEN_WINGS);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.LARGE_CHICKEN_WINGS);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.STRAIGHTEN_ELBOWS);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.SUPPINATE_ARMS_IN_A_LITTLE);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.ARMS_BACK);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.LARGER_CHICKEN_WINGS);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.ARMS_OUT_EXTENDED);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.SUPPINATE_ARMS_IN_MORE);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.SUPPINATE_ARMS_IN_A_LOT);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.SUPER_CHICKEN_WINGS);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.FLYING);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.FLYING_SUPPINATE_IN);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.FLYING_SUPPINATE_OUT);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.ARM_NINETY_ELBOW_DOWN);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.ARM_NINETY_ELBOW_FORWARD);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.ARM_NINETY_ELBOW_UP);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.ARM_FORTFIVE_ELBOW_UP);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.ARM_FORTFIVE_ELBOW_DOWN);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.ARM_OUT_TRICEP_EXERCISE);
         submitSymmetricHumanoidArmPose(HumanoidArmPose.ARM_STRAIGHT_DOWN);
      }
      else
      {
         submitHumanoidArmPose(robotSide, HumanoidArmPose.STAND_PREP);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.REACH_BACK);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.REACH_WAY_BACK);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.ARMS_03);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.REACH_FORWARD);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.SMALL_CHICKEN_WINGS);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.LARGE_CHICKEN_WINGS);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.STRAIGHTEN_ELBOWS);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.SUPPINATE_ARMS_IN_A_LITTLE);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.ARMS_BACK);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.LARGER_CHICKEN_WINGS);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.ARMS_OUT_EXTENDED);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.SUPPINATE_ARMS_IN_MORE);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.SUPPINATE_ARMS_IN_A_LOT);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.SUPER_CHICKEN_WINGS);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.FLYING);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.FLYING_SUPPINATE_IN);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.FLYING_SUPPINATE_OUT);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.ARM_NINETY_ELBOW_DOWN);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.ARM_NINETY_ELBOW_FORWARD);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.ARM_NINETY_ELBOW_UP);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.ARM_FORTFIVE_ELBOW_UP);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.ARM_FORTFIVE_ELBOW_DOWN);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.ARM_OUT_TRICEP_EXERCISE);
         submitHumanoidArmPose(robotSide, HumanoidArmPose.ARM_STRAIGHT_DOWN);
      }
   }

   private void sequenceFootPoseShort()
   {
      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);

      RobotSide robotSide = activeSideForFootControl.getEnumValue();
      if (robotSide == null)
      {
         for (RobotSide side : RobotSide.values())
         {
            submitFootPosesShort(side);
         }
      }
      else
      {
         submitFootPosesShort(robotSide);
      }

      submitArmGoHomeCommand(false);
   }

   private void submitFootPosesShort(RobotSide robotSide)
   {

      double outsideFootDisplacement = maxFootPoseDisplacement.getDoubleValue();
      double insideFootDisplacement = 0.4 * maxFootPoseDisplacement.getDoubleValue();
      double footPoseHeight = maxFootPoseHeight.getDoubleValue();

      ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide);
      //foot remains flat
      boolean parallelize = false;
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, outsideFootDisplacement, 0.0, footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, -outsideFootDisplacement, 0.0, footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(outsideFootDisplacement), footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, footPoseHeight));

      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, outsideFootDisplacement, 0.0, footPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), footPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, outsideFootDisplacement, robotSide.negateIfRightSide(-insideFootDisplacement), footPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), footPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), footPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(-insideFootDisplacement), footPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), footPoseHeight));

      //footOrientation changes
      //      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, footPoseHeight));
      //      submitFootPose(parallelize, robotSide, ankleZUpFrame, 0.6, robotSide.negateIfRightSide(0.02), 0.15, 0.0, -0.9, 0.0);
      //      submitFootPose(parallelize, robotSide, ankleZUpFrame, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0);
      //      submitFootPose(parallelize, robotSide, ankleZUpFrame, -0.25, robotSide.negateIfRightSide(0.01), 0.15, 0.0, 1.2, 0.0);
      //      submitFootPose(parallelize, robotSide, ankleZUpFrame, -0.5, robotSide.negateIfRightSide(0.02), 0.30, 0.0, 2.4, 0.0);
      //      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, footPoseHeight));
      //      submitFootPose(parallelize, robotSide, ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.3), 0.20, 0.0, 0.0, robotSide.negateIfRightSide(0.5));
      //

      //put the foot back on the ground
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, -0.1));
   }

   private void sequenceFootPoseLong()
   {
      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);

      RobotSide robotSide = activeSideForFootControl.getEnumValue();
      if (robotSide == null)
      {
         for (RobotSide side : RobotSide.values())
         {
            submitFootPosesLong(side);
         }
      }
      else
      {
         submitFootPosesLong(robotSide);
      }

      submitArmGoHomeCommand(false);
   }

   private void submitFootPosesLong(RobotSide robotSide)
   {
      ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide);

      double outsideFootDisplacement = maxFootPoseDisplacement.getDoubleValue();
      double insideFootDisplacement = 0.2 * maxFootPoseDisplacement.getDoubleValue();

      double higherFootPoseHeight = maxFootPoseHeight.getDoubleValue();
      double midFootPoseHeight = 0.5 * maxFootPoseHeight.getDoubleValue();

      ///////////////////     good     ////////////////////////////
      boolean parallelize = false;
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, outsideFootDisplacement, 0.0, higherFootPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, outsideFootDisplacement, 0.0, midFootPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, outsideFootDisplacement, 0.0, higherFootPoseHeight));

      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, -outsideFootDisplacement, 0.0, higherFootPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, -outsideFootDisplacement, 0.0, midFootPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, -outsideFootDisplacement, 0.0, higherFootPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));

      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(outsideFootDisplacement), higherFootPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(outsideFootDisplacement), midFootPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(outsideFootDisplacement), higherFootPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));

      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), higherFootPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), midFootPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, outsideFootDisplacement, robotSide.negateIfRightSide(-insideFootDisplacement), higherFootPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), midFootPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(-insideFootDisplacement), higherFootPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), midFootPoseHeight));
      submitFootPosition(parallelize, robotSide,
                         new FramePoint3D(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(-insideFootDisplacement), midFootPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));
      ////////////////////////////////////////////////////////

      //footOrientation changes
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));
      submitFootPose(parallelize, robotSide, ankleZUpFrame, 0.6, robotSide.negateIfRightSide(0.02), 0.15, 0.0, -0.9, 0.0);
      submitFootPose(parallelize, robotSide, ankleZUpFrame, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0);
      submitFootPose(parallelize, robotSide, ankleZUpFrame, -0.25, robotSide.negateIfRightSide(0.01), 0.15, 0.0, 1.2, 0.0);
      submitFootPose(parallelize, robotSide, ankleZUpFrame, -0.5, robotSide.negateIfRightSide(0.02), 0.30, 0.0, 2.4, 0.0);
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));
      submitFootPose(parallelize, robotSide, ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.3), 0.20, 0.0, 0.0, robotSide.negateIfRightSide(0.5));

      //put the foot back on the ground
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, -0.1));

   }

   private void sequenceRunningMan()
   {
      for (RobotSide robotSide : RobotSide.values)
         runningMan(robotSide);
   }

   // FIXME Atlas Can't reach far back
   private void runningMan(RobotSide robotSide)
   {
      ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide.getOppositeSide());
      boolean mirrorOrientationsForRightSide = true;

      // First Lift up the foot
      submitFootPosition(false, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.25), 0.1));

      // Go to running man pose:
      FrameQuaternion desiredUpperArmOrientation = new FrameQuaternion(fullRobotModel.getChest().getBodyFixedFrame());
      desiredUpperArmOrientation.setYawPitchRoll(-1.2, -0.90 * Math.PI / 2.0, 0.0);
      submitHandPose(robotSide, desiredUpperArmOrientation, -Math.PI / 2.0, null, mirrorOrientationsForRightSide);

      if (canArmsReachFarBehind.getValue())
         desiredUpperArmOrientation.setYawPitchRoll(1.2, 0.90 * Math.PI / 2.0, 0.0); // Normal Running man
      else
         desiredUpperArmOrientation.setYawPitchRoll(0.7800, 1.4300, -0.0000); // Running man for Atlas  //FIXME check values for atlas
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -Math.PI / 2.0, null, mirrorOrientationsForRightSide);

      FramePose3D footPose = new FramePose3D(ankleZUpFrame);
      footPose.setPosition(-0.40, robotSide.negateIfRightSide(0.25), 0.40);
      footPose.setOrientationYawPitchRoll(0.0, 0.8 * Math.PI / 2.0, 0.0);
      submitFootPose(true, robotSide, footPose);

      submitDesiredChestOrientation(true, 0.0, Math.toRadians(20.0), 0.0);
      submitDesiredPelvisOrientation(true, 0.0, Math.toRadians(10.0), 0.0);

      pipeLine.submitSingleTaskStage(new NullState());

      // Do a "Y" stance with the foot outside
      desiredUpperArmOrientation.setYawPitchRoll(0.0, 0.0, 1.1);
      submitSymmetricHandPose(desiredUpperArmOrientation, 0.0, null); // Couldn't find Solution for upper arm

      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.65), 0.13);
      footPose.setOrientationYawPitchRoll(0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(40.0)));
      submitFootPose(true, robotSide, footPose);
      submitChestHomeCommand(true);
      submitDesiredPelvisOrientation(true, 0.0, 0.0, Math.toRadians(robotSide.negateIfRightSide(25.0)));

      pipeLine.submitSingleTaskStage(new NullState());

      // Go back to stand prep but don't put the foot on the ground yet
      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);

      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.25), 0.13);
      footPose.setOrientationYawPitchRoll(0.0, 0.0, 0.0);
      submitFootPose(true, robotSide, footPose);

      submitDesiredPelvisOrientation(true, 0.0, 0.0, 0.0);

      // Put the foot back on the ground
      submitFootPosition(false, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.25), -0.3));
   }

   private final double shoulderExtensionAngle = Math.toRadians(20.0);
   private final double shakeHandAngle = Math.toRadians(20.0);

   private void sequenceHandShakePrep()
   {
      RobotSide robotSide = RobotSide.RIGHT;
      boolean mirrorOrientationForRightSide = true;

      FrameQuaternion desiredUpperArmOrientation = new FrameQuaternion(fullRobotModel.getChest().getBodyFixedFrame());
      FrameQuaternion desiredHandOrientation = new FrameQuaternion(lowerArmsFrames.get(robotSide));

      desiredUpperArmOrientation.setYawPitchRoll(0.0, -shoulderExtensionAngle, -1.2708);
      desiredHandOrientation.setYawPitchRoll(0.0, Math.PI / 2.0, 0.0);
      submitHandPose(robotSide, desiredUpperArmOrientation, shoulderExtensionAngle - Math.PI / 2.0, desiredHandOrientation, mirrorOrientationForRightSide);
      pipeLine.requestNewStage();

   }

   private void sequenceHandShakeShake()
   {
      RobotSide robotSide = RobotSide.RIGHT;
      boolean mirrorOrientationForRightSide = true;

      FrameQuaternion desiredUpperArmOrientation = new FrameQuaternion(fullRobotModel.getChest().getBodyFixedFrame());
      FrameQuaternion desiredHandOrientation = new FrameQuaternion(lowerArmsFrames.get(robotSide));

      desiredUpperArmOrientation.setYawPitchRoll(0.0, -shoulderExtensionAngle, -1.2708);

      int numberOfShakes = 3;
      for (int i = 0; i < numberOfShakes; i++)
      {
         desiredHandOrientation.setYawPitchRoll(0.0, Math.PI / 2.0, shakeHandAngle);
         submitHandPose(robotSide, desiredUpperArmOrientation, shakeHandAngle + shoulderExtensionAngle - Math.PI / 2.0, desiredHandOrientation,
                        mirrorOrientationForRightSide);
         pipeLine.requestNewStage();

         desiredHandOrientation.setYawPitchRoll(0.0, Math.PI / 2.0, 0.0);
         submitHandPose(robotSide, desiredUpperArmOrientation, shoulderExtensionAngle - Math.PI / 2.0, desiredHandOrientation, mirrorOrientationForRightSide);
         pipeLine.requestNewStage();
      }
   }

   private void sequenceFlexUp()
   {
      flexUp();
      sequenceGoHome();
   }

   private void sequenceFlexUpFlexDown()
   {
      RobotSide robotSide = RobotSide.LEFT;
      ReferenceFrame soleZUpFrame = soleZUpFrames.get(robotSide);

      //put the left foot forward
      FramePose3D desiredFootstepPosition = new FramePose3D(soleZUpFrame);
      Point3D position = new Point3D(0.2, robotSide.negateIfRightSide(0.12), 0.0);
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      desiredFootstepPosition.set(position, orientation);
      desiredFootstepPosition.changeFrame(worldFrame);
      submitFootstepPose(true, robotSide, desiredFootstepPosition);
      pipeLine.requestNewStage();

      flexUp();
      flexDown();

      sequenceGoHome();

      sequenceSquareUp();
      submitChestHomeCommand(true);
      submitPelvisHomeCommand(true);
      pipeLine.requestNewStage();

   }

   private void flexUp()
   {
      submitSymmetricHumanoidArmPose(HumanoidArmPose.FLYING_PALMS_UP);
      pipeLine.requestNewStage();
      submitSymmetricHumanoidArmPose(HumanoidArmPose.FLEX_UP);
      pipeLine.requestNewStage();
      submitSymmetricHumanoidArmPose(HumanoidArmPose.FLYING_PALMS_UP);
      pipeLine.requestNewStage();
   }

   private void sequenceFlexDown()
   {
      RobotSide robotSide = RobotSide.LEFT;
      ReferenceFrame soleZUpFrame = soleZUpFrames.get(robotSide);

      //put the left foot forward
      FramePose3D desiredFootstepPosition = new FramePose3D(soleZUpFrame);
      Point3D position = new Point3D(0.2, robotSide.negateIfRightSide(0.12), 0.0);
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      desiredFootstepPosition.set(position, orientation);
      desiredFootstepPosition.changeFrame(worldFrame);
      submitFootstepPose(true, robotSide, desiredFootstepPosition);
      pipeLine.requestNewStage();

      flexDown();

      sequenceGoHome();

      sequenceSquareUp();
      submitChestHomeCommand(true);
      submitPelvisHomeCommand(true);
      pipeLine.requestNewStage();
   }

   private void flexDown()
   {
      submitSymmetricHumanoidArmPose(HumanoidArmPose.FLYING_SUPPINATE_IN);
      pipeLine.requestNewStage();
      submitDesiredChestOrientation(true, 0.0, 0.7 * maxPitchForward, 0.0);
      submitSymmetricHumanoidArmPose(HumanoidArmPose.FLEX_DOWN);
      pipeLine.requestNewStage();
   }

   private void sequenceCuteWave()
   {
      if (activeSideForHandControl.getEnumValue() == null)
      {
         for (RobotSide robotSide : RobotSide.values())
         {
            cuteWave(robotSide);
         }
      }
      else
      {
         cuteWave(activeSideForHandControl.getEnumValue());
      }
   }

   private void cuteWave(RobotSide robotSide)
   {

      FrameQuaternion desiredUpperArmOrientation = new FrameQuaternion(fullRobotModel.getChest().getBodyFixedFrame());
      FrameQuaternion desiredHandOnHipOrientation = new FrameQuaternion(lowerArmsFrames.get(robotSide.getOppositeSide()));
      FrameQuaternion desiredHandWavingOrientation = new FrameQuaternion(lowerArmsFrames.get(robotSide));
      boolean mirrorOrientationForRightSide = true;

      submitDesiredPelvisOrientation(true, 0.0, 0.0, Math.toRadians((robotSide == RobotSide.RIGHT ? 20.0 : -20.0)));

      desiredUpperArmOrientation.setYawPitchRoll(0.0, Math.toRadians(45.0), Math.toRadians(-75.0));
      desiredHandOnHipOrientation.setYawPitchRoll(0.0, Math.PI / 2.0, 0.0);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -Math.PI / 2.0, desiredHandOnHipOrientation, mirrorOrientationForRightSide);

      desiredUpperArmOrientation.setYawPitchRoll(0.0, -Math.PI / 2.0, 0.0);
      desiredHandWavingOrientation.setYawPitchRoll(0.0, Math.PI / 2.0, 0.0);
      submitHandPose(robotSide, desiredUpperArmOrientation, -Math.PI / 2.0, desiredHandWavingOrientation, mirrorOrientationForRightSide);
      pipeLine.requestNewStage();

      int numberOfWaves = 3;

      for (int i = 0; i < numberOfWaves; i++)
      {
         desiredHandWavingOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.PI / 2.0, Math.toRadians(-30.0));
         submitHandPose(robotSide, desiredUpperArmOrientation, -Math.PI / 2.0, desiredHandWavingOrientation, mirrorOrientationForRightSide);
         pipeLine.requestNewStage();

         desiredHandWavingOrientation.setYawPitchRoll(Math.toRadians(0.0), Math.PI / 2.0, Math.toRadians(25.0));
         submitHandPose(robotSide, desiredUpperArmOrientation, -Math.PI / 2.0, desiredHandWavingOrientation, mirrorOrientationForRightSide);
         pipeLine.requestNewStage();
      }
      sequenceGoHome();
   }

   private void sequenceBow()
   {
      bow(RobotSide.LEFT);
   }

   private void bow(RobotSide robotSide)
   {
      ReferenceFrame soleZUpFrame = soleZUpFrames.get(robotSide.getOppositeSide());
      FrameQuaternion desiredUpperArmOrientation = new FrameQuaternion(fullRobotModel.getChest().getBodyFixedFrame());
      boolean mirrorOrientationForRightSide = true;

      //put the foot forward and prepare the arms
      FramePose3D desiredFootstepPosition = new FramePose3D(soleZUpFrame);
      Point3D position = new Point3D(0.2, robotSide.negateIfRightSide(0.25), 0.0);
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      desiredFootstepPosition.set(position, orientation);
      desiredFootstepPosition.changeFrame(worldFrame);
      submitFootstepPose(true, robotSide, desiredFootstepPosition);

      if (canArmsReachFarBehind.getValue())
         desiredUpperArmOrientation.setYawPitchRoll(0.2, -0.05, -1.3708);
      else
         desiredUpperArmOrientation.setYawPitchRoll(0.0000, 0.5230, -1.2708); //FIXME check values for atlas
      submitHandPose(robotSide, desiredUpperArmOrientation, 0.0, null, mirrorOrientationForRightSide);
      desiredUpperArmOrientation.setYawPitchRoll(0.0000, -0.5235, -1.2708);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -0.1, null, mirrorOrientationForRightSide);
      pipeLine.requestNewStage();

      if (canArmsReachFarBehind.getValue())
      {
         desiredUpperArmOrientation.setYawPitchRoll(-1.7242, 0.2588, -2.1144);
         submitHandPose(robotSide, desiredUpperArmOrientation, -0.1, null, mirrorOrientationForRightSide);
      }
      desiredUpperArmOrientation.setYawPitchRoll(-1.4173, 0.2588, -1.0272);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -0.1, null, mirrorOrientationForRightSide);
      pipeLine.requestNewStage();

      //bend forward and arms
      if (canArmsReachFarBehind.getValue())
      {
         desiredUpperArmOrientation.setYawPitchRoll(-1.7242, 0.2588, -2.1144);
         submitHandPose(robotSide, desiredUpperArmOrientation, -Math.PI / 2.0, null, mirrorOrientationForRightSide);
      }
      desiredUpperArmOrientation.setYawPitchRoll(-1.4173, 0.2588, -1.0272);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -Math.PI / 2.0, null, mirrorOrientationForRightSide);
      submitDesiredChestOrientation(true, 0.0, 0.7 * maxPitchForward, 0.0);
      submitDesiredPelvisOrientation(true, 0.0, 0.5 * maxPitchForward, 0.0);
      pipeLine.requestNewStage();

      if (canArmsReachFarBehind.getValue())
      {
         desiredUpperArmOrientation.setYawPitchRoll(-1.7242, 0.2588, -2.1144);
         submitHandPose(robotSide, desiredUpperArmOrientation, -0.1, null, mirrorOrientationForRightSide);
      }
      pipeLine.requestNewStage();

      //back to normal stance
      submitChestHomeCommand(true);
      submitPelvisHomeCommand(true);
      pipeLine.requestNewStage();

      desiredUpperArmOrientation.setYawPitchRoll(-1.4173, 0.2588, -1.0272);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -0.1, null, mirrorOrientationForRightSide);
      pipeLine.requestNewStage();

      desiredUpperArmOrientation.setYawPitchRoll(0.0000, -0.5230, -1.2708);
      submitHandPose(robotSide, desiredUpperArmOrientation, -0.1, null, mirrorOrientationForRightSide);
      desiredUpperArmOrientation.setYawPitchRoll(0.0000, -0.5235, -1.2708);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -0.1, null, mirrorOrientationForRightSide);
      pipeLine.requestNewStage();

      submitArmGoHomeCommand(true);
      pipeLine.requestNewStage();

      //square up the feet
      desiredFootstepPosition = new FramePose3D(soleZUpFrame);
      position = new Point3D(0., robotSide.negateIfRightSide(0.25), 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      desiredFootstepPosition.set(position, orientation);
      desiredFootstepPosition.changeFrame(worldFrame);
      submitFootstepPose(false, robotSide, desiredFootstepPosition);
      pipeLine.requestNewStage();

      submitChestHomeCommand(true);
      submitPelvisHomeCommand(true);
      pipeLine.requestNewStage();
   }

   private void sequenceStepsLong()
   {
      pipeLine.requestNewStage();

      double distanceToWalk = 0.5;

      /////////// normal stance ///////////
      // forward
      submitWalkToLocation(false, distanceToWalk, 0.0, 0.0, 0.0);
      //backward
      submitWalkToLocation(false, 0.0, 0.0, 0.0, Math.PI); // Math.PI
      //sideWalk
      submitWalkToLocation(false, distanceToWalk, 0.0, -Math.PI / 2.0, -Math.PI / 2.0);
      submitWalkToLocation(false, 0.0, 0.0, 0.0, Math.PI / 2.0);
      //turn in place
      submitWalkToLocation(false, 0.0, 0.0, 0.0, 0.0);

      /////////// arms out ///////////
      submitSymmetricHumanoidArmPose(HumanoidArmPose.LARGE_CHICKEN_WINGS);

      // forward
      submitWalkToLocation(false, distanceToWalk, 0.0, 0.0, 0.0);
      //backward
      submitWalkToLocation(false, 0.0, 0.0, 0.0, Math.PI); // Math.PI
      //sideWalk
      submitWalkToLocation(false, distanceToWalk, 0.0, -Math.PI / 2.0, -Math.PI / 2.0);
      submitWalkToLocation(false, 0.0, 0.0, 0.0, Math.PI / 2.0);
      //turn in place
      submitWalkToLocation(false, 0.0, 0.0, 0.0, 0.0);

      submitArmGoHomeCommand(false);

      /////////// chest bending backward///////////
      submitDesiredChestOrientation(false, 0.0, Math.toRadians(-10.0), 0.0);

      // forward
      submitWalkToLocation(false, distanceToWalk, 0.0, 0.0, 0.0);
      //backward
      submitWalkToLocation(false, 0.0, 0.0, 0.0, Math.PI); // Math.PI
      //sideWalk
      submitWalkToLocation(false, distanceToWalk, 0.0, -Math.PI / 2.0, -Math.PI / 2.0);
      submitWalkToLocation(false, 0.0, 0.0, 0.0, Math.PI / 2.0);
      //turn in place
      submitWalkToLocation(false, 0.0, 0.0, 0.0, 0.0);

      submitChestHomeCommand(false);
   }

   private void sequenceStepsShort()
   {
      pipeLine.requestNewStage();

      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
      {
         // forward
         submitWalkToLocation(false, 0.5, 0.0, 0.0, 0.0);
         //backward
         submitWalkToLocation(false, 0.0, 0.0, 0.0, Math.PI); // Math.PI
         //sideWalk
         submitWalkToLocation(false, 0.5, 0.0, -Math.PI / 2.0, -Math.PI / 2.0);
         submitWalkToLocation(false, 0.0, 0.0, 0.0, Math.PI / 2.0);
         //turn in place
         submitWalkToLocation(false, 0.0, 0.0, 0.0, 0.0);
      }
   }

   private void sequenceWalkForwardBackward(double percentOfMaxFootstepLength)
   {
      // forward
      submitWalkToLocation(false, 1.0, 0.0, 0.0, 0.0, percentOfMaxFootstepLength);
      //backward
      submitWalkToLocation(false, 0.0, 0.0, 0.0, Math.PI, percentOfMaxFootstepLength);
   }

   private void submitWalkToLocation(boolean parallelize, double x, double y, double robotYaw, double angleRelativeToPath, double percentOfMaxFootstepLength)
   {
      FramePose2D targetPoseInWorld = new FramePose2D();
      targetPoseInWorld.setIncludingFrame(midFeetZUpFrame, x, y, robotYaw);
      targetPoseInWorld.changeFrame(worldFrame);

      WalkToLocationTask walkToLocationTask = new WalkToLocationTask(targetPoseInWorld, walkToLocationBehavior, angleRelativeToPath,
                                                                     footstepLength.getDoubleValue() * percentOfMaxFootstepLength, swingTime.getDoubleValue(),
                                                                     transferTime.getDoubleValue());
      if (parallelize)
      {
         pipeLine.submitTaskForPallelPipesStage(walkToLocationBehavior, walkToLocationTask);
      }
      else
      {
         pipeLine.submitSingleTaskStage(walkToLocationTask);
      }
   }

   private void submitWalkToLocation(boolean parallelize, double x, double y, double robotYaw, double angleRelativeToPath)
   {
      submitWalkToLocation(parallelize, x, y, robotYaw, angleRelativeToPath, 1.0);
   }

   private void sequenceStepsInPlace()
   {
      FootstepDataListMessage footstepDataList = HumanoidMessageTools.createFootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      FramePose3D footstepPose = new FramePose3D();

      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
      {
         for (RobotSide robotSide : RobotSide.values())
         {
            footstepPose.setToZero(fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG));
            footstepPose.changeFrame(worldFrame);

            Point3D footLocation = new Point3D(footstepPose.getPosition());
            Quaternion footOrientation = new Quaternion(footstepPose.getOrientation());

            FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, footLocation, footOrientation);

            footstepDataList.getFootstepDataList().add().set(footstepData);
         }
      }
      pipeLine.submitSingleTaskStage(new FootstepListTask(footstepListBehavior, footstepDataList));
   }

   private void sequenceSquareUp()
   {
      FootstepDataListMessage footstepDataList = HumanoidMessageTools.createFootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      FramePose3D footstepPose = new FramePose3D();

      RobotSide robotSide = activeSideForFootControl.getEnumValue();

      if (robotSide == null)
         System.out.println("choose a foot to be squared up");
      else
      {
         footstepPose.setToZero(fullRobotModel.getSoleFrame(robotSide.getOppositeSide()));
         footstepPose.setY(robotSide.getOppositeSide().negateIfLeftSide(walkingControllerParameters.getSteppingParameters().getInPlaceWidth()));
         footstepPose.changeFrame(worldFrame);

         Point3D footLocation = new Point3D(footstepPose.getPosition());
         Quaternion footOrientation = new Quaternion(footstepPose.getOrientation());

         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, footLocation, footOrientation);

         footstepDataList.getFootstepDataList().add().set(footstepData);
         pipeLine.submitSingleTaskStage(new FootstepListTask(footstepListBehavior, footstepDataList));
      }

   }

   private void sequenceKraneKick()
   {
      kraneKick(RobotSide.RIGHT);
   }

   private void kraneKick(RobotSide robotSide)
   {
      ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide.getOppositeSide());
      // First Lift up the foot
      submitFootPosition(false, robotSide, new FramePoint3D(ankleZUpFrame, 0.1, robotSide.negateIfRightSide(0.25), 0.2));
      submitSymmetricHumanoidArmPose(HumanoidArmPose.KARATE_KID_KRANE_KICK);
      pipeLine.requestNewStage();

      // Go back to stand prep but don't put the foot on the ground yet
      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);

      FramePose3D footPose = new FramePose3D();
      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.25), 0.1);
      footPose.setOrientationYawPitchRoll(0.0, 0.0, 0.0);
      submitFootPose(true, robotSide, footPose);

      submitChestHomeCommand(true);

      // Put the foot back on the ground
      submitFootPosition(false, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.25), -0.3));
   }

   private void karateKid(RobotSide robotSide)
   {
      ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide.getOppositeSide());
      // First Lift up the foot
      submitFootPosition(false, robotSide, new FramePoint3D(ankleZUpFrame, 0.1, robotSide.negateIfRightSide(0.25), 0.2));

      // Put the arm down
      double halfPi = Math.PI / 2.0;
      ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();

      FrameQuaternion upperArmDown2 = new FrameQuaternion(chestFrame, -0.7443, 0.2789, -1.2803);
      FrameQuaternion upperArmIntermediateOnWayUp = new FrameQuaternion(chestFrame, -0.0000, 0.7853, -0.0000);
      FrameQuaternion upperArmUp1 = new FrameQuaternion(chestFrame, 0.6154, 0.5235, 0.9553);
      FrameQuaternion upperArmUp2 = new FrameQuaternion(chestFrame, -0.6154, -0.5235, 0.9553);
      FrameQuaternion upperArmIntermediateOnWayDown = new FrameQuaternion(chestFrame, 0.0000, -0.7853, -0.0000);
      FrameQuaternion upperArmDown1 = new FrameQuaternion(chestFrame, 0.7443, -0.2789, -1.2803);

      SideDependentList<double[]> armsDown2 = computeSymmetricArmJointAngles(upperArmDown2, 0.0, null, true);
      SideDependentList<double[]> armsIntermediateOnWayUp = computeSymmetricArmJointAngles(upperArmIntermediateOnWayUp, -halfPi / 2.0, null, true);
      SideDependentList<double[]> armsUp1 = computeSymmetricArmJointAngles(upperArmUp1, 0.0, null, true);
      SideDependentList<double[]> armsUp2 = computeSymmetricArmJointAngles(upperArmUp2, 0.0, null, true);
      SideDependentList<double[]> armsIntermediateOnWayDown = computeSymmetricArmJointAngles(upperArmIntermediateOnWayDown, -halfPi / 2.0, null, true);
      SideDependentList<double[]> armsDown1 = computeSymmetricArmJointAngles(upperArmDown1, 0.0, null, true);

      int numberOfHandPoses = 10;

      OneDoFTrajectoryPointCalculator calculator = new OneDoFTrajectoryPointCalculator();

      for (RobotSide flyingSide : RobotSide.values)
      {
         ArmTrajectoryMessage flyingMessage = HumanoidMessageTools.createArmTrajectoryMessage(flyingSide);

         for (int jointIndex = 0; jointIndex < numberOfArmJoints; jointIndex++)
         {
            calculator.clear();

            for (int poseIndex = 0; poseIndex < numberOfHandPoses; poseIndex++)
            {
               double desiredJointAngle;
               switch (poseIndex % 6)
               {
               case 0:
                  desiredJointAngle = armsDown1.get(flyingSide)[jointIndex];
                  break;
               case 1:
                  desiredJointAngle = armsDown2.get(flyingSide)[jointIndex];
                  break;
               case 2:
                  desiredJointAngle = armsIntermediateOnWayUp.get(flyingSide)[jointIndex];
                  break;
               case 3:
                  desiredJointAngle = armsUp1.get(flyingSide)[jointIndex];
                  break;
               case 4:
                  desiredJointAngle = armsUp2.get(flyingSide)[jointIndex];
                  break;
               case 5:
                  desiredJointAngle = armsIntermediateOnWayDown.get(flyingSide)[jointIndex];
                  break;
               default:
                  throw new RuntimeException("Should not get there!");
               }
               calculator.appendTrajectoryPoint(desiredJointAngle);
            }
            calculator.compute(flyingTrajectoryTime.getDoubleValue());
            OneDoFTrajectoryPointList trajectoryData = calculator.getTrajectoryData();
            trajectoryData.addTimeOffset(trajectoryTime.getDoubleValue()); // Add time to reach the first waypoint.

            for (int i = 1; i < trajectoryData.getNumberOfTrajectoryPoints(); i++)
            {
               OneDoFTrajectoryPoint previousTrajectoryPoint = trajectoryData.getTrajectoryPoint(i - 1);
               OneDoFTrajectoryPoint trajectoryPoint = trajectoryData.getTrajectoryPoint(i);
               if (previousTrajectoryPoint.getTime() >= trajectoryPoint.getTime())
                  trajectoryPoint.setTime(previousTrajectoryPoint.getTime() + 1.0e-5); // Hack to get the controller to accept some of the waypoints, needs to be properly fixed at some point.
            }
            OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = flyingMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add();
            oneDoFJointTrajectoryMessage.set(HumanoidMessageTools.createOneDoFJointTrajectoryMessage(trajectoryData));
         }

         pipeLine.submitTaskForPallelPipesStage(armTrajectoryBehaviors.get(flyingSide),
                                                new ArmTrajectoryTask(flyingMessage, armTrajectoryBehaviors.get(flyingSide)));

         pipeLine.submitTaskForPallelPipesStage(armTrajectoryBehaviors.get(flyingSide), createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));
      }

      // Put the arms in front
      FrameQuaternion desiredUpperArmOrientation = new FrameQuaternion(chestFrame);
      desiredUpperArmOrientation.setYawPitchRoll(0.0000, -0.0000, -0.9708);
      submitSymmetricHandPose(desiredUpperArmOrientation, -halfPi, null);
      pipeLine.requestNewStage();

      desiredUpperArmOrientation.setYawPitchRoll(-0.7800, 0.1585, -0.8235);
      submitSymmetricHandPose(desiredUpperArmOrientation, -1.40, null);

      // Supa powerful front kick!!!!!
      FramePose3D footPose = new FramePose3D();
      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.75, robotSide.negateIfRightSide(0.25), 0.25);
      footPose.setOrientationYawPitchRoll(0.0, -halfPi / 2.0, 0.0);
      submitFootPose(true, robotSide, footPose);

      submitDesiredChestOrientation(true, 0.0, Math.toRadians(-5.0), 0.0);
      submitDesiredPelvisOrientation(true, 0.0, Math.toRadians(-15.0), 0.0);

      pipeLine.requestNewStage();

      // Supa powerful back kick!!!!!
      desiredUpperArmOrientation.setYawPitchRoll(-0.7566, -0.9980, -0.5761);
      submitHandPose(robotSide, desiredUpperArmOrientation, -0.3, null, true);

      desiredUpperArmOrientation.setYawPitchRoll(0.0000, 1.2566, -1.2708);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -0.3, null, true);

      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(-0.75, robotSide.negateIfRightSide(0.25), 0.35);
      footPose.setOrientationYawPitchRoll(0.0, 0.8 * halfPi, 0.0);
      submitFootPose(true, robotSide, footPose);

      submitDesiredChestOrientation(true, 0.0, Math.toRadians(30.0), 0.0);
      submitDesiredPelvisOrientation(true, 0.0, Math.toRadians(20.0), 0.0);

      pipeLine.requestNewStage();

      // Supa powerful front kick!!!!!
      desiredUpperArmOrientation.setYawPitchRoll(-0.7800, 0.1585, -0.8235);
      submitSymmetricHandPose(desiredUpperArmOrientation, -1.40, null);
      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.75, robotSide.negateIfRightSide(0.25), 0.25);
      footPose.setOrientationYawPitchRoll(0.0, -halfPi / 2.0, 0.0);
      submitFootPose(true, robotSide, footPose);

      submitDesiredChestOrientation(true, 0.0, Math.toRadians(-5.0), 0.0);
      submitDesiredPelvisOrientation(true, 0.0, Math.toRadians(-15.0), 0.0);

      pipeLine.requestNewStage();

      // Supa powerful side kick!!!!!
      desiredUpperArmOrientation.setYawPitchRoll(0.0000, 0.4000, -0.0000);
      submitHandPose(robotSide, desiredUpperArmOrientation, -0.1, null, true);

      desiredUpperArmOrientation.setYawPitchRoll(-1.2707, 0.0, 0.0);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -halfPi, null, true);

      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.65), 0.2);
      footPose.setOrientationYawPitchRoll(0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(40.0)));
      submitFootPose(true, robotSide, footPose);

      submitDesiredChestOrientation(true, 0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(30.0)));
      submitDesiredPelvisOrientation(true, 0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(20.0)));

      pipeLine.requestNewStage();

      // Go back to stand prep but don't put the foot on the ground yet
      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);

      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.25), 0.1);
      footPose.setOrientationYawPitchRoll(0.0, 0.0, 0.0);
      submitFootPose(true, robotSide, footPose);

      submitChestHomeCommand(true);
      submitPelvisHomeCommand(true);

      // Put the foot back on the ground
      submitFootPosition(false, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.25), -0.3));

      //
      submitArmGoHomeCommand(true);
      submitChestHomeCommand(true);
      submitPelvisHomeCommand(true);
   }

   private void sequenceSquats()
   {
      submitDesiredPelvisHeight(false, minCoMHeightOffset.getDoubleValue());
      submitDesiredPelvisHeight(false, maxCoMHeightOffset.getDoubleValue());
   }

   private void sequenceSquatathon()
   {
      boolean parallelize = true;
      submitDesiredPelvisOrientation(parallelize, 0.0, Math.toRadians(15), 0.0);
      submitDesiredChestOrientation(parallelize, 0.0, Math.toRadians(15.0), 0.0);

      submitSymmetricHumanoidArmPose(HumanoidArmPose.REACH_WAY_FORWARD);
      submitDesiredPelvisHeight(parallelize, minCoMHeightOffset.getDoubleValue());

      pipeLine.requestNewStage();

      submitChestHomeCommand(parallelize);
      submitPelvisHomeCommand(parallelize);

      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);

      submitDesiredPelvisHeight(parallelize, maxCoMHeightOffset.getDoubleValue());
   }

   private ReferenceFrame findFixedFrameForPelvisOrientation()
   {
      if (supportLeg.getEnumValue() == null)
         return midFeetZUpFrame;
      else
         return ankleZUpFrames.get(supportLeg.getEnumValue());
   }

   private void submitChestHomeCommand(boolean parallelize)
   {
      GoHomeMessage goHomeMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.CHEST, trajectoryTime.getDoubleValue());
      GoHomeTask goHomeTask = new GoHomeTask(goHomeMessage, chestGoHomeBehavior);
      if (parallelize)
      {
         pipeLine.submitTaskForPallelPipesStage(chestGoHomeBehavior, goHomeTask);
         pipeLine.submitTaskForPallelPipesStage(chestGoHomeBehavior, createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));

      }
      else
      {
         pipeLine.submitSingleTaskStage(goHomeTask);
         pipeLine.submitSingleTaskStage(createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));

      }
   }

   private void submitDesiredChestOrientation(boolean parallelize, double yaw, double pitch, double roll)
   {
      FrameQuaternion desiredChestOrientation = new FrameQuaternion(pelvisZUpFrame, yaw, pitch, roll);
      ChestOrientationTask chestOrientationTask = new ChestOrientationTask(desiredChestOrientation, chestTrajectoryBehavior, trajectoryTime.getDoubleValue(),
                                                                           pelvisZUpFrame);
      if (parallelize)
      {
         pipeLine.submitTaskForPallelPipesStage(chestTrajectoryBehavior, chestOrientationTask);
         pipeLine.submitTaskForPallelPipesStage(chestTrajectoryBehavior, createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));

      }
      else
      {
         pipeLine.submitSingleTaskStage(chestOrientationTask);
         pipeLine.submitSingleTaskStage(createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));

      }
   }

   private void submitDesiredPelvisHeight(boolean parallelize, double offsetHeight)
   {
      ReferenceFrame frameAfterRootJoint = fullRobotModel.getRootJoint().getFrameAfterJoint();
      FramePoint3D desiredPelvisPosition = new FramePoint3D(frameAfterRootJoint);
      desiredPelvisPosition.setZ(offsetHeight);
      desiredPelvisPosition.changeFrame(worldFrame);
      PelvisHeightTrajectoryTask comHeightTask = new PelvisHeightTrajectoryTask(desiredPelvisPosition.getZ(), pelvisHeightTrajectoryBehavior,
                                                                                trajectoryTime.getDoubleValue());
      if (parallelize)
      {
         pipeLine.submitTaskForPallelPipesStage(pelvisHeightTrajectoryBehavior, comHeightTask);
         pipeLine.submitTaskForPallelPipesStage(pelvisHeightTrajectoryBehavior, createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));

      }
      else
      {
         pipeLine.submitSingleTaskStage(comHeightTask);
         pipeLine.submitSingleTaskStage(createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));

      }
   }

   private void submitPelvisHomeCommand(boolean parallelize)
   {
      GoHomeMessage goHomeMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.PELVIS, trajectoryTime.getDoubleValue());
      GoHomeTask goHomeTask = new GoHomeTask(goHomeMessage, pelvisGoHomeBehavior);

      if (parallelize)
      {
         pipeLine.submitTaskForPallelPipesStage(pelvisGoHomeBehavior, goHomeTask);
         pipeLine.submitTaskForPallelPipesStage(pelvisGoHomeBehavior, createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));

      }
      else
      {
         pipeLine.submitSingleTaskStage(goHomeTask);
         pipeLine.submitSingleTaskStage(createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));

      }
   }

   private void submitDesiredPelvisOrientation(boolean parallelize, double yaw, double pitch, double roll)
   {
      submitDesiredPelvisOrientation(parallelize, yaw, pitch, roll, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue());
   }

   private void submitDesiredPelvisOrientation(boolean parallelize, double yaw, double pitch, double roll, double trajectoryTime, double sleepTime)
   {
      FrameQuaternion desiredPelvisOrientation = new FrameQuaternion(findFixedFrameForPelvisOrientation(), yaw, pitch, roll);
      desiredPelvisOrientation.changeFrame(worldFrame);
      Quaternion desiredQuaternion = new Quaternion(desiredPelvisOrientation);
      PelvisOrientationTrajectoryMessage message = HumanoidMessageTools.createPelvisOrientationTrajectoryMessage(trajectoryTime, desiredQuaternion);
      PelvisOrientationTrajectoryTask task = new PelvisOrientationTrajectoryTask(message, pelvisOrientationTrajectoryBehavior);

      if (parallelize)
      {
         pipeLine.submitTaskForPallelPipesStage(pelvisOrientationTrajectoryBehavior, task);
         pipeLine.submitTaskForPallelPipesStage(pelvisOrientationTrajectoryBehavior, createSleepTask(sleepTime));

      }
      else
      {
         pipeLine.submitSingleTaskStage(task);
         pipeLine.submitSingleTaskStage(createSleepTask(sleepTime));

      }
   }

   private void submitDesiredPelvisPositionOffset(boolean parallelize, double dx, double dy, double dz)
   {
      submitDesiredPelvisPositionOffsetAndOrientation(parallelize, dx, dy, dz, 0.0, 0.0, 0.0);
   }

   private void submitDesiredPelvisPositionOffsetAndOrientation(boolean parallelize, double dx, double dy, double dz, double yaw, double pitch, double roll)
   {
      ReferenceFrame frameAfterRootJoint = fullRobotModel.getRootJoint().getFrameAfterJoint();
      FramePose3D desiredPelvisPose = new FramePose3D(frameAfterRootJoint);
      desiredPelvisPose.setPosition(dx, dy, dz);
      desiredPelvisPose.setOrientationYawPitchRoll(yaw, pitch, roll);
      desiredPelvisPose.changeFrame(worldFrame);
      Point3D position = new Point3D();
      Quaternion orientation = new Quaternion();
      desiredPelvisPose.get(position, orientation);
      PelvisTrajectoryMessage message = HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime.getDoubleValue(), position, orientation);
      PelvisTrajectoryTask task = new PelvisTrajectoryTask(message, pelvisTrajectoryBehavior);
      if (parallelize)
      {
         pipeLine.submitTaskForPallelPipesStage(pelvisTrajectoryBehavior, task);
         pipeLine.submitTaskForPallelPipesStage(pelvisTrajectoryBehavior, createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));

      }
      else
      {
         pipeLine.submitSingleTaskStage(task);
         pipeLine.submitSingleTaskStage(createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));

      }
   }

   private void submitArmGoHomeCommand(boolean parallelize)
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         GoHomeMessage goHomeMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.ARM, robotSide, trajectoryTime.getDoubleValue());
         GoHomeBehavior armGoHomeBehavior = armGoHomeBehaviors.get(robotSide);
         if (parallelize)
            pipeLine.submitTaskForPallelPipesStage(armGoHomeBehavior, new GoHomeTask(goHomeMessage, armGoHomeBehavior));
         else
            pipeLine.submitSingleTaskStage(new GoHomeTask(goHomeMessage, armGoHomeBehavior));
      }
   }

   public void submitSymmetricHumanoidArmPose(HumanoidArmPose armPose)
   {
      for (RobotSide robotSide : RobotSide.values)
         submitHumanoidArmPose(robotSide, armPose);
   }

   public void submitHumanoidArmPose(RobotSide robotSide, HumanoidArmPose armPose)
   {
      FrameQuaternion desiredUpperArmOrientation = new FrameQuaternion();
      desiredUpperArmOrientation.setYawPitchRollIncludingFrame(fullRobotModel.getChest().getBodyFixedFrame(), armPose.getDesiredUpperArmYawPitchRoll());
      double elbowAngle = robotSide.negateIfRightSide(armPose.getDesiredElbowAngle(robotSide));
      double[] handOrientation = new double[3];
      if (enableHandOrientation.getBooleanValue())
      {
         handOrientation = armPose.getDesiredHandYawPitchRoll();
      }
      FrameQuaternion desiredHandOrientation = new FrameQuaternion();
      desiredHandOrientation.setYawPitchRollIncludingFrame(lowerArmsFrames.get(robotSide), handOrientation);
      submitHandPose(robotSide, desiredUpperArmOrientation, elbowAngle, desiredHandOrientation, true);
   }

   public void submitSymmetricHandPose(FrameQuaternion desiredUpperArmOrientation, double elbowAngle, FrameQuaternion desiredHandOrientation)
   {
      for (RobotSide robotSide : RobotSide.values)
         submitHandPose(robotSide, desiredUpperArmOrientation, elbowAngle, desiredHandOrientation, true);
   }

   public void submitHandPose(RobotSide robotSide, FrameQuaternion desiredUpperArmOrientation, double elbowAngle, FrameQuaternion desiredHandOrientation,
                              boolean mirrorOrientationForRightSide)
   {
      double[] desiredJointAngles = computeArmJointAngles(robotSide, desiredUpperArmOrientation, elbowAngle, desiredHandOrientation,
                                                          mirrorOrientationForRightSide);

      if (desiredJointAngles != null)
      {
         if (DEBUG)
         {
            String msg = "QDesireds: ";
            for (int i = 0; i < desiredJointAngles.length; i++)
               msg += desiredJointAngles[i] + ", ";
            System.out.println(msg);
         }
         ArmTrajectoryBehavior armTrajectoryBehavior = armTrajectoryBehaviors.get(robotSide);
         ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime.getDoubleValue(), desiredJointAngles);
         pipeLine.submitTaskForPallelPipesStage(armTrajectoryBehavior, new ArmTrajectoryTask(message, armTrajectoryBehavior));
         pipeLine.submitTaskForPallelPipesStage(armTrajectoryBehavior, createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));

      }
   }

   public SideDependentList<double[]> computeSymmetricArmJointAngles(FrameQuaternion desiredUpperArmOrientation, double elbowAngle,
                                                                     FrameQuaternion desiredHandOrientation, boolean mirrorOrientationForRightSide)
   {
      SideDependentList<double[]> desiredSymmetricJointAngles = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         desiredSymmetricJointAngles.put(robotSide, computeArmJointAngles(robotSide, desiredUpperArmOrientation, elbowAngle, desiredHandOrientation,
                                                                          mirrorOrientationForRightSide));
      }
      return desiredSymmetricJointAngles;
   }

   private double[] computeArmJointAngles(RobotSide robotSide, FrameQuaternion desiredUpperArmOrientation, double elbowAngle,
                                          FrameQuaternion desiredHandOrientation, boolean mirrorOrientationForRightSide)
   {
      double[] desiredUpperArmJointAngles = computeUpperArmJointAngles(robotSide, desiredUpperArmOrientation, mirrorOrientationForRightSide, 0);
      if (desiredUpperArmJointAngles == null)
         return null;

      double[] desiredLowerArmJointAngles = computeLowerArmJointAngles(robotSide, desiredHandOrientation, mirrorOrientationForRightSide);
      if (desiredLowerArmJointAngles == null)
         return null;

      int numberOfOneDoFjoints = desiredUpperArmJointAngles.length + desiredLowerArmJointAngles.length + 1;
      int jointIndex = 0;
      double[] desiredJointAngles = new double[numberOfOneDoFjoints];

      for (int i = 0; i < desiredUpperArmJointAngles.length; i++)
      {
         desiredJointAngles[jointIndex] = desiredUpperArmJointAngles[i];
         jointIndex++;
      }

      desiredJointAngles[jointIndex] = elbowAngle * elbowJointSign.get(robotSide);
      jointIndex++;

      for (int i = 0; i < desiredLowerArmJointAngles.length; i++)
      {
         desiredJointAngles[jointIndex] = desiredLowerArmJointAngles[i];
         jointIndex++;
      }

      return desiredJointAngles;
   }

   private final Random random = new Random(541654L);

   private double[] computeUpperArmJointAngles(RobotSide robotSide, FrameQuaternion desiredUpperArmOrientation, boolean mirrorOrientationForRightSide,
                                               int iteration)
   {
      if (desiredUpperArmOrientation == null)
         return new double[upperArmJointsClone.get(robotSide).length];

      FrameQuaternion temporaryDesiredUpperArmOrientation = new FrameQuaternion();
      temporaryDesiredUpperArmOrientation.setIncludingFrame(desiredUpperArmOrientation);

      if (iteration == 0)
      {
         for (OneDoFJointBasics joint : upperArmJointsClone.get(robotSide))
            joint.setQ(0.0);
      }
      else if (iteration >= 15)
      {
         System.err.println("Could not find desired joint angles for the upper arm joints");
         if (DEBUG)
         {
            String msg = "Upper arm QDesireds: ";
            for (int i = 0; i < upperArmJointsClone.get(robotSide).length; i++)
               msg += upperArmJointsClone.get(robotSide)[i] + ", ";
            System.out.println(msg);
         }
         return null;
      }

      temporaryDesiredUpperArmOrientation.checkReferenceFrameMatch(fullRobotModel.getChest().getBodyFixedFrame());

      if (mirrorOrientationForRightSide)
      {
         double[] yawPitchRoll = new double[3];
         temporaryDesiredUpperArmOrientation.getYawPitchRoll(yawPitchRoll);
         yawPitchRoll[0] = robotSide.negateIfRightSide(yawPitchRoll[0]);
         yawPitchRoll[2] = robotSide.negateIfRightSide(yawPitchRoll[2]);
         temporaryDesiredUpperArmOrientation.setYawPitchRoll(yawPitchRoll);
      }

      RigidBodyTransform desiredTransformForUpperArm = new RigidBodyTransform();
      desiredTransformForUpperArm.setRotation(temporaryDesiredUpperArmOrientation);
      desiredTransformForUpperArm.set(desiredTransformForUpperArm);
      desiredTransformForUpperArm.multiply(armZeroJointAngleConfigurationOffsets.get(robotSide));
      boolean success = inverseKinematicsForUpperArms.get(robotSide).solve(desiredTransformForUpperArm);

      if (!success)
      {
         MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, upperArmJointsClone.get(robotSide));
         return computeUpperArmJointAngles(robotSide, temporaryDesiredUpperArmOrientation, false, iteration + 1);
      }

      double[] desiredUpperArmJointAngles = new double[upperArmJointsClone.get(robotSide).length];

      for (int i = 0; i < upperArmJointsClone.get(robotSide).length; i++)
      {
         OneDoFJointBasics joint = upperArmJointsClone.get(robotSide)[i];
         double qDesired = joint.getQ();
         double qLow = joint.getJointLimitLower();
         double qUp = joint.getJointLimitUpper();
         double qRange = qUp - qLow;
         desiredUpperArmJointAngles[i] = MathTools.clamp(qDesired, qLow + 0.01 * qRange, qUp - 0.01 * qRange);
      }

      return desiredUpperArmJointAngles;
   }

   private double[] computeLowerArmJointAngles(RobotSide robotSide, FrameQuaternion desiredHandOrientation, boolean mirrorOrientationForRightSide)
   {
      if (desiredHandOrientation == null)
         return new double[lowerArmJointsClone.get(robotSide).length];

      FrameQuaternion temporaryDesiredHandOrientation = new FrameQuaternion();
      temporaryDesiredHandOrientation.setIncludingFrame(desiredHandOrientation);

      temporaryDesiredHandOrientation.checkReferenceFrameMatch(lowerArmsFrames.get(robotSide));
      //      desiredHandOrientation.applyTransform(armZeroJointAngleConfigurationOffsets.get(robotSide));

      for (OneDoFJointBasics joint : lowerArmJointsClone.get(robotSide))
         joint.setQ(0.0);

      if (mirrorOrientationForRightSide)
      {
         double qx = -temporaryDesiredHandOrientation.getX();
         double qy = temporaryDesiredHandOrientation.getY();
         double qz = -temporaryDesiredHandOrientation.getZ();
         double qs = temporaryDesiredHandOrientation.getS();
         temporaryDesiredHandOrientation.set(qx, qy, qz, qs);
      }

      RigidBodyTransform desiredTransformForLowerArm = new RigidBodyTransform();
      desiredTransformForLowerArm.setRotation(temporaryDesiredHandOrientation);
      boolean success = inverseKinematicsForLowerArms.get(robotSide).solve(desiredTransformForLowerArm);

      if (!success)
      {
         System.err.println("Could not find desired joint angles for the lower arm joints");
         if (DEBUG)
         {
            String msg = "Lower arm QDesireds: ";
            for (int i = 0; i < lowerArmJointsClone.get(robotSide).length; i++)
               msg += lowerArmJointsClone.get(robotSide)[i] + ", ";
            System.out.println(msg);
         }
         return null;
      }

      double[] desiredLowerArmJointAngles = new double[lowerArmJointsClone.get(robotSide).length];

      for (int i = 0; i < lowerArmJointsClone.get(robotSide).length; i++)
      {
         desiredLowerArmJointAngles[i] = lowerArmJointsClone.get(robotSide)[i].getQ();
      }

      return desiredLowerArmJointAngles;
   }

   private void submitFootstepPose(boolean parallelize, RobotSide robotSide, FramePose3D desiredFootstepPose)
   {
      FramePose3D footPose = new FramePose3D(desiredFootstepPose);

      FootstepTask footstepTask = new FootstepTask(fullRobotModel, robotSide, footstepListBehavior, footPose);
      if (parallelize)
         pipeLine.submitTaskForPallelPipesStage(footstepListBehavior, footstepTask);
      else
         pipeLine.submitSingleTaskStage(footstepTask);
   }

   private void submitFootPosition(boolean parallelize, RobotSide robotSide, FramePoint3D desiredFootPosition)
   {
      FrameQuaternion desiredFootOrientation = new FrameQuaternion(desiredFootPosition.getReferenceFrame());
      FramePose3D desiredFootPose = new FramePose3D(desiredFootPosition, desiredFootOrientation);
      submitFootPose(parallelize, robotSide, desiredFootPose);
   }

   private void submitFootPose(boolean parallelize, RobotSide robotSide, FramePose3D desiredFootPose)
   {
      desiredFootPose.changeFrame(worldFrame);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      desiredFootPose.get(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryTask footPoseTask = new FootTrajectoryTask(robotSide, desiredFootPosition, desiredFootOrientation, footPoseBehavior,
                                                               trajectoryTime.getDoubleValue());

      if (parallelize)
      {
         pipeLine.submitTaskForPallelPipesStage(footPoseBehavior, footPoseTask);
         pipeLine.submitTaskForPallelPipesStage(footPoseBehavior, createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));

      }
      else
      {
         pipeLine.submitSingleTaskStage(footPoseTask);
         pipeLine.submitSingleTaskStage(createSleepTask(sleepTimeBetweenPoses.getDoubleValue()));
      }
   }

   private void submitFootPose(boolean parallelize, RobotSide robotSide, ReferenceFrame referenceFrame, double x, double y, double z, double yaw, double pitch,
                               double roll)
   {
      FramePoint3D framePosition = new FramePoint3D(referenceFrame, x, y, z);
      FrameQuaternion frameOrientation = new FrameQuaternion(referenceFrame, yaw, pitch, roll);
      FramePose3D desiredFootPose = new FramePose3D(framePosition, frameOrientation);
      submitFootPose(parallelize, robotSide, desiredFootPose);
   }

   @Override
   public void onBehaviorEntered()
   {
      initialized = false;

   }

   private final FrameQuaternion tempFrameOrientation = new FrameQuaternion();

   @Override
   public void doControl()
   {
      diagnosticBehaviorEnabled.set(isControllerReady());

      handleAutomaticDiagnosticRoutine();

      if (!diagnosticBehaviorEnabled.getBooleanValue())
         return;

      for (RobotSide robotSide : RobotSide.values)
      {
         tempFrameOrientation.setToZero(upperArmsFrames.get(robotSide));
         currentUpperArmOrientations.get(robotSide).setMatchingFrame(tempFrameOrientation);

         tempFrameOrientation.setToZero(fullRobotModel.getHand(robotSide).getBodyFixedFrame());
         currentHandOrientations.get(robotSide).setMatchingFrame(tempFrameOrientation);
      }

      handleRequestedSymmetricArmPose();

      handleRequestedArmPose();

      handleRequestedDiagnostic();

      handleIcpOffsetSending();

      pipeLine.doControl();
   }

   private boolean isControllerReady()
   {
      if (!hasControllerWakenUp.getBooleanValue())
      {
         boolean justReceivedAPacketFromController = inputListeningQueue.poll() != null;
         if (justReceivedAPacketFromController)
         {
            timeWhenControllerWokeUp.set(yoTime.getDoubleValue());
            hasControllerWakenUp.set(true);
         }

         return false;
      }
      else
      {
         boolean isControllerReady = yoTime.getDoubleValue() - timeWhenControllerWokeUp.getDoubleValue() > timeToWaitBeforeEnable.getDoubleValue();
         return isControllerReady;
      }
   }

   private void handleRequestedDiagnostic()
   {
      if (requestedDiagnostic.getEnumValue() != null)
      {
         switch (requestedDiagnostic.getEnumValue())
         {
         case ARM_MOTIONS:
            lastDiagnosticTask.set(DiagnosticTask.ARM_MOTIONS);
            sequenceArmPose(activeSideForHandControl.getEnumValue());
            break;
         case CHEST_ROTATIONS:
            lastDiagnosticTask.set(DiagnosticTask.CHEST_ROTATIONS);
            sequenceChestRotations(0.55);
            break;
         case PELVIS_ROTATIONS:
            lastDiagnosticTask.set(DiagnosticTask.PELVIS_ROTATIONS);
            sequencePelvisRotations(0.3);
            break;
         case COMBINED_CHEST_PELVIS:
            lastDiagnosticTask.set(DiagnosticTask.COMBINED_CHEST_PELVIS);
            sequenceMovingChestAndPelvisOnly();
            break;
         case UPPER_BODY:
            lastDiagnosticTask.set(DiagnosticTask.UPPER_BODY);
            sequenceUpperBody();
            break;
         case FOOT_LIFT:
            lastDiagnosticTask.set(DiagnosticTask.FOOT_LIFT);
            sequenceFootLift();
            break;
         case FOOT_POSES_SHORT:
            lastDiagnosticTask.set(DiagnosticTask.FOOT_POSES_SHORT);
            sequenceFootPoseShort();
            break;
         case FOOT_POSES_LONG:
            lastDiagnosticTask.set(DiagnosticTask.FOOT_POSES_LONG);
            sequenceFootPoseLong();
            break;
         case RUNNING_MAN:
            lastDiagnosticTask.set(DiagnosticTask.RUNNING_MAN);
            runningMan(activeSideForFootControl.getEnumValue());
            //            sequenceRunningMan();
            break;
         case BOW:
            lastDiagnosticTask.set(DiagnosticTask.BOW);
            sequenceBow();
            break;
         case KARATE_KID:
            lastDiagnosticTask.set(DiagnosticTask.KARATE_KID);
            karateKid(activeSideForFootControl.getEnumValue());
            break;
         case STEPS_FORWARD_BACKWARD:
            lastDiagnosticTask.set(DiagnosticTask.STEPS_FORWARD_BACKWARD);
            sequenceWalkForwardBackward(0.5);
            sequenceWalkForwardBackward(1.0);
            break;
         case STEPS_SHORT:
            lastDiagnosticTask.set(DiagnosticTask.STEPS_SHORT);
            sequenceStepsShort();
            break;
         case STEPS_LONG:
            lastDiagnosticTask.set(DiagnosticTask.STEPS_LONG);
            sequenceStepsLong();
            break;
         case WHOLE_SCHEBANG:
            lastDiagnosticTask.set(DiagnosticTask.WHOLE_SCHEBANG);
            sequenceStepsLong();
            sequenceRunningMan();
            karateKid(activeSideForFootControl.getEnumValue());
            sequenceBow();
            break;
         case SQUATS:
            lastDiagnosticTask.set(DiagnosticTask.SQUATS);
            for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
               sequenceSquats();
         case SQUATATHON:
            lastDiagnosticTask.set(DiagnosticTask.SQUATATHON);
            for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
               sequenceSquatathon();
            break;
         case SHIFT_WEIGHT:
            lastDiagnosticTask.set(DiagnosticTask.SHIFT_WEIGHT);
            sequenceShiftWeight();
            break;
         case SIMPLE_WARMUP:
            lastDiagnosticTask.set(DiagnosticTask.SIMPLE_WARMUP);
            sequenceSimpleWarmup();
            break;
         case MEDIUM_WARMUP:
            lastDiagnosticTask.set(DiagnosticTask.MEDIUM_WARMUP);
            sequenceMediumWarmup();
            break;
         case HARD_WARMUP:
            lastDiagnosticTask.set(DiagnosticTask.HARD_WARMUP);
            sequenceHardWarmup();
            break;
         case STEPS_IN_PLACE:
            lastDiagnosticTask.set(DiagnosticTask.STEPS_IN_PLACE);
            sequenceStepsInPlace();
            break;
         case TURN_IN_PLACE_SEQUENCE:
            lastDiagnosticTask.set(DiagnosticTask.TURN_IN_PLACE_SEQUENCE);
            sequenceTurnInPlace();
            break;
         case TURN_IN_PLACE_ANGLE:
            lastDiagnosticTask.set(DiagnosticTask.TURN_IN_PLACE_ANGLE);
            submitTurnInPlaceAngle(false, Math.toRadians(angleToTurnInDegrees.getDoubleValue()));
            break;
         case FEET_SQUARE_UP:
            lastDiagnosticTask.set(DiagnosticTask.FEET_SQUARE_UP);
            sequenceSquareUp();
            break;
         case BOOTY_SHAKE:
            lastDiagnosticTask.set(DiagnosticTask.BOOTY_SHAKE);
            sequenceBootyShake(activeSideForFootControl.getEnumValue());
         case GO_HOME:
            lastDiagnosticTask.set(DiagnosticTask.GO_HOME);
            sequenceGoHome();
            break;
         case ARM_SHAKE:
            lastDiagnosticTask.set(DiagnosticTask.ARM_SHAKE);
            sequenceArmShake(activeSideForHandControl.getEnumValue());
            break;
         case REDO_LAST_TASK:
            if (lastDiagnosticTask.getEnumValue() != null)
            {
               requestedDiagnostic.set(lastDiagnosticTask.getEnumValue());
               handleRequestedDiagnostic();
            }
            break;
         case CUTE_WAVE:
            lastDiagnosticTask.set(DiagnosticTask.CUTE_WAVE);
            sequenceCuteWave();
            break;
         case HAND_SHAKE_PREP:
            lastDiagnosticTask.set(DiagnosticTask.HAND_SHAKE_PREP);
            sequenceHandShakePrep();
            break;
         case HAND_SHAKE_SHAKE:
            lastDiagnosticTask.set(DiagnosticTask.HAND_SHAKE_SHAKE);
            sequenceHandShakeShake();
            break;
         case FLEX_UP:
            lastDiagnosticTask.set(DiagnosticTask.FLEX_UP);
            sequenceFlexUp();
            break;
         case FLEX_DOWN:
            lastDiagnosticTask.set(DiagnosticTask.FLEX_DOWN);
            sequenceFlexDown();
            break;
         case FLEX_UP_FLEX_DOWN:
            lastDiagnosticTask.set(DiagnosticTask.FLEX_UP_FLEX_DOWN);
            sequenceFlexUpFlexDown();
         case KRANE_KICK:
            lastDiagnosticTask.set(DiagnosticTask.KRANE_KICK);
            sequenceKraneKick();
         default:
            break;
         }
         //JJC simply here to get junit test working. they unit test  
         initialized = true;
         requestedDiagnostic.set(null);
      }
   }

   public void requestDiagnosticBehavior(DiagnosticTask diagnosticTask)
   {
      requestedDiagnostic.set(diagnosticTask);
   }

   private void sequenceTurnInPlace()
   {
      submitTurnInPlaceAngle(false, -Math.PI / 2.0 + 0.01); // little values to be sure in which direction the robot will turn
      submitTurnInPlaceAngle(false, Math.PI - 0.02);
      submitTurnInPlaceAngle(false, -Math.PI / 2.0 + 0.01);
   }

   private void submitTurnInPlaceAngle(boolean parallelize, double angleToTurn)
   {
      if (parallelize)
         pipeLine.submitTaskForPallelPipesStage(turnInPlaceBehavior, new TurnInPlaceTask(angleToTurn, turnInPlaceBehavior));
      else
         pipeLine.submitSingleTaskStage(new TurnInPlaceTask(angleToTurn, turnInPlaceBehavior));
   }

   private void sequenceFootLift()
   {
      RobotSide robotSide = activeSideForFootControl.getEnumValue();

      if (robotSide == null)
      {
         for (RobotSide side : RobotSide.values())
         {
            ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(side);
            submitFootPosition(false, side, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, maxFootPoseHeight.getDoubleValue()));
            submitFootPosition(false, side, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, -0.1));
         }
      }
      else
      {
         ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide);
         submitFootPosition(false, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, maxFootPoseHeight.getDoubleValue()));
         submitFootPosition(false, robotSide, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, -0.1));
      }
   }

   private void sequenceBootyShake(RobotSide footSideToPickUp)
   {
      if (footSideToPickUp != null)
      {
         ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(footSideToPickUp);
         submitFootPosition(false, footSideToPickUp, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, maxFootPoseHeight.getDoubleValue()));
      }

      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
      {
         double yaw = (i % 2) == 0 ? 1.0 : -1.0;
         yaw *= this.minMaxYaw * pelvisOrientationScaleFactor.getDoubleValue();
         submitDesiredPelvisOrientation(false, yaw, 0.0, 0.0, bootyShakeTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue());
      }

      submitPelvisHomeCommand(false);

      if (footSideToPickUp != null)
      {
         ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(footSideToPickUp);
         submitFootPosition(false, footSideToPickUp, new FramePoint3D(ankleZUpFrame, 0.0, 0.0, -0.1));
      }
   }

   private void sequenceArmShake(RobotSide armSide)
   {
      double halfPi = Math.PI / 2.0;
      FrameQuaternion desiredUpperArmOrientation = new FrameQuaternion(fullRobotModel.getChest().getBodyFixedFrame());
      boolean mirrorOrientationForRightSide = true;

      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
      {
         double yaw = (i % 2) == 0 ? 1.0 : -1.0;
         yaw += 0.75;
         yaw *= Math.toRadians(10.0);
         desiredUpperArmOrientation.setYawPitchRoll(yaw, 0.0, -1.2);
         if (armSide == null)
            submitSymmetricHandPose(desiredUpperArmOrientation, -halfPi, null);
         else
            submitHandPose(armSide, desiredUpperArmOrientation, -halfPi, null, mirrorOrientationForRightSide);
      }
   }

   private void handleRequestedArmPose()
   {
      if (requestedSingleArmPose.getEnumValue() != null && activeSideForHandControl.getEnumValue() != null)
      {
         submitHumanoidArmPose(activeSideForHandControl.getEnumValue(), requestedSingleArmPose.getEnumValue());
         requestedSingleArmPose.set(null);
      }
   }

   private void handleRequestedSymmetricArmPose()
   {
      if (requestedSymmetricArmPose.getEnumValue() != null)
      {
         submitSymmetricHumanoidArmPose(requestedSymmetricArmPose.getEnumValue());
         requestedSymmetricArmPose.set(null);
      }
   }

   private void handleIcpOffsetSending()
   {
      RigidBodyTransform lastPelvisPoseInWorldFrame = new RigidBodyTransform();
      lastPelvisPoseInWorldFrame.set(fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame());
      stateEstimatorPelvisPoseBuffer.put(lastPelvisPoseInWorldFrame, Conversions.secondsToNanoseconds(yoTime.getDoubleValue()));

      if (isIcpOffsetSenderEnabled.getBooleanValue())
      {
         if (yoTime.getDoubleValue() > previousIcpPacketSentTime.getDoubleValue() + 1.0)
         {
            long timestamp = Conversions.secondsToNanoseconds(yoTime.getDoubleValue() - icpTimeDelay.getDoubleValue());

            TimeStampedTransform3D pelvisTimeStampedTransformInThePast = new TimeStampedTransform3D();
            stateEstimatorPelvisPoseBuffer.findTransform(timestamp, pelvisTimeStampedTransformInThePast);

            RigidBodyTransform pelvisTransformInPast_Translation = new RigidBodyTransform(pelvisTimeStampedTransformInThePast.getTransform3D());
            RigidBodyTransform pelvisTransformInPast_Rotation = new RigidBodyTransform(pelvisTransformInPast_Translation);
            pelvisTransformInPast_Translation.setRotationToZero();
            pelvisTransformInPast_Rotation.setTranslationToZero();

            Quaternion orientationOffset = RandomGeometry.nextQuaternion(random, minMaxIcpAngularOffset.getDoubleValue());
            Vector3D translationOffset = RandomGeometry.nextVector3D(random, minMaxIcpTranslationOffset.getDoubleValue());

            RigidBodyTransform offsetRotationTransform = new RigidBodyTransform(orientationOffset, new Vector3D());
            RigidBodyTransform offsetTranslationTransform = new RigidBodyTransform(new Quaternion(), translationOffset);
            RigidBodyTransform pelvisTransformWithOffset = new RigidBodyTransform();

            pelvisTransformWithOffset.setIdentity();
            pelvisTransformWithOffset.multiply(pelvisTransformInPast_Translation);
            pelvisTransformWithOffset.multiply(offsetTranslationTransform);
            pelvisTransformWithOffset.multiply(pelvisTransformInPast_Rotation);
            pelvisTransformWithOffset.multiply(offsetRotationTransform);

            TimeStampedTransform3D timeStampedTransform3D = new TimeStampedTransform3D(pelvisTransformWithOffset, timestamp);
            stampedPosePublisher.publish(HumanoidMessageTools.createStampedPosePacket("/pelvis", timeStampedTransform3D, 1.0));

            previousIcpPacketSentTime.set(yoTime.getDoubleValue());
         }
      }
   }

   private boolean willStartMessageSent = false;

   private void handleAutomaticDiagnosticRoutine()
   {
      if (!automaticDiagnosticRoutineRequested.getBooleanValue())
         return;
      if (hasControllerWakenUp.getBooleanValue() && !willStartMessageSent)
      {
         System.out.println("\n");
         System.out.println("///////////////////////////////////////////////////////////");
         System.out.println("// Automatic diagnostic routine will start in " + timeToWaitBeforeEnable.getDoubleValue() + " seconds //");
         System.out.println("///////////////////////////////////////////////////////////");
         willStartMessageSent = true;
      }
      if (!diagnosticBehaviorEnabled.getBooleanValue())
         return;
      if (!automaticDiagnosticRoutineHasStarted.getBooleanValue())
      {
         automaticDiagnosticRoutine();
         automaticDiagnosticRoutineHasStarted.set(true);
         System.out.println("\n");
         System.out.println("///////////////////////////////////////////////////////////");
         System.out.println("//         Starting automatic diagnostic routine         //");
         System.out.println("///////////////////////////////////////////////////////////");
      }
      if (automaticDiagnosticRoutineHasStarted.getBooleanValue() && pipeLine.isDone())
      {
         System.out.println("\n");
         System.out.println("///////////////////////////////////////////////////////////");
         System.out.println("//         Automatic diagnostic routine complete         //");
         System.out.println("///////////////////////////////////////////////////////////");
         automaticDiagnosticRoutineRequested.set(false);
         diagnosticBehaviorEnabled.set(false);
      }
   }

   @Override
   public void onBehaviorAborted()
   {
      pipeLine.clearAll();
   }

   @Override
   public void onBehaviorPaused()
   {
      isPaused.set(true);
      pelvisTrajectoryBehavior.pause();
      chestTrajectoryBehavior.pause();
      for (RobotSide robotSide : RobotSide.values)
      {
         handTrajectoryBehaviors.get(robotSide).pause();
         armTrajectoryBehaviors.get(robotSide).pause();
      }
   }

   @Override
   public void onBehaviorResumed()
   {
      isPaused.set(false);
      pelvisTrajectoryBehavior.resume();
      chestTrajectoryBehavior.resume();
      for (RobotSide robotSide : RobotSide.values)
      {
         handTrajectoryBehaviors.get(robotSide).pause();
         armTrajectoryBehaviors.get(robotSide).pause();
      }
   }

   private BehaviorAction createSleepTask(double sleepTime)
   {
      SleepBehavior sleepBehavior = new SleepBehavior(robotName, ros2Node, yoTime);
      SleepTask sleepTask = new SleepTask(sleepBehavior, sleepTime);
      return sleepTask;
   }

   @Override
   public boolean isDone()
   {
      return initialized && pipeLine.isDone();
   }

   @Override
   public void onBehaviorExited()
   {
   }

}
