package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HumanoidArmPose;
import us.ihmc.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestOrientationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseListBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.ChestOrientationTask;
import us.ihmc.humanoidBehaviors.taskExecutor.CoMHeightTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FootPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FootstepTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseListTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.PelvisPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WalkToLocationTask;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.kinematics.NumericalInverseKinematicsCalculator;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJointReferenceFrame;
import us.ihmc.utilities.taskExecutor.NullTask;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;

public class DiagnosticBehavior extends BehaviorInterface
{
   private static final boolean FAST_MOTION = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final SideDependentList<HandPoseBehavior> handPoseBehaviors = new SideDependentList<>();
   private final SideDependentList<HandPoseListBehavior> handPoseListBehaviors = new SideDependentList<>();
   private final FootPoseBehavior footPoseBehavior;
   private final ChestOrientationBehavior chestOrientationBehavior;
   private final PelvisPoseBehavior pelvisPoseBehavior;
   private final FootstepListBehavior footstepListBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final ComHeightBehavior comHeightBehavior;

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable trajectoryTime, flyingTrajectoryTime;
   private final DoubleYoVariable sleepTimeBetweenPoses;
   private final BooleanYoVariable doPelvisAndChestYaw;
   private final IntegerYoVariable numberOfCyclesToRun;
   private final DoubleYoVariable minCoMHeightOffset, maxCoMHeightOffset;
   private final int numberOfArmJoints;
   private final FullRobotModel fullRobotModel;

   private final YoFrameConvexPolygon2d yoSupportPolygon;

   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private final YoFrameVector2d pelvisShiftScaleFactor;

   private enum DiagnosticTask
   {
      CHEST_ROTATIONS, PELVIS_ROTATIONS, SHIFT_WEIGHT, COMBINED_CHEST_PELVIS, ARM_MOTIONS, UPPER_BODY, FOOT_POSES, RUNNING_MAN, BOW, KARATE_KID, WHOLE_SCHEBANG, STEPS, SQUATS, SIMPLE_WARMUP
   };

   private final EnumYoVariable<DiagnosticTask> requestedDiagnostic;
   private final EnumYoVariable<HumanoidArmPose> requestedSymmetricArmPose;
   private final EnumYoVariable<HumanoidArmPose> requestedSingleArmPose;
   private final EnumYoVariable<RobotSide> activeSideForHandControl;
   private final EnumYoVariable<RobotSide> activeSideForFootControl;
   private final EnumYoVariable<RobotSide> supportLeg;

   private final double maxPitch = Math.toRadians(-5.0);
   private final double minPitch = Math.toRadians(40.0);
   private final double minMaxRoll = Math.toRadians(15.0);
   private final double minMaxYaw = Math.toRadians(30.0);
   private final DoubleYoVariable footstepLength;

   private final SideDependentList<ReferenceFrame> upperArmsFrames = new SideDependentList<>();
   private final SideDependentList<OneDoFJoint[]> armJointsClone = new SideDependentList<>();
   private final SideDependentList<NumericalInverseKinematicsCalculator> inverseKinematicsForUpperArms = new SideDependentList<>();
   private final SideDependentList<NumericalInverseKinematicsCalculator> inverseKinematicsForLowerArms = new SideDependentList<>();

   public DiagnosticBehavior(FullRobotModel fullRobotModel, EnumYoVariable<RobotSide> supportLeg, ReferenceFrames referenceFrames, DoubleYoVariable yoTime,
         BooleanYoVariable yoDoubleSupport, OutgoingCommunicationBridgeInterface outgoingCommunicationBridge,
         WalkingControllerParameters walkingControllerParameters, YoFrameConvexPolygon2d yoSupportPolygon)
   {
      super(outgoingCommunicationBridge);

      this.supportLeg = supportLeg;
      this.fullRobotModel = fullRobotModel;
      this.yoSupportPolygon = yoSupportPolygon;

      numberOfArmJoints = fullRobotModel.getRobotSpecificJointNames().getArmJointNames().length;
      this.yoTime = yoTime;
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();

      String behaviorNameFirstLowerCase = FormattingTools.lowerCaseFirstLetter(getName());
      trajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      flyingTrajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "flyingTrajectoryTime", registry);
      trajectoryTime.set(FAST_MOTION ? 0.5 : 3.0);
      flyingTrajectoryTime.set(FAST_MOTION ? 0.5 : 10.0);
      sleepTimeBetweenPoses = new DoubleYoVariable(behaviorNameFirstLowerCase + "SleepTimeBetweenPoses", registry);
      sleepTimeBetweenPoses.set(FAST_MOTION ? 0.0 : 0.5);

      minCoMHeightOffset = new DoubleYoVariable(behaviorNameFirstLowerCase + "MinCoMHeightOffset", registry);
      minCoMHeightOffset.set(-0.15);
      maxCoMHeightOffset = new DoubleYoVariable(behaviorNameFirstLowerCase + "MaxCoMHeightOffset", registry);
      maxCoMHeightOffset.set(0.05);

      footstepLength = new DoubleYoVariable(behaviorNameFirstLowerCase + "FootstepLength", registry);
      footstepLength.set(0.3);

      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      registry.addChild(walkToLocationBehavior.getYoVariableRegistry());

      chestOrientationBehavior = new ChestOrientationBehavior(outgoingCommunicationBridge, yoTime);
      registry.addChild(chestOrientationBehavior.getYoVariableRegistry());

      pelvisPoseBehavior = new PelvisPoseBehavior(outgoingCommunicationBridge, yoTime);
      registry.addChild(pelvisPoseBehavior.getYoVariableRegistry());

      footPoseBehavior = new FootPoseBehavior(outgoingCommunicationBridge, yoTime, yoDoubleSupport);
      registry.addChild(footPoseBehavior.getYoVariableRegistry());

      footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge);
      registry.addChild(footstepListBehavior.getYoVariableRegistry());

      comHeightBehavior = new ComHeightBehavior(outgoingCommunicationBridge, yoTime);
      registry.addChild(comHeightBehavior.getYoVariableRegistry());

      for (RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = robotSide.getCamelCaseNameForMiddleOfExpression();
         HandPoseBehavior handPoseBehavior = new HandPoseBehavior(namePrefix, outgoingCommunicationBridge, yoTime);
         registry.addChild(handPoseBehavior.getYoVariableRegistry());
         handPoseBehaviors.put(robotSide, handPoseBehavior);

         HandPoseListBehavior handPoseListBehavior = new HandPoseListBehavior(namePrefix, outgoingCommunicationBridge, yoTime);
         registry.addChild(handPoseListBehavior.getYoVariableRegistry());
         handPoseListBehaviors.put(robotSide, handPoseListBehavior);
      }

      requestedDiagnostic = new EnumYoVariable<>("requestedDiagnostic", registry, DiagnosticTask.class, true);
      requestedDiagnostic.set(null);

      requestedSymmetricArmPose = new EnumYoVariable<>("requestedSymmetricArmPose", registry, HumanoidArmPose.class, true);
      requestedSymmetricArmPose.set(null);

      requestedSingleArmPose = new EnumYoVariable<>("requestedSingleArmPose", registry, HumanoidArmPose.class, true);
      requestedSingleArmPose.set(null);

      activeSideForFootControl = new EnumYoVariable<>("activeSideForFootControl", registry, RobotSide.class);
      activeSideForFootControl.set(RobotSide.LEFT);

      activeSideForHandControl = new EnumYoVariable<>("activeSideForHandControl", registry, RobotSide.class);
      activeSideForHandControl.set(RobotSide.LEFT);

      numberOfCyclesToRun = new IntegerYoVariable("numberOfDiagnosticCyclesToRun", registry);
      numberOfCyclesToRun.set(2);

      doPelvisAndChestYaw = new BooleanYoVariable("diagnosticDoPelvisAndChestYaw", registry);
      doPelvisAndChestYaw.set(true);

      pelvisShiftScaleFactor = new YoFrameVector2d("DiagnosticPelvisShiftScaleFactor", null, registry);
      pelvisShiftScaleFactor.set(0.6, 0.8);

      double tolerance = 1e-8;
      int maxIterations = 500;
      double maxStepSize = 1.0;
      double minRandomSearchScalar = -0.5;
      double maxRandomSearchScalar = 1.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();
         RigidBody upperArmBody = fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH).getPredecessor();
         RigidBody hand = fullRobotModel.getHand(robotSide);

         upperArmsFrames.put(robotSide, upperArmBody.getBodyFixedFrame());

         OneDoFJoint[] upperArmJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(chest, upperArmBody), OneDoFJoint.class);
         OneDoFJoint[] upperArmJointsClone = ScrewTools.filterJoints(ScrewTools.cloneJointPath(upperArmJoints), OneDoFJoint.class);
         GeometricJacobian upperArmJacobian = new GeometricJacobian(upperArmJointsClone, upperArmsFrames.get(robotSide));
//         NumericalInverseKinematicsCalculator inverseKinematicsForUpperArm = new NumericalInverseKinematicsCalculator(upperArmJacobian, tolerance, maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);
//         inverseKinematicsForUpperArm.solveForPosition(false);
//         inverseKinematicsForUpperArms.put(robotSide, inverseKinematicsForUpperArm);

         OneDoFJoint[] lowerArmJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(upperArmBody, hand), OneDoFJoint.class);
         OneDoFJoint[] lowerArmJointsClone = ScrewTools.filterJoints(ScrewTools.cloneJointPath(lowerArmJoints), OneDoFJoint.class);
         GeometricJacobian lowerArmJacobian = new GeometricJacobian(lowerArmJointsClone, hand.getBodyFixedFrame());
//         NumericalInverseKinematicsCalculator inverseKinematicsForLowerArm = new NumericalInverseKinematicsCalculator(lowerArmJacobian, tolerance, maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);
//         inverseKinematicsForLowerArm.solveForPosition(false);
//         inverseKinematicsForLowerArms.put(robotSide, inverseKinematicsForLowerArm);
         
         armJointsClone.put(robotSide, new OneDoFJoint[upperArmJointsClone.length + lowerArmJointsClone.length]);
         for (int i = 0; i < upperArmJointsClone.length; i++)
            armJointsClone.get(robotSide)[i] = upperArmJointsClone[i];
         for (int i = 0; i < lowerArmJointsClone.length; i++)
            armJointsClone.get(robotSide)[i + upperArmJointsClone.length] = lowerArmJointsClone[i];
      }
   }

   private void sequenceSimpleWarmup()
   {
      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
         sequenceSquats();
      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
         sequenceChestRotations();
      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
         sequencePelvisRotations();
      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
         sequenceShiftWeight();
   }

   private void sequenceUpperBody()
   {
      sequenceArmPose();

      submitSymmetricHandPose(HumanoidArmPose.LARGE_CHICKEN_WINGS.getArmJointAngles());
      sequenceChestRotations();
      sequencePelvisRotations();
      sequenceMovingChestAndPelvisOnly();

      submitSymmetricHandPose(HumanoidArmPose.REACH_FAR_FORWARD.getArmJointAngles());
      sequenceChestRotations();
      sequencePelvisRotations();
      sequenceMovingChestAndPelvisOnly();

      submitSymmetricHandPose(HumanoidArmPose.REACH_FAR_BACK.getArmJointAngles());
      sequenceChestRotations();
      sequencePelvisRotations();
      sequenceMovingChestAndPelvisOnly();

      submitHandPoses(new SideDependentList<double[]>(new double[] { 0.0, -Math.PI / 2.0, 0.0, 0.0 }, HumanoidArmPose.ARM_STRAIGHT_DOWN.getArmJointAngles()));
      sequenceChestRotations();
      sequencePelvisRotations();
      sequenceMovingChestAndPelvisOnly();

      submitHandPoses(new SideDependentList<double[]>(HumanoidArmPose.ARM_STRAIGHT_DOWN.getArmJointAngles(), new double[] { 0.0, -Math.PI / 2.0, 0.0, 0.0 }));
      sequenceChestRotations();
      sequencePelvisRotations();
      sequenceMovingChestAndPelvisOnly();

      submitSymmetricHandPose(HumanoidArmPose.STAND_PREP.getArmJointAngles());
   }

   private void sequenceChestRotations()
   {
      double percentOfJointLimit = 0.55;
      double roll = percentOfJointLimit * minMaxRoll;
      submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, 0.0, percentOfJointLimit * minPitch, 0.0));
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, minMaxYaw, percentOfJointLimit * minPitch, 0.0));
         submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, -minMaxYaw, percentOfJointLimit * minPitch, 0.0));
      }
      submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, 0.0, 0.0, roll));
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, minMaxYaw, 0.0, roll));
         submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, -minMaxYaw, 0.0, roll));
      }
      submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, 0.0, 0.0, -roll));
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, minMaxYaw, 0.0, -roll));
         submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, -minMaxYaw, 0.0, -roll));
      }
      submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, 0.0, percentOfJointLimit * minPitch, roll));
      submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, 0.0, percentOfJointLimit * minPitch, -roll));

      submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, 0.0, 0.0, 0.0));
   }

   private void sequencePelvisRotations()
   {
      double percentOfJointLimit = 0.3;
      double roll = percentOfJointLimit * minMaxRoll;
      double yaw = percentOfJointLimit * minMaxYaw;
      submitDesiredPelvisOrientation(0.0, percentOfJointLimit * minPitch, 0.0);
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredPelvisOrientation(yaw, percentOfJointLimit * minPitch, 0.0);
         submitDesiredPelvisOrientation(-yaw, percentOfJointLimit * minPitch, 0.0);
      }
      submitDesiredPelvisOrientation(0.0, 0.0, roll);
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredPelvisOrientation(yaw, 0.0, roll);
         submitDesiredPelvisOrientation(-yaw, 0.0, roll);
      }
      submitDesiredPelvisOrientation(0.0, 0.0, -roll);
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredPelvisOrientation(yaw, 0.0, -roll);
         submitDesiredPelvisOrientation(-yaw, 0.0, -roll);
      }
      submitDesiredPelvisOrientation(0.0, percentOfJointLimit * minPitch, roll);
      submitDesiredPelvisOrientation(0.0, percentOfJointLimit * minPitch, -roll);

      submitPelvisHomeCommand();
   }

   private void sequenceShiftWeight()
   {
      FramePoint2d center = new FramePoint2d(midFeetZUpFrame);

      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(yoSupportPolygon.getFrameConvexPolygon2d());
      supportPolygon.changeFrameAndProjectToXYPlane(midFeetZUpFrame);

      FrameVector2d desiredPelvisOffset = new FrameVector2d(midFeetZUpFrame);

      for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
      {
         desiredPelvisOffset.set(supportPolygon.getFrameVertex(i));
         desiredPelvisOffset.sub(center);
         submitDesiredPelvisPositionOffset(pelvisShiftScaleFactor.getX() * desiredPelvisOffset.getX(), pelvisShiftScaleFactor.getY() * desiredPelvisOffset.getY(), 0.0);
      }
      // Get back to the first vertex again
      desiredPelvisOffset.set(supportPolygon.getFrameVertex(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(pelvisShiftScaleFactor.getX() * desiredPelvisOffset.getX(), pelvisShiftScaleFactor.getY() * desiredPelvisOffset.getY(), 0.0);

      submitPelvisHomeCommand();
   }

   private void sequenceMovingChestAndPelvisOnly()
   {
      double percentOfJointLimit = 0.8;
      double percentOfJointLimitForPelvis = 0.5;
      submitDesiredChestAndPelvisOrientationsParallel(new FrameOrientation(pelvisZUpFrame, 0.0, percentOfJointLimit * minPitch, 0.0), 0.0,
            percentOfJointLimitForPelvis * minPitch, 0.0);
      submitDesiredChestAndPelvisOrientationsParallel(new FrameOrientation(pelvisZUpFrame, 0.0, percentOfJointLimit * maxPitch, 0.0), 0.0,
            percentOfJointLimitForPelvis * maxPitch, 0.0);

      submitDesiredChestAndPelvisOrientationsParallel(new FrameOrientation(pelvisZUpFrame, 0.0, 0.0, percentOfJointLimit * minMaxRoll), 0.0, 0.0,
            percentOfJointLimitForPelvis * minMaxRoll);
      submitDesiredChestAndPelvisOrientationsParallel(new FrameOrientation(pelvisZUpFrame, 0.0, 0.0, -percentOfJointLimit * minMaxRoll), 0.0, 0.0,
            -percentOfJointLimitForPelvis * minMaxRoll);

      submitDesiredChestAndPelvisOrientationsParallel(new FrameOrientation(pelvisZUpFrame, 0.0, percentOfJointLimit * minPitch, 0.0), 0.0,
            percentOfJointLimitForPelvis * minPitch, percentOfJointLimitForPelvis * minMaxRoll);
      submitDesiredChestAndPelvisOrientationsParallel(new FrameOrientation(pelvisZUpFrame, 0.0, percentOfJointLimit * minPitch, 0.0), 0.0,
            percentOfJointLimitForPelvis * minPitch, -percentOfJointLimitForPelvis * minMaxRoll);
      submitDesiredChestAndPelvisOrientationsParallel(new FrameOrientation(pelvisZUpFrame, 0.0, percentOfJointLimit * maxPitch, 0.0), 0.0,
            percentOfJointLimitForPelvis * maxPitch, -percentOfJointLimitForPelvis * minMaxRoll);
      submitDesiredChestAndPelvisOrientationsParallel(new FrameOrientation(pelvisZUpFrame, 0.0, percentOfJointLimit * maxPitch, 0.0), 0.0,
            percentOfJointLimitForPelvis * maxPitch, percentOfJointLimitForPelvis * minMaxRoll);

      submitDesiredChestAndPelvisOrientationsParallel(new FrameOrientation(pelvisZUpFrame, 0.0, 0.0, 0.0), 0.0, 0.0, 0.0);
   }

   private void sequenceArmPose()
   {
      submitSymmetricHandPose(HumanoidArmPose.STAND_PREP.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.REACH_BACK.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.REACH_WAY_BACK.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.ARMS_03.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.REACH_FORWARD.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.SMALL_CHICKEN_WINGS.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.LARGE_CHICKEN_WINGS.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.STRAIGHTEN_ELBOWS.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.SUPPINATE_ARMS_IN_A_LITTLE.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.ARMS_BACK.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.LARGER_CHICKEN_WINGS.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.ARMS_OUT_EXTENDED.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.SUPPINATE_ARMS_IN_MORE.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.SUPPINATE_ARMS_IN_A_LOT.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.SUPER_CHICKEN_WINGS.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.FLYING.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.FLYING_SUPPINATE_IN.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.FLYING_SUPPINATE_OUT.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.ARM_NINETY_ELBOW_DOWN.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.ARM_NINETY_ELBOW_FORWARD.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.ARM_NINETY_ELBOW_UP.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.ARM_FORTFIVE_ELBOW_UP.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.ARM_FORTFIVE_ELBOW_DOWN.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.ARM_OUT_TRICEP_EXERCISE.getArmJointAngles());
      submitSymmetricHandPose(HumanoidArmPose.ARM_STRAIGHT_DOWN.getArmJointAngles());
   }

   private void sequenceFootPose()
   {
      RobotSide robotSide = activeSideForFootControl.getEnumValue();
      ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide);

      submitFootPosition(robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, 0.1));
      submitFootPosition(robotSide, new FramePoint(ankleZUpFrame, 0.1, 0.0, 0.1));
      submitFootPosition(robotSide, new FramePoint(ankleZUpFrame, -0.1, 0.0, 0.1));
      submitFootPosition(robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, 0.1));
      submitFootPosition(robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, -0.1));
   }

   private void sequenceRunningMan()
   {
      for (RobotSide robotSide : RobotSide.values)
         runningMan(robotSide);
   }

   private void runningMan(RobotSide robotSide)
   {
      ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide.getOppositeSide());
      // First Lift up the foot
      submitFootPosition(robotSide, new FramePoint(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.25), 0.1));

      // Go to running man pose:
      SideDependentList<double[]> desiredJointAngles = new SideDependentList<>();
      desiredJointAngles.put(robotSide, new double[] { -Math.PI / 2.0, -0.3, 0.0, -Math.PI / 2.0 });
      desiredJointAngles.put(robotSide.getOppositeSide(), new double[] { Math.PI / 2.0, -0.3, 0.0, -Math.PI / 2.0 });
      submitHandPoses(desiredJointAngles);

      FramePose footPose = new FramePose(ankleZUpFrame);
      footPose.setPosition(-0.40, robotSide.negateIfRightSide(0.25), 0.40);
      footPose.setOrientation(0.0, 0.8 * Math.PI / 2.0, 0.0);
      footPose.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(footPoseBehavior, new FootPoseTask(robotSide, footPose, yoTime, footPoseBehavior, trajectoryTime.getDoubleValue(),
            sleepTimeBetweenPoses.getDoubleValue()));

      FrameOrientation desiredChestOrientation = new FrameOrientation(pelvisZUpFrame, 0.0, Math.toRadians(20.0), 0.0);
      desiredChestOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior, new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior,
            trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
      FrameOrientation desiredPelvisOrientation = new FrameOrientation(findFixedFrameForPelvisOrientation());
      desiredPelvisOrientation.setYawPitchRoll(0.0, Math.toRadians(10.0), 0.0);
      desiredPelvisOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior,
            new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      pipeLine.submitSingleTaskStage(new NullTask());

      // Do a "Y" stance with the foot outside
      submitSymmetricHandPose(new double[] { 0.0, -1.5 * Math.PI / 2.0, 0.0, 0.0 });

      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.65), 0.1);
      footPose.setOrientation(0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(40.0)));
      footPose.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(footPoseBehavior, new FootPoseTask(robotSide, footPose, yoTime, footPoseBehavior, trajectoryTime.getDoubleValue(),
            sleepTimeBetweenPoses.getDoubleValue()));
      desiredChestOrientation.setIncludingFrame(pelvisZUpFrame, 0.0, 0.0, 0.0);
      desiredChestOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior, new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior,
            trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
      desiredPelvisOrientation.setToZero(findFixedFrameForPelvisOrientation());
      desiredPelvisOrientation.setYawPitchRoll(0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(25.0)));
      desiredPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior,
            new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      pipeLine.submitSingleTaskStage(new NullTask());

      // Go back to stand prep but don't put the foot on the ground yet
      submitSymmetricHandPose(HumanoidArmPose.STAND_PREP.getArmJointAngles());

      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.25), 0.1);
      footPose.setOrientation(0.0, 0.0, 0.0);
      footPose.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(footPoseBehavior, new FootPoseTask(robotSide, footPose, yoTime, footPoseBehavior, trajectoryTime.getDoubleValue(),
            sleepTimeBetweenPoses.getDoubleValue()));

      desiredPelvisOrientation.setToZero(findFixedFrameForPelvisOrientation());
      desiredPelvisOrientation.setYawPitchRoll(0.0, 0.0, 0.0);
      desiredPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior,
            new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      // Put the foot back on the ground
      submitFootPosition(robotSide, new FramePoint(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.25), -0.3));
   }

   private void sequenceBow()
   {
      //      for (RobotSide robotSide : RobotSide.values)
      bow(RobotSide.LEFT);
   }

   private void bow(RobotSide robotSide)
   {
      ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide.getOppositeSide());

      //put the foot forward and prepare the arms
      FramePose desiredFootstepPosition = new FramePose(ankleZUpFrame);
      Point3d position = new Point3d(0.2, robotSide.negateIfRightSide(0.25), 0.0);
      Quat4d orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      desiredFootstepPosition.setPose(position, orientation);
      desiredFootstepPosition.changeFrame(worldFrame);
      submitFootstepPosition(robotSide, desiredFootstepPosition);

      SideDependentList<double[]> desiredJointAngles = new SideDependentList<>();
      desiredJointAngles.put(robotSide, new double[] { Math.toRadians(30), -0.3, 0.0, -0.1 });
      desiredJointAngles.put(robotSide.getOppositeSide(), new double[] { -Math.toRadians(30), -0.3, 0.0, -0.1 });
      submitHandPoses(desiredJointAngles);

      pipeLine.requestNewStage();

      desiredJointAngles = new SideDependentList<>();
      desiredJointAngles.put(robotSide, new double[] { Math.toRadians(30), -0.3, Math.PI / 2, -0.1 });
      desiredJointAngles.put(robotSide.getOppositeSide(), new double[] { -Math.toRadians(30), -0.3, Math.PI / 2, -0.1 });
      submitHandPoses(desiredJointAngles);

      pipeLine.requestNewStage();

      //bend forward and arms
      desiredJointAngles = new SideDependentList<>();
      desiredJointAngles.put(robotSide, new double[] { Math.toRadians(30), -0.3, Math.PI / 2, -Math.PI / 2 });
      desiredJointAngles.put(robotSide.getOppositeSide(), new double[] { -Math.toRadians(30), -0.3, Math.PI / 2, -Math.PI / 2 });
      submitHandPoses(desiredJointAngles);

      FrameOrientation desiredPelvisOrientation = new FrameOrientation(findFixedFrameForPelvisOrientation());
      desiredPelvisOrientation.setYawPitchRoll(0.0, Math.toRadians(30), 0.0);
      desiredPelvisOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior,
            new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      FrameOrientation desiredChestOrientation = new FrameOrientation(pelvisZUpFrame, 0.0, Math.toRadians(35.0), 0.0);
      desiredChestOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior, new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior,
            trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      pipeLine.requestNewStage();

      //back to normal stance
      desiredChestOrientation = new FrameOrientation(pelvisZUpFrame, 0.0, 0.0, 0.0);
      desiredChestOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior, new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior,
            trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      desiredPelvisOrientation = new FrameOrientation(findFixedFrameForPelvisOrientation());
      desiredPelvisOrientation.setYawPitchRoll(0.0, 0.0, 0.0);
      desiredPelvisOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior,
            new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      pipeLine.requestNewStage();

      desiredJointAngles = new SideDependentList<>();
      desiredJointAngles.put(robotSide, new double[] { Math.toRadians(30), -0.3, Math.PI / 2, -0.1 });
      desiredJointAngles.put(robotSide.getOppositeSide(), new double[] { -Math.toRadians(30), -0.3, Math.PI / 2, -0.1 });
      submitHandPoses(desiredJointAngles);

      desiredFootstepPosition = new FramePose(ankleZUpFrame);
      position = new Point3d(0., robotSide.negateIfRightSide(0.25), 0.0);
      orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      desiredFootstepPosition.setPose(position, orientation);
      desiredFootstepPosition.changeFrame(worldFrame);
      submitFootstepPosition(robotSide, desiredFootstepPosition);

      pipeLine.requestNewStage();

      // Put the foot back on the ground
      // Put the foot back on the ground
      //arms at good spot
      desiredJointAngles = new SideDependentList<>();
      desiredJointAngles.put(robotSide, new double[] { 0.0, -0.3, 0.0, -0.2 });
      desiredJointAngles.put(robotSide.getOppositeSide(), new double[] { 0.0, -0.3, 0.0, -0.2 });
      submitHandPoses(desiredJointAngles);

      pipeLine.requestNewStage();

      desiredJointAngles = new SideDependentList<>();
      desiredJointAngles.put(robotSide, new double[] { 0.0, -0.3, 0.0, -Math.PI / 2 });
      desiredJointAngles.put(robotSide.getOppositeSide(), new double[] { 0.0, -0.3, 0.0, -Math.PI / 2 });
      submitHandPoses(desiredJointAngles);

      pipeLine.requestNewStage();
   }

   private void sequenceSteps()
   {
      pipeLine.requestNewStage();

      // forward
      FramePose2d targetPoseInWorld = new FramePose2d();
      targetPoseInWorld.setPoseIncludingFrame(midFeetZUpFrame, 0.5, 0.0, 0.0);
      targetPoseInWorld.changeFrame(worldFrame);
      pipeLine.submitSingleTaskStage(new WalkToLocationTask(targetPoseInWorld, walkToLocationBehavior, 0.0, footstepLength.getDoubleValue(), yoTime));

      pipeLine.requestNewStage();

      //backward
      targetPoseInWorld = new FramePose2d();
      targetPoseInWorld.setPoseIncludingFrame(midFeetZUpFrame, 0.0, 0.0, 0.0);
      targetPoseInWorld.changeFrame(worldFrame);
      pipeLine.submitSingleTaskStage(new WalkToLocationTask(targetPoseInWorld, walkToLocationBehavior, Math.PI, footstepLength.getDoubleValue(), yoTime));

      pipeLine.requestNewStage();

      //sideWalk
      targetPoseInWorld = new FramePose2d();
      targetPoseInWorld.setPoseIncludingFrame(midFeetZUpFrame, 0.5, 0.0, -Math.PI / 2.0);
      targetPoseInWorld.changeFrame(worldFrame);
      pipeLine.submitSingleTaskStage(new WalkToLocationTask(targetPoseInWorld, walkToLocationBehavior, -Math.PI / 2, footstepLength.getDoubleValue(), yoTime));

      pipeLine.requestNewStage();

      targetPoseInWorld = new FramePose2d();
      targetPoseInWorld.setPoseIncludingFrame(midFeetZUpFrame, 0.0, 0.0, 0.0);
      targetPoseInWorld.changeFrame(worldFrame);
      pipeLine.submitSingleTaskStage(new WalkToLocationTask(targetPoseInWorld, walkToLocationBehavior, Math.PI / 2.0, footstepLength.getDoubleValue(), yoTime));

      pipeLine.requestNewStage();
      //      
      //      targetPoseInWorld = new FramePose2d();
      //      targetPoseInWorld.setPose(worldFrame, 0.5, 0.0, 0.0);
      //      pipeLine.submit(new WalkToLocationTask(targetPoseInWorld, walkToLocationBehavior, 0.0,footstepLength.getDoubleValue()));
      //      
      //      pipeLine.requestNextStage();

      //      pipeLine.requestNextStage();
      //      
      //      targetPoseInWorld = new FramePose2d();
      //      targetPoseInWorld.setPose(worldFrame, 0.01, 0.0, Math.PI/4.0);
      //      pipeLine.submit(new WalkToLocationTask(targetPoseInWorld, walkToLocationBehavior, 0.0,footstepLength.getDoubleValue()));
      //      //sideWalk 180degrees
      //      targetPoseInWorld = new FramePose2d();
      //      targetPoseInWorld.setPose(worldFrame, 0.5, 0.0, -Math.PI/2);
      //      pipeLine.submit(new WalkToLocationTask(targetPoseInWorld, walkToLocationBehavior, -Math.PI/2,footstepLength.getDoubleValue()));
      //      
      //      pipeLine.requestNextStage();
      //      
      //      targetPoseInWorld = new FramePose2d();
      //      targetPoseInWorld.setPose(worldFrame, 0.0, 0.0, -Math.PI/2);
      //      pipeLine.submit(new WalkToLocationTask(targetPoseInWorld, walkToLocationBehavior, Math.PI/2,footstepLength.getDoubleValue()));
   }

   private void karateKid(RobotSide robotSide)
   {
      ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide.getOppositeSide());
      // First Lift up the foot
      submitFootPosition(robotSide, new FramePoint(ankleZUpFrame, 0.1, robotSide.negateIfRightSide(0.25), 0.2));

      // Put the arm down
      double halfPi = Math.PI / 2.0;
      double[] armDown2 = ensureJointAnglesSize(new double[] { 0.0, -0.4, halfPi / 2.0, 0.0 });
      double[] armIntermediateOnWayUp = ensureJointAnglesSize(new double[] { 0.0, -halfPi, halfPi / 2.0, -halfPi / 2.0 });
      double[] armUp1 = ensureJointAnglesSize(new double[] { 0.0, -1.5 * halfPi, halfPi / 2.0, 0.0 });
      double[] armUp2 = ensureJointAnglesSize(new double[] { 0.0, -1.5 * halfPi, -halfPi / 2.0, 0.0 });
      double[] armIntermediateOnWayDown = ensureJointAnglesSize(new double[] { 0.0, -halfPi, -halfPi / 2.0, -halfPi / 2.0 });
      double[] armDown1 = ensureJointAnglesSize(new double[] { 0.0, -0.4, -halfPi / 2.0, 0.0 });

      int numberOfHandPoses = 10;
      double[][] armFlyingSequence = new double[numberOfArmJoints][numberOfHandPoses];

      for (int jointIndex = 0; jointIndex < numberOfArmJoints; jointIndex++)
      {
         for (int poseIndex = 0; poseIndex < numberOfHandPoses; poseIndex++)
         {
            double desiredJointAngle;
            switch (poseIndex % 6)
            {
               case 0:
                  desiredJointAngle = armDown1[jointIndex];
                  break;
               case 1:
                  desiredJointAngle = armDown2[jointIndex];
                  break;
               case 2:
                  desiredJointAngle = armIntermediateOnWayUp[jointIndex];
                  break;
               case 3:
                  desiredJointAngle = armUp1[jointIndex];
                  break;
               case 4:
                  desiredJointAngle = armUp2[jointIndex];
                  break;
               case 5:
                  desiredJointAngle = armIntermediateOnWayDown[jointIndex];
                  break;
               default:
                  throw new RuntimeException("Should not get there!");
            }

            armFlyingSequence[jointIndex][poseIndex] = desiredJointAngle;
         }
      }

      for (RobotSide tempSide : RobotSide.values)
      {
         HandPoseListPacket handPoseListPacket = new HandPoseListPacket(tempSide, armFlyingSequence, flyingTrajectoryTime.getDoubleValue() * numberOfHandPoses);
         pipeLine.submitTaskForPallelPipesStage(handPoseListBehaviors.get(tempSide),
               new HandPoseListTask(handPoseListPacket, handPoseListBehaviors.get(tempSide), yoTime, sleepTimeBetweenPoses.getDoubleValue()));
      }

      // Put the arms in front
      double[] armInFront = new double[] { -0.5, -0.6, 1.0, -halfPi };
      submitSymmetricHandPose(new double[] { 0.0, -0.6, 0.0, -halfPi });

      pipeLine.requestNewStage();

      submitSymmetricHandPose(armInFront);

      // Supa powerful front kick!!!!!
      FramePose footPose = new FramePose();
      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.75, robotSide.negateIfRightSide(0.25), 0.25);
      footPose.setOrientation(0.0, -halfPi / 2.0, 0.0);
      footPose.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(footPoseBehavior, new FootPoseTask(robotSide, footPose, yoTime, footPoseBehavior, trajectoryTime.getDoubleValue(),
            sleepTimeBetweenPoses.getDoubleValue()));

      FrameOrientation desiredChestOrientation = new FrameOrientation(pelvisZUpFrame, 0.0, Math.toRadians(-5.0), 0.0);
      desiredChestOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior, new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior,
            trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
      FrameOrientation desiredPelvisOrientation = new FrameOrientation(findFixedFrameForPelvisOrientation());
      desiredPelvisOrientation.setYawPitchRoll(0.0, Math.toRadians(-15.0), 0.0);
      desiredPelvisOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior,
            new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      pipeLine.requestNewStage();

      // Supa powerful back kick!!!!!
      SideDependentList<double[]> desiredJointAngles = new SideDependentList<>();
      desiredJointAngles.put(robotSide, new double[] { -0.8 * Math.PI / 2.0, -0.3, 0.4, -0.3 });
      desiredJointAngles.put(robotSide.getOppositeSide(), new double[] { 0.8 * Math.PI / 2.0, -0.3, 0.0, -0.3 });
      submitHandPoses(desiredJointAngles);
      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(-0.75, robotSide.negateIfRightSide(0.25), 0.35);
      footPose.setOrientation(0.0, 0.8 * halfPi, 0.0);
      footPose.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(footPoseBehavior, new FootPoseTask(robotSide, footPose, yoTime, footPoseBehavior, trajectoryTime.getDoubleValue(),
            sleepTimeBetweenPoses.getDoubleValue()));

      desiredChestOrientation.setIncludingFrame(pelvisZUpFrame, 0.0, Math.toRadians(30.0), 0.0);
      desiredChestOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior, new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior,
            trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
      desiredPelvisOrientation.setIncludingFrame(findFixedFrameForPelvisOrientation(), 0.0, Math.toRadians(20.0), 0.0);
      desiredPelvisOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior,
            new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      pipeLine.requestNewStage();

      // Supa powerful side kick!!!!!
      desiredJointAngles.put(robotSide, new double[] { 0.0, -halfPi, 0.4, -0.1 });
      desiredJointAngles.put(robotSide.getOppositeSide(), new double[] { -Math.PI / 2.0, -0.3, halfPi, -halfPi });
      submitHandPoses(desiredJointAngles);
      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.85), 0.3);
      footPose.setOrientation(0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(40.0)));
      footPose.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(footPoseBehavior, new FootPoseTask(robotSide, footPose, yoTime, footPoseBehavior, trajectoryTime.getDoubleValue(),
            sleepTimeBetweenPoses.getDoubleValue()));

      desiredChestOrientation.setIncludingFrame(pelvisZUpFrame, 0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(30.0)));
      desiredChestOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior, new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior,
            trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
      desiredPelvisOrientation.setIncludingFrame(findFixedFrameForPelvisOrientation(), 0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(20.0)));
      desiredPelvisOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior,
            new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      pipeLine.requestNewStage();

      // Go back to stand prep but don't put the foot on the ground yet
      submitSymmetricHandPose(HumanoidArmPose.STAND_PREP.getArmJointAngles());

      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.25), 0.1);
      footPose.setOrientation(0.0, 0.0, 0.0);
      footPose.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(footPoseBehavior, new FootPoseTask(robotSide, footPose, yoTime, footPoseBehavior, trajectoryTime.getDoubleValue(),
            sleepTimeBetweenPoses.getDoubleValue()));

      desiredChestOrientation.setIncludingFrame(pelvisZUpFrame, 0.0, 0.0, 0.0);
      desiredChestOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior, new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior,
            trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
      desiredPelvisOrientation.setToZero(findFixedFrameForPelvisOrientation());
      desiredPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior,
            new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      // Put the foot back on the ground
      submitFootPosition(robotSide, new FramePoint(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.25), -0.3));
   }

   private void sequenceSquats()
   {
      submitDesiredCoMHeightOffset(minCoMHeightOffset.getDoubleValue());
      submitDesiredCoMHeightOffset(maxCoMHeightOffset.getDoubleValue());
   }

   private ReferenceFrame findFixedFrameForPelvisOrientation()
   {
      if (supportLeg.getEnumValue() == null)
         return midFeetZUpFrame;
      else
         return ankleZUpFrames.get(supportLeg.getEnumValue());
   }

   private void submitDesiredChestOrientation(FrameOrientation desiredChestOrientation)
   {
      desiredChestOrientation.checkReferenceFrameMatch(pelvisZUpFrame);
      desiredChestOrientation.changeFrame(worldFrame);
      pipeLine.submitSingleTaskStage(new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior, trajectoryTime.getDoubleValue(),
            sleepTimeBetweenPoses.getDoubleValue()));
   }

   private void submitDesiredCoMHeightOffset(double offsetHeight)
   {
      pipeLine.submitSingleTaskStage(new CoMHeightTask(offsetHeight, yoTime, comHeightBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses
            .getDoubleValue()));
   }

   private void submitPelvisHomeCommand()
   {
      PelvisPosePacket homePelvisPacket = PacketControllerTools.createGoToHomePelvisPosePacket(trajectoryTime.getDoubleValue());
      pipeLine.submitSingleTaskStage(new PelvisPoseTask(homePelvisPacket, yoTime, pelvisPoseBehavior, sleepTimeBetweenPoses.getDoubleValue()));
   }

   private void submitDesiredPelvisOrientation(double yaw, double pitch, double roll)
   {
      FrameOrientation desiredPelvisOrientation = new FrameOrientation(findFixedFrameForPelvisOrientation(), yaw, pitch, roll);
      desiredPelvisOrientation.changeFrame(worldFrame);
      pipeLine.submitSingleTaskStage(new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(),
            sleepTimeBetweenPoses.getDoubleValue()));
   }

   private void submitDesiredPelvisPositionOffset(double dx, double dy, double dz)
   {
      SixDoFJointReferenceFrame frameAfterRootJoint = fullRobotModel.getRootJoint().getFrameAfterJoint();
      FramePoint desiredPelvisPosition = new FramePoint(frameAfterRootJoint, dx, dy, dz);
      desiredPelvisPosition.changeFrame(worldFrame);
      pipeLine.submitSingleTaskStage(new PelvisPoseTask(desiredPelvisPosition, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(),
            sleepTimeBetweenPoses.getDoubleValue()));
   }

   private void submitDesiredChestAndPelvisOrientationsParallel(FrameOrientation desiredChestOrientation, double pelvisYaw, double pelvisPitch,
         double pelvisRoll)
   {
      desiredChestOrientation.checkReferenceFrameMatch(pelvisZUpFrame);
      desiredChestOrientation.changeFrame(worldFrame);
      FrameOrientation desiredPelvisOrientation = new FrameOrientation(findFixedFrameForPelvisOrientation(), pelvisYaw, pelvisPitch, pelvisRoll);
      desiredPelvisOrientation.changeFrame(worldFrame);
      pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior, new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior,
            trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
      pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior,
            new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
   }

   private void submitSingleHandPose(RobotSide robotSide, double[] desiredJointAngles)
   {
      double[] temp = ensureJointAnglesSize(desiredJointAngles);

      HandPoseBehavior handPoseBehavior = handPoseBehaviors.get(robotSide);
      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(robotSide, temp, yoTime, handPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
   }

   private double[] ensureJointAnglesSize(double[] desiredJointAngles)
   {
      double[] ret;
      double[] src = desiredJointAngles;
      if (numberOfArmJoints > src.length)
      {
         ret = new double[numberOfArmJoints];
         System.arraycopy(src, 0, ret, 0, src.length);
      }
      else
         ret = src;
      return ret;
   }

   private void submitHandPoses(SideDependentList<double[]> desiredJointAngles)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         double[] temp = new double[numberOfArmJoints];
         double[] src = desiredJointAngles.get(robotSide);
         if (numberOfArmJoints > src.length)
         {
            System.arraycopy(src, 0, temp, 0, src.length);
         }
         else
            temp = src;

         HandPoseBehavior handPoseBehavior = handPoseBehaviors.get(robotSide);
         pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(robotSide, temp, yoTime, handPoseBehavior, trajectoryTime.getDoubleValue(),
               sleepTimeBetweenPoses.getDoubleValue()));
      }
   }

   private void submitSymmetricHandPose(double[] desiredJointAngles)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (numberOfArmJoints > desiredJointAngles.length)
         {
            double[] temp = new double[numberOfArmJoints];
            System.arraycopy(desiredJointAngles, 0, temp, 0, desiredJointAngles.length);
            desiredJointAngles = temp;
         }

         HandPoseBehavior handPoseBehavior = handPoseBehaviors.get(robotSide);
         pipeLine.submitTaskForPallelPipesStage(
               handPoseBehavior,
               new HandPoseTask(robotSide, desiredJointAngles, yoTime, handPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses
                     .getDoubleValue()));
      }
   }

   public void submitSymmetricHandPose(FrameOrientation desiredUpperArmOrientation, FrameOrientation desiredHandOrientation)
   {
      desiredUpperArmOrientation.checkReferenceFrameMatch(fullRobotModel.getChest().getBodyFixedFrame());
      desiredHandOrientation.checkReferenceFrameMatch(upperArmsFrames.get(RobotSide.LEFT));
      
      FrameOrientation tempUpperArmFrameOrientation = new FrameOrientation();
      FrameOrientation tempHandFrameOrientation = new FrameOrientation();

      for (RobotSide robotSide : RobotSide.values)
      {
         double[] yawPitchRollForUpperArm = desiredUpperArmOrientation.getYawPitchRoll();
         yawPitchRollForUpperArm[0] = robotSide.negateIfRightSide(yawPitchRollForUpperArm[0]);
         yawPitchRollForUpperArm[2] = robotSide.negateIfRightSide(yawPitchRollForUpperArm[2]);
      
         tempUpperArmFrameOrientation.setToZero(fullRobotModel.getChest().getBodyFixedFrame());
         tempUpperArmFrameOrientation.setYawPitchRoll(yawPitchRollForUpperArm);
         
         double[] yawPitchRollForHand = desiredHandOrientation.getYawPitchRoll();
         yawPitchRollForHand[0] = robotSide.negateIfRightSide(yawPitchRollForHand[0]);
         yawPitchRollForHand[2] = robotSide.negateIfRightSide(yawPitchRollForHand[2]);

         tempHandFrameOrientation.setToZero(upperArmsFrames.get(robotSide));
         tempHandFrameOrientation.setYawPitchRoll(yawPitchRollForUpperArm);
         
         submitHandPose(robotSide, tempUpperArmFrameOrientation, tempHandFrameOrientation);
      }
   }

   public void submitHandPose(RobotSide robotSide, FrameOrientation desiredUpperArmOrientation, FrameOrientation desiredHandOrientation)
   {
      desiredUpperArmOrientation.checkReferenceFrameMatch(fullRobotModel.getChest().getBodyFixedFrame());
      desiredHandOrientation.checkReferenceFrameMatch(upperArmsFrames.get(robotSide));

      boolean success = true;

      RigidBodyTransform desiredTransform = new RigidBodyTransform();
      desiredUpperArmOrientation.getTransform3D(desiredTransform);
      success &= inverseKinematicsForUpperArms.get(robotSide).solve(desiredTransform);

      desiredHandOrientation.getTransform3D(desiredTransform);
      success &= inverseKinematicsForLowerArms.get(robotSide).solve(desiredTransform);

      if (!success)
      {
         System.err.println("Could not find desired joint angles");
         return;
      }

      double[] desiredJointAngles = new double[armJointsClone.get(robotSide).length];
      
      for (int i = 0; i < armJointsClone.get(robotSide).length; i++)
      {
         desiredJointAngles[i] = armJointsClone.get(robotSide)[i].getQ();
      }

      submitSingleHandPose(robotSide, desiredJointAngles);
   }

   private void submitFootPosition(RobotSide robotSide, FramePoint desiredFootPosition)
   {
      FrameOrientation desiredFootOrientation = new FrameOrientation(desiredFootPosition.getReferenceFrame());
      FramePose desiredFootPose = new FramePose(desiredFootPosition, desiredFootOrientation);
      submitFootPose(robotSide, desiredFootPose);
   }

   private void submitFootstepPosition(RobotSide robotSide, FramePose desiredFootstepPosition)
   {

      FramePose footPose = new FramePose(desiredFootstepPosition);

      pipeLine.submitSingleTaskStage(new FootstepTask(fullRobotModel, robotSide, footstepListBehavior, footPose, yoTime));
   }

   private void submitFootPose(RobotSide robotSide, FramePose desiredFootPose)
   {
      //      desiredFootPose.checkReferenceFrameMatch(ankleZUpFrames.get(robotSide));
      desiredFootPose.changeFrame(worldFrame);
      Point3d desiredFootPosition = new Point3d();
      Quat4d desiredFootOrientation = new Quat4d();
      desiredFootPose.getPose(desiredFootPosition, desiredFootOrientation);
      pipeLine.submitSingleTaskStage(new FootPoseTask(robotSide, desiredFootPosition, desiredFootOrientation, yoTime, footPoseBehavior, trajectoryTime
            .getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void doControl()
   {
      handleRequestedSymmetricArmPose();

      handleRequestedArmPose();

      handleRequestedDiagnostic();

      pipeLine.doControl();
   }

   private void handleRequestedDiagnostic()
   {
      if (requestedDiagnostic.getEnumValue() != null)
      {
         switch (requestedDiagnostic.getEnumValue())
         {
            case ARM_MOTIONS:
               sequenceArmPose();
               break;
            case CHEST_ROTATIONS:
               sequenceChestRotations();
               break;
            case PELVIS_ROTATIONS:
               sequencePelvisRotations();
               break;
            case COMBINED_CHEST_PELVIS:
               sequenceMovingChestAndPelvisOnly();
               break;
            case UPPER_BODY:
               sequenceUpperBody();
               break;
            case FOOT_POSES:
               sequenceFootPose();
               break;
            case RUNNING_MAN:
               sequenceRunningMan();
               break;
            case BOW:
               sequenceBow();
               break;
            case KARATE_KID:
               karateKid(activeSideForFootControl.getEnumValue());
               break;
            case STEPS:
               sequenceSteps();
               break;
            case WHOLE_SCHEBANG:
               sequenceSteps();
               sequenceRunningMan();
               karateKid(activeSideForFootControl.getEnumValue());
               sequenceBow();
               break;
            case SQUATS:
               for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
                  sequenceSquats();
               break;
            case SHIFT_WEIGHT:
               sequenceShiftWeight();
               break;
            case SIMPLE_WARMUP:
               sequenceSimpleWarmup();
               break;
            default:
               break;
         }
         requestedDiagnostic.set(null);
      }
   }

   private void handleRequestedArmPose()
   {
      if (requestedSingleArmPose.getEnumValue() != null)
      {
         submitSingleHandPose(activeSideForHandControl.getEnumValue(), requestedSingleArmPose.getEnumValue().getArmJointAngles());
         requestedSingleArmPose.set(null);
      }
   }

   private void handleRequestedSymmetricArmPose()
   {
      if (requestedSymmetricArmPose.getEnumValue() != null)
      {
         submitSymmetricHandPose(requestedSymmetricArmPose.getEnumValue().getArmJointAngles());
         requestedSymmetricArmPose.set(null);
      }
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         handPoseBehaviors.get(robotSide).consumeObjectFromNetworkProcessor(object);
         footstepListBehavior.consumeObjectFromNetworkProcessor(object);
         walkToLocationBehavior.consumeObjectFromNetworkProcessor(object);
      }
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         handPoseBehaviors.get(robotSide).consumeObjectFromController(object);
         footstepListBehavior.consumeObjectFromController(object);
         walkToLocationBehavior.consumeObjectFromController(object);
      }
   }

   @Override
   public void stop()
   {
      pipeLine.clearAll();
   }

   @Override
   public void enableActions()
   {
   }

   @Override
   public void pause()
   {
      isPaused.set(true);
      pelvisPoseBehavior.pause();
      chestOrientationBehavior.pause();
      for (RobotSide robotSide : RobotSide.values)
      {
         handPoseBehaviors.get(robotSide).pause();
      }
   }

   @Override
   public void resume()
   {
      isPaused.set(false);
      pelvisPoseBehavior.resume();
      chestOrientationBehavior.resume();
      for (RobotSide robotSide : RobotSide.values)
      {
         handPoseBehaviors.get(robotSide).resume();
      }
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

   @Override
   public void finalize()
   {
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return false;
   }
}
