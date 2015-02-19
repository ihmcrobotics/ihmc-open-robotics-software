package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HumanoidArmPose;
import us.ihmc.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
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
import us.ihmc.humanoidBehaviors.taskExecutor.FootstepListTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FootstepTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseListTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.PelvisPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WalkToLocationTask;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.LimbName;
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
import us.ihmc.utilities.screwTheory.ScrewTestTools;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJointReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
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
   private static final boolean DEBUG = false;

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

   private  final SideDependentList<OneDoFJoint[]> upperArmJoints = new SideDependentList<OneDoFJoint[]>();
   private  final SideDependentList<OneDoFJoint[]> lowerArmJoints = new SideDependentList<OneDoFJoint[]>();

   private  final SideDependentList<OneDoFJoint[]> upperArmJointsClone = new SideDependentList<OneDoFJoint[]>();
   private  final SideDependentList<OneDoFJoint[]> lowerArmJointsClone = new SideDependentList<OneDoFJoint[]>();
   
   private final SideDependentList<Double> elbowJointSign = new SideDependentList<>();

   private enum DiagnosticTask
   {
      CHEST_ROTATIONS,
      PELVIS_ROTATIONS,
      SHIFT_WEIGHT,
      COMBINED_CHEST_PELVIS,
      ARM_MOTIONS,
      UPPER_BODY,
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
      STEPS_SHORT,
      STEPS_LONG,
      STEPS_IN_PLACE
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
   private final DoubleYoVariable swingTime;
   private final DoubleYoVariable transferTime;

   private final DoubleYoVariable maxFootPoseHeight;
   private final DoubleYoVariable maxFootPoseDisplacement;
   
   private final SideDependentList<ReferenceFrame> upperArmsFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> lowerArmsFrames = new SideDependentList<>();
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

      swingTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "SwingTime", registry);
      swingTime.set(walkingControllerParameters.getDefaultSwingTime());
      transferTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TransferTime", registry);
      transferTime.set(walkingControllerParameters.getDefaultTransferTime());
      
      maxFootPoseHeight = new DoubleYoVariable(behaviorNameFirstLowerCase + "MaxFootPoseHeight", registry);
      maxFootPoseHeight.set(0.1);
      maxFootPoseDisplacement = new DoubleYoVariable(behaviorNameFirstLowerCase + "maxFootPoseDisplacement", registry);
      maxFootPoseDisplacement.set(0.2);
      
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

      activeSideForFootControl = new EnumYoVariable<>("activeSideForFootControl", registry, RobotSide.class, true);
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
      DenseMatrix64F angularSelectionMatrix = new DenseMatrix64F(3, SpatialMotionVector.SIZE);
      angularSelectionMatrix.set(0, 0, 1.0);
      angularSelectionMatrix.set(1, 1, 1.0);
      angularSelectionMatrix.set(2, 2, 1.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         
         // The following one works for Valkyrie but doesn't work for Atlas
//         RigidBody upperArmBody = fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH).getPredecessor();
         // Pretty hackish but will work for now: Consider the elbow joint to be the fourth joint of the chain
         OneDoFJoint[] armJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(chest, hand), OneDoFJoint.class);
         OneDoFJoint elbowJoint = armJoints[3];
         double jointSign = elbowJoint.getJointAxis().getVector().dot(new Vector3d(1.0, 1.0, 1.0));
         elbowJointSign.put(robotSide, jointSign);
         
         RigidBody upperArmBody = elbowJoint.getPredecessor();
         RigidBody lowerArmBody = elbowJoint.getSuccessor();

         upperArmsFrames.put(robotSide, upperArmBody.getBodyFixedFrame());
         lowerArmsFrames.put(robotSide, lowerArmBody.getBodyFixedFrame());

         upperArmJoints.put(robotSide, ScrewTools.filterJoints(ScrewTools.createJointPath(chest, upperArmBody), OneDoFJoint.class));
         upperArmJointsClone.put(robotSide, ScrewTools.filterJoints(ScrewTools.cloneJointPath(upperArmJoints.get(robotSide)), OneDoFJoint.class));
         GeometricJacobian upperArmJacobian = new GeometricJacobian(upperArmJointsClone.get(robotSide), upperArmJointsClone.get(robotSide)[upperArmJointsClone.get(robotSide).length - 1].getSuccessor().getBodyFixedFrame());
         NumericalInverseKinematicsCalculator inverseKinematicsForUpperArm = new NumericalInverseKinematicsCalculator(upperArmJacobian, tolerance, maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);
         inverseKinematicsForUpperArm.setSelectionMatrix(angularSelectionMatrix);
         inverseKinematicsForUpperArms.put(robotSide, inverseKinematicsForUpperArm);

         lowerArmJoints.put(robotSide, ScrewTools.filterJoints(ScrewTools.createJointPath(lowerArmBody, hand), OneDoFJoint.class));
         lowerArmJointsClone.put(robotSide, ScrewTools.filterJoints(ScrewTools.cloneJointPath(lowerArmJoints.get(robotSide)), OneDoFJoint.class));
         GeometricJacobian lowerArmJacobian = new GeometricJacobian(lowerArmJointsClone.get(robotSide), lowerArmJointsClone.get(robotSide)[lowerArmJointsClone.get(robotSide).length - 1].getSuccessor().getBodyFixedFrame());
         NumericalInverseKinematicsCalculator inverseKinematicsForLowerArm = new NumericalInverseKinematicsCalculator(lowerArmJacobian, tolerance, maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);
         inverseKinematicsForLowerArm.setSelectionMatrix(angularSelectionMatrix);
         inverseKinematicsForLowerArms.put(robotSide, inverseKinematicsForLowerArm);
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
   
   private void sequenceMediumWarmup()
   {
      
         sequenceSquats();
         sequenceChestRotations();
         sequencePelvisRotations();
   }
   
   private void sequenceHardWarmup()
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

      FrameOrientation desiredUpperArmOrientation = new FrameOrientation(fullRobotModel.getChest().getBodyFixedFrame());
      
      submitSymmetricHumanoidArmPose(HumanoidArmPose.LARGE_CHICKEN_WINGS);
      sequenceChestRotations();
      sequencePelvisRotations();
      sequenceMovingChestAndPelvisOnly();

      submitSymmetricHumanoidArmPose(HumanoidArmPose.REACH_FAR_FORWARD);
      sequenceChestRotations();
      sequencePelvisRotations();
      sequenceMovingChestAndPelvisOnly();

      submitSymmetricHumanoidArmPose(HumanoidArmPose.REACH_FAR_BACK);
      sequenceChestRotations();
      sequencePelvisRotations();
      sequenceMovingChestAndPelvisOnly();

      desiredUpperArmOrientation.setYawPitchRoll(0.0, 0.0, Math.PI / 2.0);
      submitHandPose(RobotSide.LEFT, desiredUpperArmOrientation, 0.0, null);
      submitHumanoidArmPose(RobotSide.RIGHT, HumanoidArmPose.ARM_STRAIGHT_DOWN);
      sequenceChestRotations();
      sequencePelvisRotations();
      sequenceMovingChestAndPelvisOnly();

      submitHumanoidArmPose(RobotSide.LEFT, HumanoidArmPose.ARM_STRAIGHT_DOWN);
      desiredUpperArmOrientation.setYawPitchRoll(0.0, 0.0, -Math.PI / 2.0);
      submitHandPose(RobotSide.RIGHT, desiredUpperArmOrientation, 0.0, null);
      sequenceChestRotations();
      sequencePelvisRotations();
      sequenceMovingChestAndPelvisOnly();

      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);
   }

   private void sequenceChestRotations()
   {
      double percentOfJointLimit = 0.55;
      double roll = percentOfJointLimit * minMaxRoll;
      submitDesiredChestOrientation(false, 0.0, percentOfJointLimit * minPitch, 0.0);
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredChestOrientation(false, minMaxYaw, percentOfJointLimit * minPitch, 0.0);
         submitDesiredChestOrientation(false, -minMaxYaw, percentOfJointLimit * minPitch, 0.0);
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
      submitDesiredChestOrientation(false, 0.0, percentOfJointLimit * minPitch, roll);
      submitDesiredChestOrientation(false, 0.0, percentOfJointLimit * minPitch, -roll);

      submitDesiredChestOrientation(false, 0.0, 0.0, 0.0);
   }

   private void sequencePelvisRotations()
   {
      double percentOfJointLimit = 0.3;
      double roll = percentOfJointLimit * minMaxRoll;
      double yaw = percentOfJointLimit * minMaxYaw;
      submitDesiredPelvisOrientation(false, 0.0, percentOfJointLimit * minPitch, 0.0);
      if (doPelvisAndChestYaw.getBooleanValue())
      {
         submitDesiredPelvisOrientation(false, yaw, percentOfJointLimit * minPitch, 0.0);
         submitDesiredPelvisOrientation(false, -yaw, percentOfJointLimit * minPitch, 0.0);
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
      submitDesiredPelvisOrientation(false, 0.0, percentOfJointLimit * minPitch, roll);
      submitDesiredPelvisOrientation(false, 0.0, percentOfJointLimit * minPitch, -roll);

      submitPelvisHomeCommand(false);
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
         submitDesiredPelvisPositionOffset(false, pelvisShiftScaleFactor.getX() * desiredPelvisOffset.getX(), pelvisShiftScaleFactor.getY() * desiredPelvisOffset.getY(), 0.0);
      }
      // Get back to the first vertex again
      desiredPelvisOffset.set(supportPolygon.getFrameVertex(0));
      desiredPelvisOffset.sub(center);
      submitDesiredPelvisPositionOffset(false, pelvisShiftScaleFactor.getX() * desiredPelvisOffset.getX(), pelvisShiftScaleFactor.getY() * desiredPelvisOffset.getY(), 0.0);

      submitPelvisHomeCommand(false);
   }

   private void sequenceMovingChestAndPelvisOnly()
   {
      double percentOfJointLimit = 0.8;
      double percentOfJointLimitForPelvis = 0.5;
      submitDesiredChestOrientation(true, 0.0, percentOfJointLimit * minPitch, 0.0);
      submitDesiredPelvisOrientation(true, 0.0, percentOfJointLimitForPelvis * minPitch, 0.0);
      
      submitDesiredChestOrientation(true, 0.0, percentOfJointLimit * maxPitch, 0.0);
      submitDesiredPelvisOrientation(true, 0.0, percentOfJointLimitForPelvis * maxPitch, 0.0);
      
      submitDesiredChestOrientation(true, 0.0, 0.0, percentOfJointLimit * minMaxRoll);
      submitDesiredPelvisOrientation(true, 0.0, 0.0, percentOfJointLimitForPelvis * minMaxRoll);
      
      submitDesiredChestOrientation(true, 0.0, 0.0, -percentOfJointLimit * minMaxRoll);
      submitDesiredPelvisOrientation(true, 0.0, 0.0, -percentOfJointLimitForPelvis * minMaxRoll);
      
      submitDesiredChestOrientation(true, 0.0, percentOfJointLimit * minPitch, 0.0);
      submitDesiredPelvisOrientation(true, 0.0, percentOfJointLimitForPelvis * minPitch, percentOfJointLimitForPelvis * minMaxRoll);
      
      submitDesiredChestOrientation(true, 0.0, percentOfJointLimit * minPitch, 0.0);
      submitDesiredPelvisOrientation(true, 0.0, percentOfJointLimitForPelvis * minPitch, -percentOfJointLimitForPelvis * minMaxRoll);
      
      submitDesiredChestOrientation(true, 0.0, percentOfJointLimit * maxPitch, 0.0);
      submitDesiredPelvisOrientation(true, 0.0, percentOfJointLimitForPelvis * maxPitch, -percentOfJointLimitForPelvis * minMaxRoll);
      
      submitDesiredChestOrientation(true, 0.0, percentOfJointLimit * maxPitch, 0.0);
      submitDesiredPelvisOrientation(true, 0.0, percentOfJointLimitForPelvis * maxPitch, percentOfJointLimitForPelvis * minMaxRoll);
      
      submitChestHomeCommand(true);
      submitPelvisHomeCommand(true);
   }

   private void sequenceArmPose()
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

   private void sequenceFootPoseShort()
   {
      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);
      
      RobotSide robotSide = activeSideForFootControl.getEnumValue();
      if(robotSide == null)
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
      
      submitHandPoseHomeCommand();
   }
   
   private void submitFootPosesShort(RobotSide robotSide)
   {
      
      double outsideFootDisplacement = maxFootPoseDisplacement.getDoubleValue();
      double insideFootDisplacement = 0.4 * maxFootPoseDisplacement.getDoubleValue();
      double footPoseHeight = maxFootPoseHeight.getDoubleValue();
      
      ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide);
      //foot remains flat
      boolean parallelize = false;
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, outsideFootDisplacement, 0.0, footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, -outsideFootDisplacement, 0.0, footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(outsideFootDisplacement), footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, footPoseHeight));
      
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, outsideFootDisplacement, 0.0, footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, outsideFootDisplacement, robotSide.negateIfRightSide(-insideFootDisplacement), footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(-insideFootDisplacement), footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), footPoseHeight));
      
      //footOrientation changes
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, footPoseHeight));
      submitFootPose(parallelize, robotSide, ankleZUpFrame, 0.6, robotSide.negateIfRightSide(0.02), 0.15, 0.0, -0.9, 0.0);
      submitFootPose(parallelize, robotSide, ankleZUpFrame, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0);
      submitFootPose(parallelize, robotSide, ankleZUpFrame, -0.25, robotSide.negateIfRightSide(0.01), 0.15, 0.0, 1.2, 0.0);
      submitFootPose(parallelize, robotSide, ankleZUpFrame, -0.5, robotSide.negateIfRightSide(0.02), 0.30, 0.0, 2.4, 0.0);
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, footPoseHeight));
      submitFootPose(parallelize, robotSide, ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.3), 0.20, 0.0, 0.0, robotSide.negateIfRightSide(0.5));
      
      
      //put the foot back on the ground
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, footPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, -0.1));
   }
   
   private void sequenceFootPoseLong()
   {
      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);
      
      RobotSide robotSide = activeSideForFootControl.getEnumValue();
      if(robotSide == null)
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
      
      submitHandPoseHomeCommand();
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
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, outsideFootDisplacement, 0.0, higherFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, outsideFootDisplacement, 0.0, midFootPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, outsideFootDisplacement, 0.0, higherFootPoseHeight));
      
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, -outsideFootDisplacement, 0.0, higherFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, -outsideFootDisplacement, 0.0, midFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, -outsideFootDisplacement, 0.0, higherFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));    
      
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(outsideFootDisplacement), higherFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(outsideFootDisplacement), midFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(outsideFootDisplacement), higherFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));    
      
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), higherFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), midFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, outsideFootDisplacement, robotSide.negateIfRightSide(-insideFootDisplacement), higherFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), midFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(-insideFootDisplacement), higherFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(outsideFootDisplacement), midFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, -outsideFootDisplacement, robotSide.negateIfRightSide(-insideFootDisplacement), midFootPoseHeight));    
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));
      ////////////////////////////////////////////////////////
      
      //footOrientation changes
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));
      submitFootPose(parallelize, robotSide, ankleZUpFrame, 0.6, robotSide.negateIfRightSide(0.02), 0.15, 0.0, -0.9, 0.0);
      submitFootPose(parallelize, robotSide, ankleZUpFrame, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0);
      submitFootPose(parallelize, robotSide, ankleZUpFrame, -0.25, robotSide.negateIfRightSide(0.01), 0.15, 0.0, 1.2, 0.0);
      submitFootPose(parallelize, robotSide, ankleZUpFrame, -0.5, robotSide.negateIfRightSide(0.02), 0.30, 0.0, 2.4, 0.0);
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));
      submitFootPose(parallelize, robotSide, ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.3), 0.20, 0.0, 0.0, robotSide.negateIfRightSide(0.5));    
      
      //put the foot back on the ground
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, higherFootPoseHeight));
      submitFootPosition(parallelize, robotSide, new FramePoint(ankleZUpFrame, 0.0, 0.0, -0.1));
      
   }

   private void sequenceRunningMan()
   {
      for (RobotSide robotSide : RobotSide.values)
         runningMan(robotSide);
   }

   private void runningMan(RobotSide robotSide)
   {
      ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide.getOppositeSide());
      boolean mirrorOrientationsForRightSide = true;

      // First Lift up the foot
      submitFootPosition(false, robotSide, new FramePoint(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.25), 0.1));

      // Go to running man pose:
      FrameOrientation desiredUpperArmOrientation = new FrameOrientation(fullRobotModel.getChest().getBodyFixedFrame());
      desiredUpperArmOrientation.setYawPitchRoll(0.0, -Math.PI / 2.0, 0.3);
      submitHandPose(robotSide, desiredUpperArmOrientation, -Math.PI / 2.0, null, mirrorOrientationsForRightSide);

      desiredUpperArmOrientation.setYawPitchRoll(0.0, Math.PI / 2.0, 0.3);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -Math.PI / 2.0, null, mirrorOrientationsForRightSide);
      
      FramePose footPose = new FramePose(ankleZUpFrame);
      footPose.setPosition(-0.40, robotSide.negateIfRightSide(0.25), 0.40);
      footPose.setOrientation(0.0, 0.8 * Math.PI / 2.0, 0.0);
      submitFootPose(true, robotSide, footPose);

      submitDesiredChestOrientation(true, 0.0, Math.toRadians(20.0), 0.0);
      submitDesiredPelvisOrientation(true, 0.0, Math.toRadians(10.0), 0.0);

      pipeLine.submitSingleTaskStage(new NullTask());

      // Do a "Y" stance with the foot outside
      desiredUpperArmOrientation.setYawPitchRoll(0.0, -0.0, 2.3561);
      submitSymmetricHandPose(desiredUpperArmOrientation, 0.0, null);

      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.65), 0.1);
      footPose.setOrientation(0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(40.0)));
      submitFootPose(true, robotSide, footPose);
      submitChestHomeCommand(true);
      submitDesiredPelvisOrientation(true, 0.0, 0.0, Math.toRadians(25.0));


      pipeLine.submitSingleTaskStage(new NullTask());

      // Go back to stand prep but don't put the foot on the ground yet
      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);

      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.25), 0.1);
      footPose.setOrientation(0.0, 0.0, 0.0);
      submitFootPose(true, robotSide, footPose);

      submitDesiredPelvisOrientation(true, 0.0, 0.0, 0.0);

      // Put the foot back on the ground
      submitFootPosition(false, robotSide, new FramePoint(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.25), -0.3));
   }

   private void sequenceBow()
   {
      bow(RobotSide.LEFT);
   }

   private void bow(RobotSide robotSide)
   {
      ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide.getOppositeSide());
      FrameOrientation desiredUpperArmOrientation = new FrameOrientation(fullRobotModel.getChest().getBodyFixedFrame());
      boolean mirrorOrientationForRightSide = true;
      
      //put the foot forward and prepare the arms
      FramePose desiredFootstepPosition = new FramePose(ankleZUpFrame);
      Point3d position = new Point3d(0.2, robotSide.negateIfRightSide(0.25), 0.0);
      Quat4d orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      desiredFootstepPosition.setPose(position, orientation);
      desiredFootstepPosition.changeFrame(worldFrame);
      submitFootstepPose(false, robotSide, desiredFootstepPosition);

      desiredUpperArmOrientation.setYawPitchRoll(0.0, 0.5235, 0.3);
      submitHandPose(robotSide, desiredUpperArmOrientation, -0.1, null, mirrorOrientationForRightSide);
      
      desiredUpperArmOrientation.setYawPitchRoll(0.0, -0.5235, 0.3);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -0.1, null, mirrorOrientationForRightSide);
      
      pipeLine.requestNewStage();

      desiredUpperArmOrientation.setYawPitchRoll(-1.7242, 0.2588, -0.5436);
      submitHandPose(robotSide, desiredUpperArmOrientation, -0.1, null, mirrorOrientationForRightSide);
      
      desiredUpperArmOrientation.setYawPitchRoll(-1.4173, 0.2588, 0.5436);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -0.1, null, mirrorOrientationForRightSide);
      
      pipeLine.requestNewStage();

      //bend forward and arms
      desiredUpperArmOrientation.setYawPitchRoll(-1.7242, 0.2588, -0.5436);
      submitHandPose(robotSide, desiredUpperArmOrientation, -Math.PI / 2.0, null, mirrorOrientationForRightSide);
      
      desiredUpperArmOrientation.setYawPitchRoll(-1.4173, 0.2588, 0.5436);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -Math.PI / 2.0, null, mirrorOrientationForRightSide);

      submitDesiredChestOrientation(true, 0.0, Math.toRadians(35.0), 0.0);
      submitDesiredPelvisOrientation(true, 0.0, Math.toRadians(30), 0.0);

      pipeLine.requestNewStage();

      //back to normal stance
      submitChestHomeCommand(true);
      submitPelvisHomeCommand(true);

      pipeLine.requestNewStage();

      desiredUpperArmOrientation.setYawPitchRoll(-1.7242, 0.2588, -0.5436);
      submitHandPose(robotSide, desiredUpperArmOrientation, -0.1, null, mirrorOrientationForRightSide);
      
      desiredUpperArmOrientation.setYawPitchRoll(-1.4173, 0.2588, 0.5436);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -0.1, null, mirrorOrientationForRightSide);

      desiredFootstepPosition = new FramePose(ankleZUpFrame);
      position = new Point3d(0., robotSide.negateIfRightSide(0.25), 0.0);
      orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      desiredFootstepPosition.setPose(position, orientation);
      desiredFootstepPosition.changeFrame(worldFrame);
      submitFootstepPose(false, robotSide, desiredFootstepPosition);

      pipeLine.requestNewStage();

      // Put the foot back on the ground
      // Put the foot back on the ground
      //arms at good spot
      desiredUpperArmOrientation.setYawPitchRoll(0.0, 0.0, 0.3);
      submitHandPose(robotSide, desiredUpperArmOrientation, -0.2, null, mirrorOrientationForRightSide);
      
      desiredUpperArmOrientation.setYawPitchRoll(0.0, 0.0, 0.3);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -0.2, null, mirrorOrientationForRightSide);

      pipeLine.requestNewStage();

      desiredUpperArmOrientation.setYawPitchRoll(0.0, 0.0, 0.3);
      submitHandPose(robotSide, desiredUpperArmOrientation, -Math.PI / 2.0, null, mirrorOrientationForRightSide);
      
      desiredUpperArmOrientation.setYawPitchRoll(0.0, 0.0, 0.3);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -Math.PI / 2.0 , null, mirrorOrientationForRightSide);

      pipeLine.requestNewStage();
   }

   private void sequenceStepsLong()
   {
      pipeLine.requestNewStage();

      double distanceToWalk = 0.5;
      
      /////////// normal stance ///////////
      // forward
      submitWalkToLocation(false, distanceToWalk, 0.0, 0.0,0.0);
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
      submitWalkToLocation(false, distanceToWalk, 0.0, 0.0,0.0);
      //backward
      submitWalkToLocation(false, 0.0, 0.0, 0.0, Math.PI); // Math.PI
      //sideWalk
      submitWalkToLocation(false, distanceToWalk, 0.0, -Math.PI / 2.0, -Math.PI / 2.0); 
      submitWalkToLocation(false, 0.0, 0.0, 0.0, Math.PI / 2.0);
      //turn in place
      submitWalkToLocation(false, 0.0, 0.0, 0.0, 0.0);
      
      submitHandPoseHomeCommand();
      
      /////////// chest bending backward///////////
      submitDesiredChestOrientation(false, 0.0, Math.toRadians(-10.0), 0.0);
      
      // forward
      submitWalkToLocation(false, distanceToWalk, 0.0, 0.0,0.0);
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

   private void submitWalkToLocation(boolean parallelize, double x, double y, double robotYaw,double angleRelativeToPath)
   {
      FramePose2d targetPoseInWorld = new FramePose2d();
      targetPoseInWorld.setPoseIncludingFrame(midFeetZUpFrame, x, y, robotYaw);
      targetPoseInWorld.changeFrame(worldFrame);
      
      WalkToLocationTask walkToLocationTask = new WalkToLocationTask(targetPoseInWorld, walkToLocationBehavior, angleRelativeToPath, footstepLength.getDoubleValue(), yoTime);
      if (parallelize)
         pipeLine.submitTaskForPallelPipesStage(walkToLocationBehavior, walkToLocationTask);
      else
         pipeLine.submitSingleTaskStage(walkToLocationTask);
   }

   private void sequenceStepsInPlace()
   {
      FootstepDataList footstepDataList = new FootstepDataList(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      FramePose footstepPose = new FramePose();
      
      for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
      {
         for (RobotSide robotSide : RobotSide.values())
         {
            footstepPose.setToZero(fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG));
            footstepPose.changeFrame(worldFrame);

            Point3d footLocation = new Point3d();
            Quat4d footOrientation = new Quat4d();

            footstepPose.getPosition(footLocation);
            footstepPose.getOrientation(footOrientation);

            FootstepData footstepData = new FootstepData(robotSide, footLocation, footOrientation);

            footstepDataList.add(footstepData);
         }
      }
      pipeLine.submitSingleTaskStage(new FootstepListTask(footstepListBehavior, footstepDataList, yoTime));
   }

   private void karateKid(RobotSide robotSide)
   {
      ReferenceFrame ankleZUpFrame = ankleZUpFrames.get(robotSide.getOppositeSide());
      // First Lift up the foot
      submitFootPosition(false, robotSide, new FramePoint(ankleZUpFrame, 0.1, robotSide.negateIfRightSide(0.25), 0.2));

      // Put the arm down
      double halfPi = Math.PI / 2.0;
      ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      FrameOrientation upperArmDown2                   = new FrameOrientation(chestFrame, -0.7443, 0.2789, 0.2905 );
      FrameOrientation upperArmIntermediateOnWayUp     = new FrameOrientation(chestFrame, 0.0, 0.7853, halfPi );
      FrameOrientation upperArmUp1                     = new FrameOrientation(chestFrame, 0.6154, 0.5235, 2.5261 );
      FrameOrientation upperArmUp2                     = new FrameOrientation(chestFrame, -0.6154, -0.5235, 2.5261 );
      FrameOrientation upperArmIntermediateOnWayDown   = new FrameOrientation(chestFrame, 0.0, -0.7853, halfPi );
      FrameOrientation upperArmDown1                   = new FrameOrientation(chestFrame, 0.7443, -0.2789, 0.2905 );

      SideDependentList<double[]> armsDown2 = computeSymmetricArmJointAngles(upperArmDown2, 0.0, null, true);
      SideDependentList<double[]> armsIntermediateOnWayUp = computeSymmetricArmJointAngles(upperArmIntermediateOnWayUp, -halfPi / 2.0, null, true);
      SideDependentList<double[]> armsUp1 = computeSymmetricArmJointAngles(upperArmUp1, 0.0, null, true);
      SideDependentList<double[]> armsUp2 = computeSymmetricArmJointAngles(upperArmUp2, 0.0, null, true);
      SideDependentList<double[]> armsIntermediateOnWayDown = computeSymmetricArmJointAngles(upperArmIntermediateOnWayDown, -halfPi / 2.0, null, true);
      SideDependentList<double[]> armsDown1 = computeSymmetricArmJointAngles(upperArmDown1, 0.0, null, true);

      int numberOfHandPoses = 10;
      SideDependentList<double[][]> armFlyingSequence = new SideDependentList<>(new double[numberOfArmJoints][numberOfHandPoses], new double[numberOfArmJoints][numberOfHandPoses]);

      for (int jointIndex = 0; jointIndex < numberOfArmJoints; jointIndex++)
      {
         for (int poseIndex = 0; poseIndex < numberOfHandPoses; poseIndex++)
         {
            for (RobotSide side : RobotSide.values)
            {
               double desiredJointAngle;
               switch (poseIndex % 6)
               {
                  case 0:
                     desiredJointAngle = armsDown1.get(side)[jointIndex];
                     break;
                  case 1:
                     desiredJointAngle = armsDown2.get(side)[jointIndex];
                     break;
                  case 2:
                     desiredJointAngle = armsIntermediateOnWayUp.get(side)[jointIndex];
                     break;
                  case 3:
                     desiredJointAngle = armsUp1.get(side)[jointIndex];
                     break;
                  case 4:
                     desiredJointAngle = armsUp2.get(side)[jointIndex];
                     break;
                  case 5:
                     desiredJointAngle = armsIntermediateOnWayDown.get(side)[jointIndex];
                     break;
                  default:
                     throw new RuntimeException("Should not get there!");
               }
               armFlyingSequence.get(side)[jointIndex][poseIndex] = desiredJointAngle;
            }
         }
      }

      for (RobotSide tempSide : RobotSide.values)
      {
         HandPoseListPacket handPoseListPacket = new HandPoseListPacket(tempSide, armFlyingSequence.get(tempSide), flyingTrajectoryTime.getDoubleValue() * numberOfHandPoses);
         pipeLine.submitTaskForPallelPipesStage(handPoseListBehaviors.get(tempSide),
               new HandPoseListTask(handPoseListPacket, handPoseListBehaviors.get(tempSide), yoTime, sleepTimeBetweenPoses.getDoubleValue()));
      }

      // Put the arms in front
      FrameOrientation desiredUpperArmOrientation = new FrameOrientation(chestFrame);
      desiredUpperArmOrientation.setYawPitchRoll(0.0, -0.0, 0.6);
      submitSymmetricHandPose(desiredUpperArmOrientation, -halfPi, null);
      
      pipeLine.requestNewStage();

      desiredUpperArmOrientation.setYawPitchRoll(-0.7800, 0.1585, 0.7473);
      submitSymmetricHandPose(desiredUpperArmOrientation, -halfPi, null);

      // Supa powerful front kick!!!!!
      FramePose footPose = new FramePose();
      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.75, robotSide.negateIfRightSide(0.25), 0.25);
      footPose.setOrientation(0.0, -halfPi / 2.0, 0.0);
      submitFootPose(true, robotSide, footPose);

      submitDesiredChestOrientation(true, 0.0, Math.toRadians(-5.0), 0.0);
      submitDesiredPelvisOrientation(true, 0.0, Math.toRadians(-15.0), 0.0);

      pipeLine.requestNewStage();

      // Supa powerful back kick!!!!!
      desiredUpperArmOrientation.setYawPitchRoll(-0.7566, -0.9980, 0.9947);
      submitHandPose(robotSide, desiredUpperArmOrientation, -0.3, null, true);
      
      desiredUpperArmOrientation.setYawPitchRoll(0.0, 1.2566, 0.3);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -0.3, null, true);
      
      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(-0.75, robotSide.negateIfRightSide(0.25), 0.35);
      footPose.setOrientation(0.0, 0.8 * halfPi, 0.0);
      submitFootPose(true, robotSide, footPose);

      submitDesiredChestOrientation(true, 0.0, Math.toRadians(30.0), 0.0);
      submitDesiredPelvisOrientation(true, 0.0, Math.toRadians(20.0), 0.0);

      pipeLine.requestNewStage();

      // Supa powerful side kick!!!!!
      desiredUpperArmOrientation.setYawPitchRoll(0.0, 0.4, halfPi);
      submitHandPose(robotSide, desiredUpperArmOrientation, -0.1, null, true);
      
      desiredUpperArmOrientation.setYawPitchRoll(-1.2707, 0.0, halfPi);
      submitHandPose(robotSide.getOppositeSide(), desiredUpperArmOrientation, -halfPi, null, true);
      
      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.85), 0.3);
      footPose.setOrientation(0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(40.0)));
      submitFootPose(true, robotSide, footPose);

      submitDesiredChestOrientation(true, 0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(30.0)));
      submitDesiredPelvisOrientation(true, 0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(20.0)));

      pipeLine.requestNewStage();

      // Go back to stand prep but don't put the foot on the ground yet
      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);

      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.25), 0.1);
      footPose.setOrientation(0.0, 0.0, 0.0);
      submitFootPose(true, robotSide, footPose);

      submitChestHomeCommand(true);
      submitPelvisHomeCommand(true);

      // Put the foot back on the ground
      submitFootPosition(false, robotSide, new FramePoint(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.25), -0.3));
   }

   private void sequenceSquats()
   {
      submitDesiredCoMHeightOffset(false, minCoMHeightOffset.getDoubleValue());
      submitDesiredCoMHeightOffset(false, maxCoMHeightOffset.getDoubleValue());
   }

   private void sequenceSquatathon()
   {
      boolean parallelize = true;
      submitDesiredPelvisOrientation(parallelize, 0.0, Math.toRadians(15), 0.0);
      submitDesiredChestOrientation(parallelize, 0.0, Math.toRadians(15.0), 0.0);

      submitSymmetricHumanoidArmPose(HumanoidArmPose.REACH_WAY_FORWARD);
      submitDesiredCoMHeightOffset(parallelize, minCoMHeightOffset.getDoubleValue());

      pipeLine.requestNewStage();

      submitChestHomeCommand(parallelize);
      submitPelvisHomeCommand(parallelize);

      submitSymmetricHumanoidArmPose(HumanoidArmPose.STAND_PREP);

      submitDesiredCoMHeightOffset(parallelize, maxCoMHeightOffset.getDoubleValue());
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
      ChestOrientationPacket homeChestPacket = PacketControllerTools.createGoToHomeChestOrientationPacket(trajectoryTime.getDoubleValue());
      ChestOrientationTask chestOrientationTask = new ChestOrientationTask(homeChestPacket, yoTime, chestOrientationBehavior, sleepTimeBetweenPoses.getDoubleValue());
      if (parallelize)
         pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior, chestOrientationTask);
      else
         pipeLine.submitSingleTaskStage(chestOrientationTask);
   }

   private void submitDesiredChestOrientation(boolean parallelize, double yaw, double pitch, double roll)
   {
      FrameOrientation desiredChestOrientation = new FrameOrientation(pelvisZUpFrame, yaw, pitch, roll);
      desiredChestOrientation.changeFrame(worldFrame);
      ChestOrientationTask chestOrientationTask = new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue());
      if (parallelize)
         pipeLine.submitTaskForPallelPipesStage(chestOrientationBehavior, chestOrientationTask);
      else
         pipeLine.submitSingleTaskStage(chestOrientationTask);
   }
   
   private void submitDesiredCoMHeightOffset(boolean parallelize, double offsetHeight)
   {
      CoMHeightTask comHeightTask = new CoMHeightTask(offsetHeight, yoTime, comHeightBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue());
      if (parallelize)
         pipeLine.submitTaskForPallelPipesStage(comHeightBehavior, comHeightTask);
      else
         pipeLine.submitSingleTaskStage(comHeightTask);
   }

   private void submitPelvisHomeCommand(boolean parallelize)
   {
      PelvisPosePacket homePelvisPacket = PacketControllerTools.createGoToHomePelvisPosePacket(trajectoryTime.getDoubleValue());
      PelvisPoseTask pelvisPoseTask = new PelvisPoseTask(homePelvisPacket, yoTime, pelvisPoseBehavior, sleepTimeBetweenPoses.getDoubleValue());
      if (parallelize)
         pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior, pelvisPoseTask);
      else
         pipeLine.submitSingleTaskStage(pelvisPoseTask);
   }

   private void submitDesiredPelvisOrientation(boolean parallelize, double yaw, double pitch, double roll)
   {
      FrameOrientation desiredPelvisOrientation = new FrameOrientation(findFixedFrameForPelvisOrientation(), yaw, pitch, roll);
      desiredPelvisOrientation.changeFrame(worldFrame);
      PelvisPoseTask pelvisPoseTask = new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue());
      if (parallelize)
         pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior, pelvisPoseTask);
      else
         pipeLine.submitSingleTaskStage(pelvisPoseTask);
   }

   private void submitDesiredPelvisPositionOffset(boolean parallelize, double dx, double dy, double dz)
   {
      SixDoFJointReferenceFrame frameAfterRootJoint = fullRobotModel.getRootJoint().getFrameAfterJoint();
      FramePoint desiredPelvisPosition = new FramePoint(frameAfterRootJoint, dx, dy, dz);
      desiredPelvisPosition.changeFrame(worldFrame);
      PelvisPoseTask pelvisPoseTask = new PelvisPoseTask(desiredPelvisPosition, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue());
      if (parallelize)
         pipeLine.submitTaskForPallelPipesStage(pelvisPoseBehavior, pelvisPoseTask);
      else
         pipeLine.submitSingleTaskStage(pelvisPoseTask);
   }

   public void submitSymmetricHumanoidArmPose(HumanoidArmPose armPose)
   {
      for (RobotSide robotSide : RobotSide.values)
         submitHumanoidArmPose(robotSide, armPose);
   }

   public void submitHumanoidArmPose(RobotSide robotSide, HumanoidArmPose armPose)
   {
      double[] yawPitchRollForUpperArm = armPose.getDesiredUpperArmYawPitchRoll();
      yawPitchRollForUpperArm[0] = robotSide.negateIfRightSide(yawPitchRollForUpperArm[0]);
      yawPitchRollForUpperArm[2] = robotSide.negateIfRightSide(yawPitchRollForUpperArm[2]);
      
      FrameOrientation desiredUpperArmOrientation = new FrameOrientation(fullRobotModel.getChest().getBodyFixedFrame());
      desiredUpperArmOrientation.setYawPitchRoll(yawPitchRollForUpperArm);
      double elbowAngle = armPose.getDesiredElbowAngle();
      FrameOrientation desiredHandOrientation = new FrameOrientation(lowerArmsFrames.get(robotSide));
      submitHandPose(robotSide, desiredUpperArmOrientation, elbowAngle, desiredHandOrientation);
   }

   public void submitSymmetricHandPose(FrameOrientation desiredUpperArmOrientation, double elbowAngle, FrameOrientation desiredHandOrientation)
   {
      desiredUpperArmOrientation.checkReferenceFrameMatch(fullRobotModel.getChest().getBodyFixedFrame());
      if (desiredHandOrientation != null)
         desiredHandOrientation.checkReferenceFrameMatch(lowerArmsFrames.get(RobotSide.LEFT));
      
      FrameOrientation tempUpperArmFrameOrientation = new FrameOrientation();
      FrameOrientation tempHandFrameOrientation = desiredHandOrientation == null ? null : new FrameOrientation();

      for (RobotSide robotSide : RobotSide.values)
      {
         double[] yawPitchRollForUpperArm = desiredUpperArmOrientation.getYawPitchRoll();
         yawPitchRollForUpperArm[0] = robotSide.negateIfRightSide(yawPitchRollForUpperArm[0]);
         yawPitchRollForUpperArm[2] = robotSide.negateIfRightSide(yawPitchRollForUpperArm[2]);
      
         tempUpperArmFrameOrientation.setToZero(fullRobotModel.getChest().getBodyFixedFrame());
         tempUpperArmFrameOrientation.setYawPitchRoll(yawPitchRollForUpperArm);
         
         if (desiredHandOrientation != null)
         {
            double[] yawPitchRollForHand = desiredHandOrientation.getYawPitchRoll();
            yawPitchRollForHand[0] = robotSide.negateIfRightSide(yawPitchRollForHand[0]);
            yawPitchRollForHand[2] = robotSide.negateIfRightSide(yawPitchRollForHand[2]);

            tempHandFrameOrientation.setToZero(lowerArmsFrames.get(robotSide));
            tempHandFrameOrientation.setYawPitchRoll(yawPitchRollForHand);
         }
         submitHandPose(robotSide, tempUpperArmFrameOrientation, elbowAngle, tempHandFrameOrientation);
      }
   }

   private void submitHandPoseHomeCommand()
   {
      for(RobotSide robotSide : RobotSide.values())
      {
         HandPosePacket handPosePacket = PacketControllerTools.createGoToHomeHandPosePacket(robotSide, trajectoryTime.getDoubleValue());
         HandPoseBehavior handPoseBehavior = handPoseBehaviors.get(robotSide);
         pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(robotSide, handPosePacket, handPoseBehavior, yoTime));
      }
   }
   
   public void submitHandPose(RobotSide robotSide, FrameOrientation desiredUpperArmOrientation, double elbowAngle, FrameOrientation desiredHandOrientation, boolean mirrorOrientationForRightSide)
   {
      if (mirrorOrientationForRightSide && robotSide == RobotSide.RIGHT)
      {
         FrameOrientation mirroredUpperArmOrientation = new FrameOrientation(desiredUpperArmOrientation);
         double[] mirroredYawPitchRollForUpperArm = mirroredUpperArmOrientation.getYawPitchRoll();
         mirroredYawPitchRollForUpperArm[0] = - mirroredYawPitchRollForUpperArm[0];
         mirroredYawPitchRollForUpperArm[2] = - mirroredYawPitchRollForUpperArm[2];
         mirroredUpperArmOrientation.setYawPitchRoll(mirroredYawPitchRollForUpperArm);
         desiredUpperArmOrientation = mirroredUpperArmOrientation;

         if (desiredHandOrientation != null)
         {
            FrameOrientation mirroredHandOrientation = new FrameOrientation(desiredHandOrientation);
            double[] mirroredYawPitchRollForHand = mirroredHandOrientation.getYawPitchRoll();
            mirroredYawPitchRollForHand[0] = - mirroredYawPitchRollForHand[0];
            mirroredYawPitchRollForHand[2] = - mirroredYawPitchRollForHand[2];
            mirroredHandOrientation.setYawPitchRoll(mirroredYawPitchRollForHand);
            desiredHandOrientation = mirroredHandOrientation;
         }
      }

      submitHandPose(robotSide, desiredUpperArmOrientation, elbowAngle, desiredHandOrientation);
   }

   public void submitHandPose(RobotSide robotSide, FrameOrientation desiredUpperArmOrientation, double elbowAngle, FrameOrientation desiredHandOrientation)
   {
      double[] desiredJointAngles = computeArmJointAngles(robotSide, desiredUpperArmOrientation, elbowAngle, desiredHandOrientation);

      HandPoseBehavior handPoseBehavior = handPoseBehaviors.get(robotSide);
      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, new HandPoseTask(robotSide, desiredJointAngles, yoTime, handPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
   }

   public SideDependentList<double[]> computeSymmetricArmJointAngles(FrameOrientation desiredUpperArmOrientation, double elbowAngle, FrameOrientation desiredHandOrientation, boolean mirrorOrientationForRightSide)
   {
      SideDependentList<double[]> desiredSymmetricJointAngles = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         if (mirrorOrientationForRightSide && robotSide == RobotSide.RIGHT)
         {
            FrameOrientation mirroredUpperArmOrientation = new FrameOrientation(desiredUpperArmOrientation);
            double[] mirroredYawPitchRollForUpperArm = mirroredUpperArmOrientation.getYawPitchRoll();
            mirroredYawPitchRollForUpperArm[0] = -mirroredYawPitchRollForUpperArm[0];
            mirroredYawPitchRollForUpperArm[2] = -mirroredYawPitchRollForUpperArm[2];
            mirroredUpperArmOrientation.setYawPitchRoll(mirroredYawPitchRollForUpperArm);
            desiredUpperArmOrientation = mirroredUpperArmOrientation;

            if (desiredHandOrientation != null)
            {
               FrameOrientation mirroredHandOrientation = new FrameOrientation(desiredHandOrientation);
               double[] mirroredYawPitchRollForHand = mirroredHandOrientation.getYawPitchRoll();
               mirroredYawPitchRollForHand[0] = -mirroredYawPitchRollForHand[0];
               mirroredYawPitchRollForHand[2] = -mirroredYawPitchRollForHand[2];
               mirroredHandOrientation.setYawPitchRoll(mirroredYawPitchRollForHand);
               desiredHandOrientation = mirroredHandOrientation;
            }
         }
         desiredSymmetricJointAngles.put(robotSide, computeArmJointAngles(robotSide, desiredUpperArmOrientation, elbowAngle, desiredHandOrientation));
      }
      return desiredSymmetricJointAngles;
   }

   private double[] computeArmJointAngles(RobotSide robotSide, FrameOrientation desiredUpperArmOrientation, double elbowAngle, FrameOrientation desiredHandOrientation)
   {
      double[] desiredUpperArmJointAngles = computeUpperArmJointAngles(robotSide, desiredUpperArmOrientation);
      
      if (desiredUpperArmJointAngles == null) return null;

      double[] desiredLowerArmJointAngles = computeLowerArmJointAngles(robotSide, desiredHandOrientation);

      if (desiredLowerArmJointAngles == null) return null;

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

   private int counterForUpperArmIK = 0;
   private final Random random = new Random(541654L);

   private double[] computeUpperArmJointAngles(RobotSide robotSide, FrameOrientation desiredUpperArmOrientation)
   {
      if (counterForUpperArmIK >= 10)
      {
         counterForUpperArmIK = 0;
         for (OneDoFJoint joint : upperArmJointsClone.get(robotSide))
            joint.setQ(0.0); // For safety
         System.err.println("Could not find desired joint angles for the upper arm joints");
         if (DEBUG)
            Thread.dumpStack();
         return null;
      }
      desiredUpperArmOrientation.checkReferenceFrameMatch(fullRobotModel.getChest().getBodyFixedFrame());

      RigidBodyTransform desiredTransformForUpperArm = new RigidBodyTransform();
      desiredUpperArmOrientation.getTransform3D(desiredTransformForUpperArm);
      boolean success = inverseKinematicsForUpperArms.get(robotSide).solve(desiredTransformForUpperArm);

      if (!success)
      {
         counterForUpperArmIK++;
         ScrewTestTools.setRandomPositions(upperArmJointsClone.get(robotSide), random);
         computeUpperArmJointAngles(robotSide, desiredUpperArmOrientation);
      }

      double[] desiredUpperArmJointAngles = new double[upperArmJointsClone.get(robotSide).length];

      for (int i = 0; i < upperArmJointsClone.get(robotSide).length; i++)
      {
         desiredUpperArmJointAngles[i] = upperArmJointsClone.get(robotSide)[i].getQ();
      }

      return desiredUpperArmJointAngles;
   }

   private double[] computeLowerArmJointAngles(RobotSide robotSide, FrameOrientation desiredHandOrientation)
   {
      if (desiredHandOrientation == null)
         return new double[lowerArmJointsClone.get(robotSide).length];

      desiredHandOrientation.checkReferenceFrameMatch(lowerArmsFrames.get(robotSide));

      RigidBodyTransform desiredTransformForLowerArm = new RigidBodyTransform();
      desiredHandOrientation.getTransform3D(desiredTransformForLowerArm);
      boolean success = inverseKinematicsForLowerArms.get(robotSide).solve(desiredTransformForLowerArm);

      if (!success)
      {
         System.err.println("Could not find desired joint angles for the lower arm joints");
         if (DEBUG)
            Thread.dumpStack();
         return null;
      }

      double[] desiredLowerArmJointAngles = new double[lowerArmJointsClone.get(robotSide).length];

      for (int i = 0; i < lowerArmJointsClone.get(robotSide).length; i++)
      {
         desiredLowerArmJointAngles[i] = lowerArmJointsClone.get(robotSide)[i].getQ();
      }

      return desiredLowerArmJointAngles;
   }

   private void submitFootstepPose(boolean parallelize, RobotSide robotSide, FramePose desiredFootstepPose)
   {
      FramePose footPose = new FramePose(desiredFootstepPose);

      FootstepTask footstepTask = new FootstepTask(fullRobotModel, robotSide, footstepListBehavior, footPose, yoTime);
      if (parallelize)
         pipeLine.submitTaskForPallelPipesStage(footstepListBehavior, footstepTask);
      else
         pipeLine.submitSingleTaskStage(footstepTask);
   }

   private void submitFootPosition(boolean parallelize, RobotSide robotSide, FramePoint desiredFootPosition)
   {
      FrameOrientation desiredFootOrientation = new FrameOrientation(desiredFootPosition.getReferenceFrame());
      FramePose desiredFootPose = new FramePose(desiredFootPosition, desiredFootOrientation);
      submitFootPose(parallelize, robotSide, desiredFootPose);
   }

   private void submitFootPose(boolean parallelize, RobotSide robotSide, FramePose desiredFootPose)
   {
      desiredFootPose.changeFrame(worldFrame);
      Point3d desiredFootPosition = new Point3d();
      Quat4d desiredFootOrientation = new Quat4d();
      desiredFootPose.getPose(desiredFootPosition, desiredFootOrientation);
      FootPoseTask footPoseTask = new FootPoseTask(robotSide, desiredFootPosition, desiredFootOrientation, yoTime, footPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue());
      
      if (parallelize)
         pipeLine.submitTaskForPallelPipesStage(footPoseBehavior, footPoseTask);
      else
         pipeLine.submitSingleTaskStage(footPoseTask);
   }

   private void submitFootPose(boolean parallelize, RobotSide robotSide, ReferenceFrame referenceFrame, double x, double y, double z, double yaw, double pitch, double roll)
   {
      FramePoint framePosition = new FramePoint(referenceFrame, x, y, z);
      FrameOrientation frameOrientation = new FrameOrientation(referenceFrame, yaw, pitch, roll);
      FramePose desiredFootPose = new FramePose(framePosition, frameOrientation);
      submitFootPose(parallelize, robotSide, desiredFootPose);
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
            case FOOT_POSES_SHORT:
               sequenceFootPoseShort();
               break;
            case FOOT_POSES_LONG:
               sequenceFootPoseLong();
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
            case STEPS_SHORT:
               sequenceStepsShort();
               break;
            case STEPS_LONG:
               sequenceStepsLong();
               break;
            case WHOLE_SCHEBANG:
               sequenceStepsLong();
               sequenceRunningMan();
               karateKid(activeSideForFootControl.getEnumValue());
               sequenceBow();
               break;
            case SQUATS:
               for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
                  sequenceSquats();
            case SQUATATHON:
               for (int i = 0; i < numberOfCyclesToRun.getIntegerValue(); i++)
                  sequenceSquatathon();
               break;
            case SHIFT_WEIGHT:
               sequenceShiftWeight();
               break;
            case SIMPLE_WARMUP:
               sequenceSimpleWarmup();
               break;
            case MEDIUM_WARMUP:
               sequenceMediumWarmup();
               break;
            case HARD_WARMUP:
               sequenceHardWarmup();
               break;
            case STEPS_IN_PLACE:
               sequenceStepsInPlace();
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
