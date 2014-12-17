package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HumanoidArmPose;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestOrientationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.ChestOrientationTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FootPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.PelvisPoseTask;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.taskExecutor.NullTask;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;

public class DiagnosticBehavior extends BehaviorInterface
{
   private static final boolean FAST_MOTION = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final SideDependentList<HandPoseBehavior> handPoseBehaviors = new SideDependentList<>();
   private final FootPoseBehavior footPoseBehavior;
   private final ChestOrientationBehavior chestOrientationBehavior;
   private final PelvisPoseBehavior pelvisPoseBehavior;

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable sleepTimeBetweenPoses;
   private final int numberOfArmJoints;

   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private enum DiagnosticTask
   {
      CHEST_MOTIONS, PELVIS_MOTIONS, COMBINED_CHEST_PELVIS, ARM_MOTIONS, UPPER_BODY, FOOT_POSES, RUNNING_MAN
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

   public DiagnosticBehavior(FullRobotModel fullRobotModel, EnumYoVariable<RobotSide> supportLeg, ReferenceFrames referenceFrames, DoubleYoVariable yoTime,
         BooleanYoVariable yoDoubleSupport, OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);

      this.supportLeg = supportLeg;

      numberOfArmJoints = fullRobotModel.getRobotSpecificJointNames().getArmJointNames().length;
      this.yoTime = yoTime;
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();

      String behaviorNameFirstLowerCase = FormattingTools.lowerCaseFirstLetter(getName());
      trajectoryTime = new DoubleYoVariable(behaviorNameFirstLowerCase + "TrajectoryTime", registry);
      trajectoryTime.set(FAST_MOTION ? 0.5 : 10.0);
      sleepTimeBetweenPoses = new DoubleYoVariable(behaviorNameFirstLowerCase + "SleepTimeBetweenPoses", registry);
      sleepTimeBetweenPoses.set(FAST_MOTION ? 0.0 : 2.0);

      chestOrientationBehavior = new ChestOrientationBehavior(outgoingCommunicationBridge, yoTime);
      registry.addChild(chestOrientationBehavior.getYoVariableRegistry());

      pelvisPoseBehavior = new PelvisPoseBehavior(outgoingCommunicationBridge, yoTime);
      registry.addChild(pelvisPoseBehavior.getYoVariableRegistry());

      footPoseBehavior = new FootPoseBehavior(outgoingCommunicationBridge, yoTime, yoDoubleSupport);
      registry.addChild(footPoseBehavior.getYoVariableRegistry());

      for (RobotSide robotSide : RobotSide.values)
      {
         HandPoseBehavior handPoseBehavior = new HandPoseBehavior(robotSide.getCamelCaseNameForMiddleOfExpression(), outgoingCommunicationBridge, yoTime);
         registry.addChild(handPoseBehavior.getYoVariableRegistry());
         handPoseBehaviors.put(robotSide, handPoseBehavior);
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
   }

   private void sequenceUpperBody()
   {
      sequenceArmPose();

      submitSymmetricHandPose(HumanoidArmPose.LARGE_CHICKEN_WINGS.getArmJointAngles());
      sequenceMovingChestOnly();
      sequenceMovingPelvisOnly();
      sequenceMovingChestAndPelvisOnly();

      submitSymmetricHandPose(HumanoidArmPose.REACH_FAR_FORWARD.getArmJointAngles());
      sequenceMovingChestOnly();
      sequenceMovingPelvisOnly();
      sequenceMovingChestAndPelvisOnly();

      submitSymmetricHandPose(HumanoidArmPose.REACH_FAR_BACK.getArmJointAngles());
      sequenceMovingChestOnly();
      sequenceMovingPelvisOnly();
      sequenceMovingChestAndPelvisOnly();

      submitHandPoses(new SideDependentList<double[]>(new double[] { 0.0, -Math.PI / 2.0, 0.0, 0.0 }, HumanoidArmPose.ARM_STRAIGHT_DOWN.getArmJointAngles()));
      sequenceMovingChestOnly();
      sequenceMovingPelvisOnly();
      sequenceMovingChestAndPelvisOnly();

      submitHandPoses(new SideDependentList<double[]>(HumanoidArmPose.ARM_STRAIGHT_DOWN.getArmJointAngles(), new double[] { 0.0, -Math.PI / 2.0, 0.0, 0.0 }));
      sequenceMovingChestOnly();
      sequenceMovingPelvisOnly();
      sequenceMovingChestAndPelvisOnly();

      submitSymmetricHandPose(HumanoidArmPose.STAND_PREP.getArmJointAngles());
   }

   private void sequenceMovingChestOnly()
   {
      double percentOfJointLimit = 0.55;
      double roll = percentOfJointLimit * minMaxRoll;
      submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, 0.0, percentOfJointLimit * minPitch, 0.0));
      submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, 0.0, 0.0, roll));
      submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, 0.0, 0.0, -roll));
      submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, 0.0, percentOfJointLimit * minPitch, roll));
      submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, 0.0, percentOfJointLimit * minPitch, -roll));

      submitDesiredChestOrientation(new FrameOrientation(pelvisZUpFrame, 0.0, 0.0, 0.0));
   }

   private void sequenceMovingPelvisOnly()
   {
      double percentOfJointLimit = 0.3;
      double roll = percentOfJointLimit * minMaxRoll;
      submitDesiredPelvisOrientation(0.0, percentOfJointLimit * minPitch, 0.0);
      submitDesiredPelvisOrientation(0.0, 0.0, roll);
      submitDesiredPelvisOrientation(0.0, 0.0, -roll);
      submitDesiredPelvisOrientation(0.0, percentOfJointLimit * minPitch, roll);
      submitDesiredPelvisOrientation(0.0, percentOfJointLimit * minPitch, -roll);

      submitDesiredPelvisOrientation(0.0, 0.0, 0.0);
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
      pipeLine.submit(footPoseBehavior, new FootPoseTask(robotSide, footPose, yoTime, footPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      FrameOrientation desiredChestOrientation = new FrameOrientation(pelvisZUpFrame, 0.0, Math.toRadians(20.0), 0.0);
      desiredChestOrientation.changeFrame(worldFrame);
      pipeLine.submit(chestOrientationBehavior, new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
      FrameOrientation desiredPelvisOrientation = new FrameOrientation(findFixedFrameForPelvisOrientation());
      desiredPelvisOrientation.setYawPitchRoll(0.0, Math.toRadians(10.0), 0.0);
      desiredPelvisOrientation.changeFrame(worldFrame);
      pipeLine.submit(pelvisPoseBehavior, new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
      
      pipeLine.submit(new NullTask());
      
      // Do a "Y" stance with the foot outside
      submitSymmetricHandPose(new double[] { 0.0, -1.5 * Math.PI / 2.0, 0.0, 0.0 });

      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.65), 0.1);
      footPose.setOrientation(0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(40.0)));
      footPose.changeFrame(worldFrame);
      pipeLine.submit(footPoseBehavior, new FootPoseTask(robotSide, footPose, yoTime, footPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
      desiredChestOrientation.setIncludingFrame(pelvisZUpFrame, 0.0, 0.0, 0.0);
      desiredChestOrientation.changeFrame(worldFrame);
      pipeLine.submit(chestOrientationBehavior, new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
      desiredPelvisOrientation.setToZero(findFixedFrameForPelvisOrientation());
      desiredPelvisOrientation.setYawPitchRoll(0.0, 0.0, robotSide.negateIfRightSide(Math.toRadians(25.0)));
      desiredPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      pipeLine.submit(pelvisPoseBehavior, new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      pipeLine.submit(new NullTask());

      // Go back to stand prep but don't put the foot on the ground yet
      submitSymmetricHandPose(HumanoidArmPose.STAND_PREP.getArmJointAngles());

      footPose.setToZero(ankleZUpFrame);
      footPose.setPosition(0.0, robotSide.negateIfRightSide(0.25), 0.1);
      footPose.setOrientation(0.0, 0.0, 0.0);
      footPose.changeFrame(worldFrame);
      pipeLine.submit(footPoseBehavior, new FootPoseTask(robotSide, footPose, yoTime, footPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));

      desiredPelvisOrientation.setToZero(findFixedFrameForPelvisOrientation());
      desiredPelvisOrientation.setYawPitchRoll(0.0, 0.0, 0.0);
      desiredPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      pipeLine.submit(pelvisPoseBehavior, new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));


      submitFootPosition(robotSide, new FramePoint(ankleZUpFrame, 0.0, robotSide.negateIfRightSide(0.25), -0.3));
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
      pipeLine.submit(new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior, trajectoryTime.getDoubleValue(),
            sleepTimeBetweenPoses.getDoubleValue()));
   }

   private void submitDesiredPelvisOrientation(double yaw, double pitch, double roll)
   {
      FrameOrientation desiredPelvisOrientation = new FrameOrientation(findFixedFrameForPelvisOrientation(), yaw, pitch, roll);
      desiredPelvisOrientation.changeFrame(worldFrame);
      pipeLine.submit(new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses
            .getDoubleValue()));
   }

   private void submitDesiredChestAndPelvisOrientationsParallel(FrameOrientation desiredChestOrientation, double pelvisYaw, double pelvisPitch,
         double pelvisRoll)
   {
      desiredChestOrientation.checkReferenceFrameMatch(pelvisZUpFrame);
      desiredChestOrientation.changeFrame(worldFrame);
      FrameOrientation desiredPelvisOrientation = new FrameOrientation(findFixedFrameForPelvisOrientation(), pelvisYaw, pelvisPitch, pelvisRoll);
      desiredPelvisOrientation.changeFrame(worldFrame);
      pipeLine.submit(
            chestOrientationBehavior,
            new ChestOrientationTask(desiredChestOrientation, yoTime, chestOrientationBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses
                  .getDoubleValue()));
      pipeLine.submit(pelvisPoseBehavior, new PelvisPoseTask(desiredPelvisOrientation, yoTime, pelvisPoseBehavior, trajectoryTime.getDoubleValue(),
            sleepTimeBetweenPoses.getDoubleValue()));
   }

   private void submitSingleHandPose(RobotSide robotSide, double[] desiredJointAngles)
   {
      double[] temp = new double[numberOfArmJoints];
      double[] src = desiredJointAngles;
      if (numberOfArmJoints > src.length)
      {
         System.arraycopy(src, 0, temp, 0, src.length);
      }
      else
         temp = src;

      HandPoseBehavior handPoseBehavior = handPoseBehaviors.get(robotSide);
      pipeLine.submit(handPoseBehavior,
            new HandPoseTask(robotSide, temp, yoTime, handPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
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
         pipeLine.submit(handPoseBehavior,
               new HandPoseTask(robotSide, temp, yoTime, handPoseBehavior, trajectoryTime.getDoubleValue(), sleepTimeBetweenPoses.getDoubleValue()));
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
         pipeLine.submit(handPoseBehavior, new HandPoseTask(robotSide, desiredJointAngles, yoTime, handPoseBehavior, trajectoryTime.getDoubleValue(),
               sleepTimeBetweenPoses.getDoubleValue()));
      }
   }

   private void submitFootPosition(RobotSide robotSide, FramePoint desiredFootPosition)
   {
      FrameOrientation desiredFootOrientation = new FrameOrientation(desiredFootPosition.getReferenceFrame());
      FramePose desiredFootPose = new FramePose(desiredFootPosition, desiredFootOrientation);
      submitFootPose(robotSide, desiredFootPose);
   }

   private void submitFootPose(RobotSide robotSide, FramePose desiredFootPose)
   {
//      desiredFootPose.checkReferenceFrameMatch(ankleZUpFrames.get(robotSide));
      desiredFootPose.changeFrame(worldFrame);
      Point3d desiredFootPosition = new Point3d();
      Quat4d desiredFootOrientation = new Quat4d();
      desiredFootPose.getPose(desiredFootPosition, desiredFootOrientation);
      pipeLine.submit(new FootPoseTask(robotSide, desiredFootPosition, desiredFootOrientation, yoTime, footPoseBehavior, trajectoryTime.getDoubleValue(),
            sleepTimeBetweenPoses.getDoubleValue()));
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
            case CHEST_MOTIONS:
               sequenceMovingChestOnly();
            case PELVIS_MOTIONS:
               sequenceMovingPelvisOnly();
            case COMBINED_CHEST_PELVIS:
               sequenceMovingChestAndPelvisOnly();
            case UPPER_BODY:
               sequenceUpperBody();
            case FOOT_POSES:
               sequenceFootPose();
            case RUNNING_MAN:
               sequenceRunningMan();
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
      }
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         handPoseBehaviors.get(robotSide).consumeObjectFromController(object);
      }
   }

   @Override
   public void stop()
   {
      pipeLine.clear();
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
