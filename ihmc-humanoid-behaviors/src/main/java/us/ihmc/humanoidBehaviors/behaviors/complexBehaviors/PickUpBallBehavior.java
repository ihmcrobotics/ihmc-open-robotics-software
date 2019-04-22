package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.util.ArrayList;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElementBehaviorSideOLD;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElementOLD.PickUpBallBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ArmTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ClearLidarBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.EnableLidarBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.GoHomeBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandDesiredConfigurationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HeadTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.SetLidarParametersBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicsBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BlobFilteredSphereDetectionBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SphereDetectionBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.WaitForUserValidationBehavior;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.humanoidBehaviors.taskExecutor.ArmTrajectoryTask;
import us.ihmc.humanoidBehaviors.taskExecutor.ChestOrientationTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GoHomeTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandDesiredConfigurationTask;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.ihmcPerception.vision.shapes.HSVRange;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.taskExecutor.PipeLine;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PickUpBallBehavior extends AbstractBehavior
{
   private static final boolean FILTER_KNOWN_COLORS_TO_SPEED_UP = false;

   private final PickUpBallBehaviorCoactiveElementBehaviorSideOLD coactiveElement;

   private final ArrayList<AbstractBehavior> behaviors = new ArrayList<AbstractBehavior>();
   private final EnableLidarBehavior enableBehaviorOnlyLidarBehavior;
   private final SetLidarParametersBehavior setLidarParametersBehavior;
   private final ClearLidarBehavior clearLidarBehavior;
   private final SphereDetectionBehavior initialSphereDetectionBehavior;
   private final BlobFilteredSphereDetectionBehavior blobFilteredSphereDetectionBehavior;
   private final WaitForUserValidationBehavior waitForUserValidationBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final ChestTrajectoryBehavior chestTrajectoryBehavior;
   private final HandDesiredConfigurationBehavior handDesiredConfigurationBehavior;
   private final WholeBodyInverseKinematicsBehavior wholeBodyBehavior;
   private final GoHomeBehavior chestGoHomeBehavior;
   private final GoHomeBehavior pelvisGoHomeBehavior;
   private final GoHomeBehavior armGoHomeLeftBehavior;
   private final GoHomeBehavior armGoHomeRightBehavior;
   private final ArmTrajectoryBehavior armTrajectoryBehavior;

   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<>();

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoDouble yoTime;
   private final ReferenceFrame midZupFrame;
   private final double standingDistance = 0.4; // 0.5;

   private HumanoidReferenceFrames referenceFrames;
   private final ReferenceFrame chestCoMFrame;
   private final ReferenceFrame pelvisZUpFrame;

   public PickUpBallBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime, YoBoolean yoDoubleSupport, FullHumanoidRobotModel fullRobotModel,
                             HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters,
                             FullHumanoidRobotModelFactory robotModelFactory)
   {
      super(robotName, ros2Node);
      this.yoTime = yoTime;
      chestCoMFrame = fullRobotModel.getChest().getBodyFixedFrame();
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();

      midZupFrame = referenceFrames.getMidFeetZUpFrame();
      this.referenceFrames = referenceFrames;

      // create sub-behaviors:
      setLidarParametersBehavior = new SetLidarParametersBehavior(robotName, ros2Node);
      behaviors.add(setLidarParametersBehavior);

      enableBehaviorOnlyLidarBehavior = new EnableLidarBehavior(robotName, ros2Node);
      behaviors.add(enableBehaviorOnlyLidarBehavior);

      clearLidarBehavior = new ClearLidarBehavior(robotName, ros2Node);
      behaviors.add(clearLidarBehavior);

      blobFilteredSphereDetectionBehavior = new BlobFilteredSphereDetectionBehavior(robotName, ros2Node, referenceFrames, fullRobotModel);
      initialSphereDetectionBehavior = new SphereDetectionBehavior(robotName, ros2Node, referenceFrames);
      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_ORANGE_BALL);
      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_BLUE_BALL);
      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_RED_BALL);
      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_YELLOW_BALL);
      blobFilteredSphereDetectionBehavior.addHSVRange(HSVRange.USGAMES_GREEN_BALL);
      behaviors.add(FILTER_KNOWN_COLORS_TO_SPEED_UP ? blobFilteredSphereDetectionBehavior : initialSphereDetectionBehavior);

      walkToLocationBehavior = new WalkToLocationBehavior(robotName, ros2Node, fullRobotModel, referenceFrames,
                                                          wholeBodyControllerParameters.getWalkingControllerParameters());
      behaviors.add(walkToLocationBehavior);
      wholeBodyBehavior = new WholeBodyInverseKinematicsBehavior(robotName, robotModelFactory, yoTime, ros2Node, fullRobotModel);

      behaviors.add(wholeBodyBehavior);

      chestTrajectoryBehavior = new ChestTrajectoryBehavior(robotName, ros2Node, yoTime);
      behaviors.add(chestTrajectoryBehavior);

      chestGoHomeBehavior = new GoHomeBehavior(robotName, "chest", ros2Node, yoTime);
      behaviors.add(chestGoHomeBehavior);

      pelvisGoHomeBehavior = new GoHomeBehavior(robotName, "pelvis", ros2Node, yoTime);
      behaviors.add(pelvisGoHomeBehavior);

      armGoHomeLeftBehavior = new GoHomeBehavior(robotName, "leftArm", ros2Node, yoTime);
      behaviors.add(armGoHomeLeftBehavior);
      armGoHomeRightBehavior = new GoHomeBehavior(robotName, "rightArm", ros2Node, yoTime);
      behaviors.add(armGoHomeRightBehavior);
      armTrajectoryBehavior = new ArmTrajectoryBehavior(robotName, "handTrajectory", ros2Node, yoTime);
      behaviors.add(armTrajectoryBehavior);

      handDesiredConfigurationBehavior = new HandDesiredConfigurationBehavior(robotName, "left", ros2Node, yoTime);
      behaviors.add(handDesiredConfigurationBehavior);

      coactiveElement = new PickUpBallBehaviorCoactiveElementBehaviorSideOLD();
      //      coactiveElement.setPickUpBallBehavior(this);
      registry.addChild(coactiveElement.getUserInterfaceWritableYoVariableRegistry());
      registry.addChild(coactiveElement.getMachineWritableYoVariableRegistry());

      waitForUserValidationBehavior = new WaitForUserValidationBehavior(robotName, ros2Node, coactiveElement.validClicked, coactiveElement.validAcknowledged);
      behaviors.add(waitForUserValidationBehavior);

      for (AbstractBehavior behavior : behaviors)
      {
         registry.addChild(behavior.getYoVariableRegistry());
      }
   }

   @Override
   public CoactiveElement getCoactiveElement()
   {
      return null;//coactiveElement;
   }

   boolean locationSet = false;

   @Override
   public void doControl()
   {
      pipeLine.doControl();
   }

   private void setupPipelineForKick()
   {

      HandDesiredConfigurationTask closeHand = new HandDesiredConfigurationTask(RobotSide.LEFT, HandConfiguration.CLOSE, handDesiredConfigurationBehavior);
      HandDesiredConfigurationTask openHand = new HandDesiredConfigurationTask(RobotSide.LEFT, HandConfiguration.OPEN, handDesiredConfigurationBehavior);

      //ENABLE LIDAR
      BehaviorAction enableLidarTask = new BehaviorAction(enableBehaviorOnlyLidarBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            // FIXME
            //            enableBehaviorOnlyLidarBehavior.setLidarState(LidarState.ENABLE_BEHAVIOR_ONLY);
            coactiveElement.currentState.set(PickUpBallBehaviorState.ENABLING_LIDAR);
         }
      };

      //REDUCE LIDAR RANGE *******************************************

      BehaviorAction setLidarMediumRangeTask = new BehaviorAction(setLidarParametersBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            coactiveElement.currentState.set(PickUpBallBehaviorState.SETTING_LIDAR_PARAMS);
            DepthDataFilterParameters param = new DepthDataFilterParameters();
            param.nearScanRadius = 1.4f;
            setLidarParametersBehavior.setInput(param);
         }
      };

      //CLEAR LIDAR POINTS FOR CLEAN SCAN *******************************************
      BehaviorAction clearLidarTask = new BehaviorAction(clearLidarBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            coactiveElement.currentState.set(PickUpBallBehaviorState.CLEARING_LIDAR);
         }
      };

      //SEARCH FOR BALL *******************************************

      BehaviorAction findBallTask = new BehaviorAction(initialSphereDetectionBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("LOOKING FOR BALL");
            coactiveElement.currentState.set(PickUpBallBehaviorState.SEARCHING_FOR_BALL);
            coactiveElement.searchingForBall.set(true);
            coactiveElement.foundBall.set(false);
            coactiveElement.ballX.set(0);
            coactiveElement.ballY.set(0);
            coactiveElement.ballZ.set(0);

         }
      };

      //

      // Confirm from the user that this is the correct ball *******************************************

      BehaviorAction validateBallTask = new BehaviorAction(waitForUserValidationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            coactiveElement.currentState.set(PickUpBallBehaviorState.WAITING_FOR_USER_CONFIRMATION);
            coactiveElement.searchingForBall.set(false);
            coactiveElement.waitingForValidation.set(true);
            coactiveElement.validAcknowledged.set(false);
            coactiveElement.foundBall.set(false);
            coactiveElement.ballX.set(initialSphereDetectionBehavior.getBallLocation().getX());
            coactiveElement.ballY.set(initialSphereDetectionBehavior.getBallLocation().getY());
            coactiveElement.ballZ.set(initialSphereDetectionBehavior.getBallLocation().getZ());
            coactiveElement.ballRadius.set(initialSphereDetectionBehavior.getSpehereRadius());

         }
      };

      // WALK TO THE BALL *******************************************

      BehaviorAction walkToBallTask = new BehaviorAction(walkToLocationBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Walking To The Ball");
            coactiveElement.currentState.set(PickUpBallBehaviorState.WALKING_TO_BALL);
            coactiveElement.searchingForBall.set(false);
            coactiveElement.waitingForValidation.set(false);
            coactiveElement.foundBall.set(true);
            walkToLocationBehavior.setTarget(getoffsetPoint());
         }
      };

      //LOOK DOWN *******************************************

      Vector3D axis = new Vector3D(0, 1, 0);
      double rotationDownAngle = 1.4;

      AxisAngle desiredAxisAngle = new AxisAngle();
      desiredAxisAngle.set(axis, rotationDownAngle);
      Quaternion desiredHeadQuat = new Quaternion();
      desiredHeadQuat.set(desiredAxisAngle);

      HeadTrajectoryMessage message = HumanoidMessageTools.createHeadTrajectoryMessage(1, desiredHeadQuat, worldFrame, chestCoMFrame);

      HeadTrajectoryBehavior headTrajectoryBehavior = new HeadTrajectoryBehavior(robotName, ros2Node, yoTime);

      headTrajectoryBehavior.initialize();
      headTrajectoryBehavior.setInput(message);

      BehaviorAction lookDown = new BehaviorAction(headTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            coactiveElement.currentState.set(PickUpBallBehaviorState.BENDING_OVER);
         }
      };

      double rotationUpAngle = 0;

      AxisAngle desiredAxisUpAngle = new AxisAngle();
      desiredAxisUpAngle.set(axis, rotationUpAngle);
      Quaternion desiredHeadUpQuat = new Quaternion();
      desiredHeadUpQuat.set(desiredAxisUpAngle);

      HeadTrajectoryMessage messageHeadUp = HumanoidMessageTools.createHeadTrajectoryMessage(1, desiredHeadUpQuat, worldFrame, chestCoMFrame);

      HeadTrajectoryBehavior headTrajectoryUpBehavior = new HeadTrajectoryBehavior(robotName, ros2Node, yoTime);

      headTrajectoryUpBehavior.initialize();
      headTrajectoryUpBehavior.setInput(messageHeadUp);

      BehaviorAction lookUp = new BehaviorAction(headTrajectoryUpBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            coactiveElement.currentState.set(PickUpBallBehaviorState.STOPPED);
         }
      };

      // BEND OVER *******************************************
      FrameQuaternion desiredChestOrientation = new FrameQuaternion(referenceFrames.getPelvisZUpFrame(), Math.toRadians(30), Math.toRadians(20), 0);
      desiredChestOrientation.changeFrame(worldFrame);
      ChestOrientationTask chestOrientationTask = new ChestOrientationTask(desiredChestOrientation, chestTrajectoryBehavior, 4, pelvisZUpFrame);

      //REDUCE LIDAR RANGE *******************************************

      BehaviorAction setLidarShortRangeTask = new BehaviorAction(setLidarParametersBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            coactiveElement.currentState.set(PickUpBallBehaviorState.SETTING_LIDAR_PARAMS);
            DepthDataFilterParameters param = new DepthDataFilterParameters();
            param.nearScanRadius = 0.6f;
            setLidarParametersBehavior.setInput(param);
         }
      };

      //CLEAR LIDAR POINTS FOR CLEAN SCAN *******************************************
      BehaviorAction clearLidarTask2 = new BehaviorAction(clearLidarBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            coactiveElement.currentState.set(PickUpBallBehaviorState.CLEARING_LIDAR);
         }
      };

      //SEARCH FOR BALL AGAIN FOR FINAL LOCATION *******************************************
      BehaviorAction finalFindBallTask = new BehaviorAction(initialSphereDetectionBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Looking for the ball");
            coactiveElement.currentState.set(PickUpBallBehaviorState.SEARCHING_FOR_BALL);
            coactiveElement.searchingForBall.set(true);
            coactiveElement.foundBall.set(false);
            coactiveElement.ballX.set(0);
            coactiveElement.ballY.set(0);
            coactiveElement.ballZ.set(0);

         }
      };

      // re-validat ball with user *******************************************
      BehaviorAction validateBallTask2 = new BehaviorAction(waitForUserValidationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            coactiveElement.currentState.set(PickUpBallBehaviorState.WAITING_FOR_USER_CONFIRMATION);
            coactiveElement.searchingForBall.set(false);
            coactiveElement.waitingForValidation.set(true);
            coactiveElement.validAcknowledged.set(false);
            coactiveElement.foundBall.set(false);
            coactiveElement.ballX.set(initialSphereDetectionBehavior.getBallLocation().getX());
            coactiveElement.ballY.set(initialSphereDetectionBehavior.getBallLocation().getY());
            coactiveElement.ballZ.set(initialSphereDetectionBehavior.getBallLocation().getZ());
            coactiveElement.ballRadius.set(initialSphereDetectionBehavior.getSpehereRadius());

         }
      };

      // GO TO INITIAL POICKUP LOCATION *******************************************
      BehaviorAction goToPickUpBallInitialLocationTask = new BehaviorAction(wholeBodyBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            System.out.println("Picking up Ball " + initialSphereDetectionBehavior.getBallLocation().getX() + " "
                  + initialSphereDetectionBehavior.getBallLocation().getY() + " " + initialSphereDetectionBehavior.getBallLocation().getZ()
                  + initialSphereDetectionBehavior.getSpehereRadius() + 0.25);

            publishTextToSpeech("I think i found the ball");
            coactiveElement.currentState.set(PickUpBallBehaviorState.REACHING_FOR_BALL);
            FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), initialSphereDetectionBehavior.getBallLocation().getX(),
                                                  initialSphereDetectionBehavior.getBallLocation().getY(),
                                                  initialSphereDetectionBehavior.getBallLocation().getZ() + initialSphereDetectionBehavior.getSpehereRadius()
                                                        + 0.25);
            wholeBodyBehavior.setSolutionQualityThreshold(2.01);
            wholeBodyBehavior.setTrajectoryTime(3);
            FrameQuaternion tmpOr = new FrameQuaternion(point.getReferenceFrame(), Math.toRadians(45), Math.toRadians(90), 0);
            wholeBodyBehavior.setDesiredHandPose(RobotSide.LEFT, point, tmpOr);

            FramePoint3D rhPoint = new FramePoint3D(referenceFrames.getChestFrame(), 0.6, -0.25, 0);
            FrameQuaternion rhOr = new FrameQuaternion(point.getReferenceFrame(), Math.toRadians(45), 0, 0);

            //            wholeBodyBehavior.setDesiredHandPose(RobotSide.RIGHT, rhPoint, rhOr);

         }
      };

      //REACH FOR THE BALL *******************************************
      BehaviorAction pickUpBallTask = new BehaviorAction(wholeBodyBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), initialSphereDetectionBehavior.getBallLocation().getX(),
                                                  initialSphereDetectionBehavior.getBallLocation().getY(),
                                                  initialSphereDetectionBehavior.getBallLocation().getZ() + initialSphereDetectionBehavior.getSpehereRadius());
            wholeBodyBehavior.setSolutionQualityThreshold(2.01);
            wholeBodyBehavior.setTrajectoryTime(3);
            FrameQuaternion tmpOr = new FrameQuaternion(point.getReferenceFrame(), Math.toRadians(45), Math.toRadians(90), 0);
            wholeBodyBehavior.setDesiredHandPose(RobotSide.LEFT, point, tmpOr);

         }
      };

      //PICK UP THE BALL *******************************************

      //      BehaviorTask goToFinalPickUpBallLocationTask = new BehaviorTask(wholeBodyBehavior, yoTime, 0)
      //      {
      //         @Override
      //         protected void setBehaviorInput()
      //         {
      //            coactiveElement.currentState.set(BehaviorState.PICKING_UP_BALL);
      //
      //            FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), initialSphereDetectionBehavior.getBallLocation().x,
      //                  initialSphereDetectionBehavior.getBallLocation().y,
      //                  initialSphereDetectionBehavior.getBallLocation().z + initialSphereDetectionBehavior.getSpehereRadius() + 0.03);
      //            wholeBodyBehavior.setSolutionQualityThreshold(2.01);
      //            wholeBodyBehavior.setTrajectoryTime(6);
      //            FrameOrientation tmpOr = new FrameOrientation(point.getReferenceFrame(), Math.toRadians(90), Math.toRadians(90), 0);
      //            wholeBodyBehavior.setDesiredHandPose(RobotSide.LEFT, point, tmpOr);
      //
      //         }
      //      };

      //RESET BODY POSITIONS *******************************************
      GoHomeMessage goHomeChestMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.CHEST, 2);
      chestGoHomeBehavior.setInput(goHomeChestMessage);
      GoHomeTask goHomeChestTask = new GoHomeTask(goHomeChestMessage, chestGoHomeBehavior);

      GoHomeMessage goHomepelvisMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.PELVIS, 2);
      pelvisGoHomeBehavior.setInput(goHomepelvisMessage);
      GoHomeTask goHomePelvisTask = new GoHomeTask(goHomepelvisMessage, pelvisGoHomeBehavior);

      GoHomeMessage goHomeLeftArmMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.ARM, RobotSide.LEFT, 2);
      armGoHomeLeftBehavior.setInput(goHomeLeftArmMessage);
      GoHomeTask goHomeLeftArmTask = new GoHomeTask(goHomeLeftArmMessage, armGoHomeLeftBehavior);

      GoHomeMessage goHomeRightArmMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.ARM, RobotSide.RIGHT, 2);
      armGoHomeRightBehavior.setInput(goHomeRightArmMessage);
      GoHomeTask goHomeRightArmTask = new GoHomeTask(goHomeRightArmMessage, armGoHomeRightBehavior);

      double[] rightHandWiderHomeJointAngles = new double[] {-0.785398, 0.5143374964757462, 2.2503094898479272, -2.132492022530739, -0.22447272781774874,
            -0.4780687104960028, -0.24919417978503655};

      ArmTrajectoryMessage widerHome = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 2, rightHandWiderHomeJointAngles);

      ArmTrajectoryTask rightArmHomeTask = new ArmTrajectoryTask(widerHome, armTrajectoryBehavior);

      double[] rightHandBucketLocation1 = new double[] {0.5489321822438367, 0.2899665391571677, 2.096340823983413, -1.2225333451166707, 0.1256161514011733,
            -1.3433026185064938, -1.1994258903111514};

      ArmTrajectoryMessage rightHandBucketLocation1Message = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 2, rightHandBucketLocation1);

      ArmTrajectoryTask rightHandBucketLocation1Task = new ArmTrajectoryTask(rightHandBucketLocation1Message, armTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            super.setBehaviorInput();
            publishTextToSpeech("Putting The Ball In The Bucket");
            coactiveElement.currentState.set(PickUpBallBehaviorState.PUTTING_BALL_IN_BASKET);
         }
      };

      double[] leftHandBucketLocation1 = new double[] {-0.5609186812662719, -0.39273790125704305, 1.89931104400202, 1.8345084796174007, -1.9173410679363112,
            -0.7657081703756509, -0.7098631227127279};

      ArmTrajectoryMessage leftHandBucketLocation1Message = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, 2, leftHandBucketLocation1);

      ArmTrajectoryTask leftHandBucketLocation1Task = new ArmTrajectoryTask(leftHandBucketLocation1Message, armTrajectoryBehavior);

      double[] rightHandBucketLocation2 = new double[] {0.4765048070153984, 0.305694742754363, 2.173812006625049, -1.4970540590789951, 0.10321456673940527,
            -1.2120648871681976, -1.1591439074587626};

      ArmTrajectoryMessage rightHandBucketLocation2Message = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 2, rightHandBucketLocation2);

      ArmTrajectoryTask rightHandBucketLocation2Task = new ArmTrajectoryTask(rightHandBucketLocation2Message, armTrajectoryBehavior);

      double[] leftHandBucketLocation2 = new double[] {-0.6312858675745908, -0.6560594198655715, 2.026449179186367, 2.0325182474649997, -1.4129369066719957,
            -0.33189990885720594, -1.1435699210219243};

      ArmTrajectoryMessage leftHandBucketLocation2Message = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, 2, leftHandBucketLocation2);

      ArmTrajectoryTask leftHandBucketLocation2Task = new ArmTrajectoryTask(leftHandBucketLocation2Message, armTrajectoryBehavior);

      double[] leftHandAfterGrabLocation = new double[] {-0.799566492522621, -0.8850712601496326, 1.1978163314288173, 0.9978871050058826, -0.22593401111949774,
            -0.2153318563363089, -1.2957848304397805};

      ArmTrajectoryMessage leftHandAfterGrabMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, 2, leftHandAfterGrabLocation);

      ArmTrajectoryTask leftHandBeforeGrab = new ArmTrajectoryTask(leftHandAfterGrabMessage, armTrajectoryBehavior);

      ArmTrajectoryTask leftHandAfterGrab = new ArmTrajectoryTask(leftHandAfterGrabMessage, armTrajectoryBehavior);

      // TASK SETUP
      pipeLine.submitTaskForPallelPipesStage(handDesiredConfigurationBehavior, closeHand);
      pipeLine.submitTaskForPallelPipesStage(enableBehaviorOnlyLidarBehavior, enableLidarTask);
      pipeLine.submitTaskForPallelPipesStage(setLidarParametersBehavior, setLidarMediumRangeTask);

      pipeLine.submitSingleTaskStage(clearLidarTask);

      pipeLine.requestNewStage();

      pipeLine.submitTaskForPallelPipesStage(armTrajectoryBehavior, goHomeRightArmTask);

      pipeLine.submitTaskForPallelPipesStage(initialSphereDetectionBehavior, findBallTask);

      //LOOK AROUND

      //      pipeLine.submitSingleTaskStage(validateBallTask);
      pipeLine.submitSingleTaskStage(rightArmHomeTask);
      //RECENTER BODY
      pipeLine.submitSingleTaskStage(walkToBallTask);

      pipeLine.requestNewStage();

      pipeLine.submitTaskForPallelPipesStage(headTrajectoryBehavior, lookDown);
      pipeLine.submitTaskForPallelPipesStage(chestTrajectoryBehavior, chestOrientationTask);
      pipeLine.submitTaskForPallelPipesStage(setLidarParametersBehavior, setLidarShortRangeTask);

      pipeLine.requestNewStage();

      pipeLine.submitSingleTaskStage(clearLidarTask2);
      pipeLine.submitSingleTaskStage(finalFindBallTask);
      //      pipeLine.submitSingleTaskStage(validateBallTask2);

      pipeLine.submitSingleTaskStage(leftHandBeforeGrab);

      pipeLine.requestNewStage();

      pipeLine.submitTaskForPallelPipesStage(wholeBodyBehavior, goToPickUpBallInitialLocationTask);
      pipeLine.submitTaskForPallelPipesStage(handDesiredConfigurationBehavior, openHand);

      pipeLine.requestNewStage();

      pipeLine.submitSingleTaskStage(pickUpBallTask);
      //      pipeLine.submitSingleTaskStage(goToFinalPickUpBallLocationTask);
      pipeLine.submitSingleTaskStage(closeHand);

      pipeLine.submitSingleTaskStage(leftHandAfterGrab);
      //

      pipeLine.submitSingleTaskStage(goHomeChestTask);
      pipeLine.submitSingleTaskStage(goHomePelvisTask);

      //PUT BALL IN BUCKET
      //      pipeLine.submitSingleTaskStage(rightArmHomeTask);

      pipeLine.submitSingleTaskStage(rightHandBucketLocation1Task);
      pipeLine.submitSingleTaskStage(leftHandBucketLocation1Task);

      pipeLine.submitSingleTaskStage(rightHandBucketLocation2Task);
      pipeLine.submitSingleTaskStage(leftHandBucketLocation2Task);

      pipeLine.submitSingleTaskStage(openHand);
      pipeLine.submitSingleTaskStage(closeHand);
      pipeLine.submitSingleTaskStage(openHand);

      pipeLine.submitSingleTaskStage(leftHandBucketLocation1Task);

      //      pipeLine.submitSingleTaskStage(goToDropBallInitialLocationTask);
      //      pipeLine.submitSingleTaskStage(goToDropBallFinalLocationTask);

      //
      //
      //      pipeLine.submitSingleTaskStage(goToDropBallInitialLocationTask);
      //      pipeLine.submitSingleTaskStage(closeHand);
      pipeLine.submitSingleTaskStage(rightArmHomeTask);
      //
      pipeLine.submitSingleTaskStage(goHomeLeftArmTask);
      //
      pipeLine.submitSingleTaskStage(goHomeChestTask);
      pipeLine.submitSingleTaskStage(goHomePelvisTask);
      //
      //
      //      pipeLine.submitSingleTaskStage(rightArmHomeTask);
      //      pipeLine.submitSingleTaskStage(lookUp);
   }

   private FramePose2D getoffsetPoint()
   {

      FramePoint2D ballPosition2d = new FramePoint2D(ReferenceFrame.getWorldFrame(), initialSphereDetectionBehavior.getBallLocation().getX(),
                                                     initialSphereDetectionBehavior.getBallLocation().getY());
      FramePoint2D robotPosition = new FramePoint2D(midZupFrame, 0.0, 0.0);
      robotPosition.changeFrame(worldFrame);
      FrameVector2D walkingDirection = new FrameVector2D(worldFrame);
      walkingDirection.set(ballPosition2d);
      walkingDirection.sub(robotPosition);
      walkingDirection.normalize();
      double walkingYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());

      //get a point offset from the ball
      double x = ballPosition2d.getX() - walkingDirection.getX() * standingDistance;
      double y = ballPosition2d.getY() - walkingDirection.getY() * standingDistance;
      double rotationAngle = Math.toRadians(55);
      //rotate that point around the ball so that the robot stands to the side.

      double newX = ballPosition2d.getX() + (x - ballPosition2d.getX()) * Math.cos(rotationAngle) - (y - ballPosition2d.getY()) * Math.sin(rotationAngle);
      double newY = ballPosition2d.getY() + (x - ballPosition2d.getX()) * Math.sin(rotationAngle) + (y - ballPosition2d.getY()) * Math.cos(rotationAngle);

      FramePose2D poseToWalkTo = new FramePose2D(worldFrame, new Point2D(newX, newY), walkingYaw);
      return poseToWalkTo;
   }

   @Override
   public void onBehaviorEntered()
   {
      setupPipelineForKick();

   }

   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("YAY IM ALL DONE");
      coactiveElement.currentState.set(PickUpBallBehaviorState.STOPPED);

      coactiveElement.searchingForBall.set(false);
      coactiveElement.waitingForValidation.set(false);
      coactiveElement.foundBall.set(false);
      coactiveElement.ballX.set(0);
      coactiveElement.ballY.set(0);
      coactiveElement.ballZ.set(0);
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.doPostBehaviorCleanup();
      }
   }

   @Override
   public void onBehaviorAborted()
   {
      onBehaviorExited();
      this.pipeLine.clearAll();

      for (AbstractBehavior behavior : behaviors)
      {
         behavior.abort();
      }
   }

   @Override
   public void onBehaviorPaused()
   {
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorPaused();
      }
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

   @Override
   public void onBehaviorResumed()
   {
   }

}
