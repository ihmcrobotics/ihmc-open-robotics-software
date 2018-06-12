package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AtlasPrimitiveActions
{
   //ALL Primitive Behaviors Go In Here. Talk To John Carff before changing this.
   public final ArmTrajectoryBehavior leftArmTrajectoryBehavior;
   public final ArmTrajectoryBehavior rightArmTrajectoryBehavior;
   public final ChestTrajectoryBehavior chestTrajectoryBehavior;
   public final ClearLidarBehavior clearLidarBehavior;
   public final EnableLidarBehavior enableLidarBehavior;
   public final FootLoadBearingBehavior leftFootEndEffectorLoadBearingBehavior;
   public final FootLoadBearingBehavior rightFootEndEffectorLoadBearingBehavior;
   public final FootstepListBehavior footstepListBehavior;
   public final GoHomeBehavior leftArmGoHomeBehavior;
   public final GoHomeBehavior rightArmGoHomeBehavior;
   public final GoHomeBehavior chestGoHomeBehavior;
   public final GoHomeBehavior pelvisGoHomeBehavior;
   public final HandDesiredConfigurationBehavior leftHandDesiredConfigurationBehavior;
   public final HandDesiredConfigurationBehavior rightHandDesiredConfigurationBehavior;
   public final HandTrajectoryBehavior leftHandTrajectoryBehavior;
   public final HandTrajectoryBehavior rightHandTrajectoryBehavior;
   public final HeadTrajectoryBehavior headTrajectoryBehavior;
   public final PelvisHeightTrajectoryBehavior pelvisHeightTrajectoryBehavior;
   public final PelvisOrientationTrajectoryBehavior pelvisOrientationTrajectoryBehavior;
   public final PelvisTrajectoryBehavior pelvisTrajectoryBehavior;
   public final SetLidarParametersBehavior setLidarParametersBehavior;
   public final WalkToLocationBehavior walkToLocationBehavior;
   public final WholeBodyInverseKinematicsBehavior wholeBodyBehavior;
   public final WalkToLocationPlannedBehavior walkToLocationPlannedBehavior;
   private final YoVariableRegistry behaviorRegistry;

   public HumanoidReferenceFrames referenceFrames;

   public AtlasPrimitiveActions(Ros2Node ros2Node, FullHumanoidRobotModel fullRobotModel,
                                FullHumanoidRobotModelFactory fullRobotModelFactory, HumanoidReferenceFrames referenceFrames, YoDouble yoTime,
                                WholeBodyControllerParameters wholeBodyControllerParameters, YoVariableRegistry behaviorRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.behaviorRegistry = behaviorRegistry;

      WalkingControllerParameters walkingControllerParameters = wholeBodyControllerParameters.getWalkingControllerParameters();

      walkToLocationPlannedBehavior = new WalkToLocationPlannedBehavior(ros2Node, fullRobotModel, referenceFrames, walkingControllerParameters, yoTime);
      addPrimitive(walkToLocationPlannedBehavior);

      leftArmTrajectoryBehavior = new ArmTrajectoryBehavior("left", ros2Node, yoTime);
      addPrimitive(leftArmTrajectoryBehavior);

      rightArmTrajectoryBehavior = new ArmTrajectoryBehavior("right", ros2Node, yoTime);
      addPrimitive(rightArmTrajectoryBehavior);
      chestTrajectoryBehavior = new ChestTrajectoryBehavior(ros2Node, yoTime);
      addPrimitive(chestTrajectoryBehavior);
      clearLidarBehavior = new ClearLidarBehavior(ros2Node);
      addPrimitive(clearLidarBehavior);
      enableLidarBehavior = new EnableLidarBehavior(ros2Node);
      addPrimitive(enableLidarBehavior);
      leftFootEndEffectorLoadBearingBehavior = new FootLoadBearingBehavior("leftFoot", ros2Node);
      addPrimitive(leftFootEndEffectorLoadBearingBehavior);
      rightFootEndEffectorLoadBearingBehavior = new FootLoadBearingBehavior("rightFoot", ros2Node);
      addPrimitive(rightFootEndEffectorLoadBearingBehavior);
      footstepListBehavior = new FootstepListBehavior(ros2Node, walkingControllerParameters);
      addPrimitive(footstepListBehavior);
      leftArmGoHomeBehavior = new GoHomeBehavior("leftArm", ros2Node, yoTime);
      addPrimitive(leftArmGoHomeBehavior);
      rightArmGoHomeBehavior = new GoHomeBehavior("rightArm", ros2Node, yoTime);
      addPrimitive(rightArmGoHomeBehavior);
      chestGoHomeBehavior = new GoHomeBehavior("chest", ros2Node, yoTime);
      addPrimitive(chestGoHomeBehavior);
      pelvisGoHomeBehavior = new GoHomeBehavior("pelvis", ros2Node, yoTime);
      addPrimitive(pelvisGoHomeBehavior);
      leftHandDesiredConfigurationBehavior = new HandDesiredConfigurationBehavior("leftHand", ros2Node, yoTime);
      addPrimitive(leftHandDesiredConfigurationBehavior);
      rightHandDesiredConfigurationBehavior = new HandDesiredConfigurationBehavior("rigthHand", ros2Node, yoTime);
      addPrimitive(rightHandDesiredConfigurationBehavior);
      leftHandTrajectoryBehavior = new HandTrajectoryBehavior("left", ros2Node, yoTime);
      addPrimitive(leftHandTrajectoryBehavior);
      rightHandTrajectoryBehavior = new HandTrajectoryBehavior("right", ros2Node, yoTime);
      addPrimitive(rightHandTrajectoryBehavior);
      headTrajectoryBehavior = new HeadTrajectoryBehavior("", ros2Node, yoTime);
      addPrimitive(headTrajectoryBehavior);
      pelvisHeightTrajectoryBehavior = new PelvisHeightTrajectoryBehavior(ros2Node, yoTime);
      addPrimitive(pelvisHeightTrajectoryBehavior);
      pelvisOrientationTrajectoryBehavior = new PelvisOrientationTrajectoryBehavior(ros2Node, yoTime);
      addPrimitive(pelvisOrientationTrajectoryBehavior);
      pelvisTrajectoryBehavior = new PelvisTrajectoryBehavior(ros2Node, yoTime);
      addPrimitive(pelvisTrajectoryBehavior);
      setLidarParametersBehavior = new SetLidarParametersBehavior(ros2Node);
      addPrimitive(setLidarParametersBehavior);
      walkToLocationBehavior = new WalkToLocationBehavior(ros2Node, fullRobotModel, referenceFrames, walkingControllerParameters);
      addPrimitive(walkToLocationBehavior);
      wholeBodyBehavior = new WholeBodyInverseKinematicsBehavior("atlas", fullRobotModelFactory, yoTime, ros2Node, fullRobotModel);
      addPrimitive(wholeBodyBehavior);


   }
   private void addPrimitive(AbstractBehavior behavior)
   {
      behaviorRegistry.addChild(behavior.getYoVariableRegistry());
   }



}
