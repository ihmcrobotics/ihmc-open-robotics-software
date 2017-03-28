package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

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

   public AtlasPrimitiveActions(CommunicationBridge outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel,
         HumanoidReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters, DoubleYoVariable yoTime,
         WholeBodyControllerParameters wholeBodyControllerParameters, YoVariableRegistry behaviorRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.behaviorRegistry = behaviorRegistry;

      walkToLocationPlannedBehavior = new WalkToLocationPlannedBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters, yoTime);
      addPrimitive(walkToLocationPlannedBehavior);

      leftArmTrajectoryBehavior = new ArmTrajectoryBehavior("left", outgoingCommunicationBridge, yoTime);
      addPrimitive(leftArmTrajectoryBehavior);

      rightArmTrajectoryBehavior = new ArmTrajectoryBehavior("right", outgoingCommunicationBridge, yoTime);
      addPrimitive(rightArmTrajectoryBehavior);
      chestTrajectoryBehavior = new ChestTrajectoryBehavior(outgoingCommunicationBridge, yoTime);
      addPrimitive(chestTrajectoryBehavior);
      clearLidarBehavior = new ClearLidarBehavior(outgoingCommunicationBridge);
      addPrimitive(clearLidarBehavior);
      enableLidarBehavior = new EnableLidarBehavior(outgoingCommunicationBridge);
      addPrimitive(enableLidarBehavior);
      leftFootEndEffectorLoadBearingBehavior = new FootLoadBearingBehavior("leftFoot", outgoingCommunicationBridge);
      addPrimitive(leftFootEndEffectorLoadBearingBehavior);
      rightFootEndEffectorLoadBearingBehavior = new FootLoadBearingBehavior("rightFoot", outgoingCommunicationBridge);
      addPrimitive(rightFootEndEffectorLoadBearingBehavior);
      footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge, walkingControllerParameters);
      addPrimitive(footstepListBehavior);
      leftArmGoHomeBehavior = new GoHomeBehavior("leftArm", outgoingCommunicationBridge, yoTime);
      addPrimitive(leftArmGoHomeBehavior);
      rightArmGoHomeBehavior = new GoHomeBehavior("rightArm", outgoingCommunicationBridge, yoTime);
      addPrimitive(rightArmGoHomeBehavior);
      chestGoHomeBehavior = new GoHomeBehavior("chest", outgoingCommunicationBridge, yoTime);
      addPrimitive(chestGoHomeBehavior);
      pelvisGoHomeBehavior = new GoHomeBehavior("pelvis", outgoingCommunicationBridge, yoTime);
      addPrimitive(pelvisGoHomeBehavior);
      leftHandDesiredConfigurationBehavior = new HandDesiredConfigurationBehavior("leftHand", outgoingCommunicationBridge, yoTime);
      addPrimitive(leftHandDesiredConfigurationBehavior);
      rightHandDesiredConfigurationBehavior = new HandDesiredConfigurationBehavior("rigthHand", outgoingCommunicationBridge, yoTime);
      addPrimitive(rightHandDesiredConfigurationBehavior);
      leftHandTrajectoryBehavior = new HandTrajectoryBehavior("left", outgoingCommunicationBridge, yoTime);
      addPrimitive(leftHandTrajectoryBehavior);
      rightHandTrajectoryBehavior = new HandTrajectoryBehavior("right", outgoingCommunicationBridge, yoTime);
      addPrimitive(rightHandTrajectoryBehavior);
      headTrajectoryBehavior = new HeadTrajectoryBehavior("", outgoingCommunicationBridge, yoTime);
      addPrimitive(headTrajectoryBehavior);
      pelvisHeightTrajectoryBehavior = new PelvisHeightTrajectoryBehavior(outgoingCommunicationBridge, yoTime);
      addPrimitive(pelvisHeightTrajectoryBehavior);
      pelvisOrientationTrajectoryBehavior = new PelvisOrientationTrajectoryBehavior(outgoingCommunicationBridge, yoTime);
      addPrimitive(pelvisOrientationTrajectoryBehavior);
      pelvisTrajectoryBehavior = new PelvisTrajectoryBehavior(outgoingCommunicationBridge, yoTime);
      addPrimitive(pelvisTrajectoryBehavior);
      setLidarParametersBehavior = new SetLidarParametersBehavior(outgoingCommunicationBridge);
      addPrimitive(setLidarParametersBehavior);
      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      addPrimitive(walkToLocationBehavior);
      wholeBodyBehavior = new WholeBodyInverseKinematicsBehavior("atlas", wholeBodyControllerParameters, yoTime, outgoingCommunicationBridge, fullRobotModel);
      addPrimitive(wholeBodyBehavior);

   }
   private void addPrimitive(AbstractBehavior behavior)
   {
      behaviorRegistry.addChild(behavior.getYoVariableRegistry());
   }



}
