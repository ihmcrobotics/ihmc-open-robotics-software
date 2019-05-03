package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
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

   public AtlasPrimitiveActions(String robotName, Ros2Node ros2Node, FootstepPlannerParameters footstepPlannerParameters,
                                FullHumanoidRobotModel fullRobotModel, FullHumanoidRobotModelFactory fullRobotModelFactory, HumanoidReferenceFrames referenceFrames,
                                YoDouble yoTime, WholeBodyControllerParameters wholeBodyControllerParameters, YoVariableRegistry behaviorRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.behaviorRegistry = behaviorRegistry;

      WalkingControllerParameters walkingControllerParameters = wholeBodyControllerParameters.getWalkingControllerParameters();

      walkToLocationPlannedBehavior = new WalkToLocationPlannedBehavior(robotName, ros2Node, fullRobotModel, referenceFrames, walkingControllerParameters,footstepPlannerParameters, yoTime);
      addPrimitive(walkToLocationPlannedBehavior);

      leftArmTrajectoryBehavior = new ArmTrajectoryBehavior(robotName, "left", ros2Node, yoTime);
      addPrimitive(leftArmTrajectoryBehavior);

      rightArmTrajectoryBehavior = new ArmTrajectoryBehavior(robotName, "right", ros2Node, yoTime);
      addPrimitive(rightArmTrajectoryBehavior);
      chestTrajectoryBehavior = new ChestTrajectoryBehavior(robotName, ros2Node, yoTime);
      addPrimitive(chestTrajectoryBehavior);
      clearLidarBehavior = new ClearLidarBehavior(robotName, ros2Node);
      addPrimitive(clearLidarBehavior);
      enableLidarBehavior = new EnableLidarBehavior(robotName, ros2Node);
      addPrimitive(enableLidarBehavior);
      leftFootEndEffectorLoadBearingBehavior = new FootLoadBearingBehavior(robotName, "leftFoot", ros2Node);
      addPrimitive(leftFootEndEffectorLoadBearingBehavior);
      rightFootEndEffectorLoadBearingBehavior = new FootLoadBearingBehavior(robotName, "rightFoot", ros2Node);
      addPrimitive(rightFootEndEffectorLoadBearingBehavior);
      footstepListBehavior = new FootstepListBehavior(robotName, ros2Node, walkingControllerParameters);
      addPrimitive(footstepListBehavior);
      leftArmGoHomeBehavior = new GoHomeBehavior(robotName, "leftArm", ros2Node, yoTime);
      addPrimitive(leftArmGoHomeBehavior);
      rightArmGoHomeBehavior = new GoHomeBehavior(robotName, "rightArm", ros2Node, yoTime);
      addPrimitive(rightArmGoHomeBehavior);
      chestGoHomeBehavior = new GoHomeBehavior(robotName, "chest", ros2Node, yoTime);
      addPrimitive(chestGoHomeBehavior);
      pelvisGoHomeBehavior = new GoHomeBehavior(robotName, "pelvis", ros2Node, yoTime);
      addPrimitive(pelvisGoHomeBehavior);
      leftHandDesiredConfigurationBehavior = new HandDesiredConfigurationBehavior(robotName, "leftHand", ros2Node, yoTime);
      addPrimitive(leftHandDesiredConfigurationBehavior);
      rightHandDesiredConfigurationBehavior = new HandDesiredConfigurationBehavior(robotName, "rigthHand", ros2Node, yoTime);
      addPrimitive(rightHandDesiredConfigurationBehavior);
      leftHandTrajectoryBehavior = new HandTrajectoryBehavior(robotName, "left", ros2Node, yoTime);
      addPrimitive(leftHandTrajectoryBehavior);
      rightHandTrajectoryBehavior = new HandTrajectoryBehavior(robotName, "right", ros2Node, yoTime);
      addPrimitive(rightHandTrajectoryBehavior);
      headTrajectoryBehavior = new HeadTrajectoryBehavior(robotName, "", ros2Node, yoTime);
      addPrimitive(headTrajectoryBehavior);
      pelvisHeightTrajectoryBehavior = new PelvisHeightTrajectoryBehavior(robotName, ros2Node, yoTime);
      addPrimitive(pelvisHeightTrajectoryBehavior);
      pelvisOrientationTrajectoryBehavior = new PelvisOrientationTrajectoryBehavior(robotName, ros2Node, yoTime);
      addPrimitive(pelvisOrientationTrajectoryBehavior);
      pelvisTrajectoryBehavior = new PelvisTrajectoryBehavior(robotName, ros2Node, yoTime);
      addPrimitive(pelvisTrajectoryBehavior);
      setLidarParametersBehavior = new SetLidarParametersBehavior(robotName, ros2Node);
      addPrimitive(setLidarParametersBehavior);
      walkToLocationBehavior = new WalkToLocationBehavior(robotName, ros2Node, fullRobotModel, referenceFrames, walkingControllerParameters);
      addPrimitive(walkToLocationBehavior);
      wholeBodyBehavior = new WholeBodyInverseKinematicsBehavior(robotName, "atlas", fullRobotModelFactory, yoTime, ros2Node, fullRobotModel);
      addPrimitive(wholeBodyBehavior);


   }
   private void addPrimitive(AbstractBehavior behavior)
   {
      behaviorRegistry.addChild(behavior.getYoVariableRegistry());
   }



}
