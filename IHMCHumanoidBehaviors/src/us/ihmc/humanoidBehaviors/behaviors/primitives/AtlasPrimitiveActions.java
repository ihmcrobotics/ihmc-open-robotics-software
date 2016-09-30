package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class AtlasPrimitiveActions
{
   //ALL Primitive Behaviors Go In Here. Talk To John Carff before chaning this.
   public final ArmTrajectoryBehavior leftArmTrajectoryBehavior;
   private final ArmTrajectoryBehavior rightArmTrajectoryBehavior;
   private final ChestTrajectoryBehavior chestTrajectoryBehavior;
   private final ClearLidarBehavior clearLidarBehavior;
   private final EnableLidarBehavior enableLidarBehavior;
   private final EndEffectorLoadBearingBehavior leftFootEndEffectorLoadBearingBehavior;
   private final EndEffectorLoadBearingBehavior rightFootEndEffectorLoadBearingBehavior;
   private final EndEffectorLoadBearingBehavior leftHandEndEffectorLoadBearingBehavior;
   private final EndEffectorLoadBearingBehavior rightHandEndEffectorLoadBearingBehavior;
   private final FootstepListBehavior footstepListBehavior;
   private final GoHomeBehavior leftArmGoHomeBehavior;
   private final GoHomeBehavior rightArmGoHomeBehavior;
   private final GoHomeBehavior chestGoHomeBehavior;
   private final GoHomeBehavior pelvisGoHomeBehavior;
   private final HandDesiredConfigurationBehavior leftHandDesiredConfigurationBehavior;
   private final HandDesiredConfigurationBehavior rightHandDesiredConfigurationBehavior;
   private final HandTrajectoryBehavior leftHandTrajectoryBehavior;
   private final HandTrajectoryBehavior rightHandTrajectoryBehavior;
   private final HeadTrajectoryBehavior headTrajectoryBehavior;
   private final PelvisHeightTrajectoryBehavior pelvisHeightTrajectoryBehavior;
   private final PelvisOrientationTrajectoryBehavior pelvisOrientationTrajectoryBehavior;
   private final PelvisTrajectoryBehavior pelvisTrajectoryBehavior;
   private final SetLidarParametersBehavior setLidarParametersBehavior;

   public AtlasPrimitiveActions(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, WalkingControllerParameters walkingControllerParameters,
         DoubleYoVariable yoTime, YoVariableRegistry behaviorRegistry)
   {
      
      leftArmTrajectoryBehavior = new ArmTrajectoryBehavior("left", outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(leftArmTrajectoryBehavior.getYoVariableRegistry());

      rightArmTrajectoryBehavior = new ArmTrajectoryBehavior("right", outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(rightArmTrajectoryBehavior.getYoVariableRegistry());
      chestTrajectoryBehavior = new ChestTrajectoryBehavior(outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(chestTrajectoryBehavior.getYoVariableRegistry());
      clearLidarBehavior = new ClearLidarBehavior(outgoingCommunicationBridge);
      behaviorRegistry.addChild(clearLidarBehavior.getYoVariableRegistry());
      enableLidarBehavior = new EnableLidarBehavior(outgoingCommunicationBridge);
      behaviorRegistry.addChild(enableLidarBehavior.getYoVariableRegistry());
      leftFootEndEffectorLoadBearingBehavior = new EndEffectorLoadBearingBehavior("leftFoot", outgoingCommunicationBridge);
      behaviorRegistry.addChild(leftFootEndEffectorLoadBearingBehavior.getYoVariableRegistry());
      rightFootEndEffectorLoadBearingBehavior = new EndEffectorLoadBearingBehavior("rightFoot", outgoingCommunicationBridge);
      behaviorRegistry.addChild(rightFootEndEffectorLoadBearingBehavior.getYoVariableRegistry());
      leftHandEndEffectorLoadBearingBehavior = new EndEffectorLoadBearingBehavior("leftHand", outgoingCommunicationBridge);
      behaviorRegistry.addChild(leftHandEndEffectorLoadBearingBehavior.getYoVariableRegistry());
      rightHandEndEffectorLoadBearingBehavior = new EndEffectorLoadBearingBehavior("rightHand", outgoingCommunicationBridge);
      behaviorRegistry.addChild(rightHandEndEffectorLoadBearingBehavior.getYoVariableRegistry());
      footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge, walkingControllerParameters);
      behaviorRegistry.addChild(footstepListBehavior.getYoVariableRegistry());
      leftArmGoHomeBehavior = new GoHomeBehavior("leftArm", outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(leftArmGoHomeBehavior.getYoVariableRegistry());
      rightArmGoHomeBehavior = new GoHomeBehavior("rightArm", outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(rightArmGoHomeBehavior.getYoVariableRegistry());
      chestGoHomeBehavior = new GoHomeBehavior("chest", outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(chestGoHomeBehavior.getYoVariableRegistry());
      pelvisGoHomeBehavior = new GoHomeBehavior("pelvis", outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(pelvisGoHomeBehavior.getYoVariableRegistry());
      leftHandDesiredConfigurationBehavior = new HandDesiredConfigurationBehavior(outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(leftHandDesiredConfigurationBehavior.getYoVariableRegistry());
      rightHandDesiredConfigurationBehavior = new HandDesiredConfigurationBehavior(outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(rightHandDesiredConfigurationBehavior.getYoVariableRegistry());
      leftHandTrajectoryBehavior = new HandTrajectoryBehavior("left", outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(leftHandTrajectoryBehavior.getYoVariableRegistry());
      rightHandTrajectoryBehavior = new HandTrajectoryBehavior("right", outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(rightHandTrajectoryBehavior.getYoVariableRegistry());
      headTrajectoryBehavior = new HeadTrajectoryBehavior("", outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(headTrajectoryBehavior.getYoVariableRegistry());
      pelvisHeightTrajectoryBehavior = new PelvisHeightTrajectoryBehavior(outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(pelvisHeightTrajectoryBehavior.getYoVariableRegistry());
      pelvisOrientationTrajectoryBehavior = new PelvisOrientationTrajectoryBehavior(outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(pelvisOrientationTrajectoryBehavior.getYoVariableRegistry());
      pelvisTrajectoryBehavior = new PelvisTrajectoryBehavior(outgoingCommunicationBridge, yoTime);
      behaviorRegistry.addChild(pelvisTrajectoryBehavior.getYoVariableRegistry());
      setLidarParametersBehavior = new SetLidarParametersBehavior(outgoingCommunicationBridge);
      behaviorRegistry.addChild(setLidarParametersBehavior.getYoVariableRegistry());

      
   }

}
