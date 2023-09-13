package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;

public class NaturalPosturePrivilegedConfigurationController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private boolean useSpinePrivilegedCommand;
   private boolean useSpinePitchPrivilegedCommand;

   private final YoDouble pPoseSpineRoll = new YoDouble("pPoseSpineRoll", registry);
   private final YoDouble pPoseSpinePitch = new YoDouble("pPoseSpinePitch", registry);
   private final YoDouble pPoseSpineYaw = new YoDouble("pPoseSpineYaw", registry);

   private final YoPDGains pPoseSpinePitchGains = new YoPDGains("pPoseSpinePitch", registry);
   private final YoPDGains pPoseSpineRollGains = new YoPDGains("pPoseSpineRoll", registry);

   private final YoDouble pPoseShoulderPitch = new YoDouble("pPoseShoulderPitch", registry);
   private final YoDouble pPoseShoulderRoll = new YoDouble("pPoseShoulderRoll", registry);
   private final YoDouble pPoseShoulderYaw = new YoDouble("pPoseShoulderYaw", registry);
   private final YoDouble pPoseElbow = new YoDouble("pPoseElbow", registry);

   private final YoDouble pPoseShoulderKp = new YoDouble("pPoseShoulderKp", registry);
   private final YoDouble pPoseShoulderKdFactor = new YoDouble("pPoseShoulderKdFactor", registry);

   private final YoDouble pPoseShoulderRollWeight = new YoDouble("pPoseShoulderRollWeight", registry);
   private final YoDouble pPoseShoulderPitchWeight = new YoDouble("pPoseShoulderPitchWeight", registry);
   private final YoDouble pPoseShoulderYawWeight = new YoDouble("pPoseShoulderYawWeight", registry);
   private final YoDouble pPoseElbowWeight = new YoDouble("pPoseElbowWeight", registry);
   private final YoDouble pPoseSpineYawWeight = new YoDouble("pPoseSpineYawWeight", registry);

   private final YoDouble pPoseElbowKp = new YoDouble("pPoseElbowKp", registry);
   private final YoDouble pPoseElbowKdFactor = new YoDouble("pPoseElbowKdFactor", registry);

   private final YoDouble npPoseSpineRollPitchKp = new YoDouble("npPoseSpineRollPitchKp", registry);

   private final YoDouble pPoseSpineRollPitchKp = new YoDouble("pPoseSpineRollPitchKp", registry);
   private final YoDouble pPoseSpineRollPitchKdFactor = new YoDouble("pPoseSpineRollPitchKdFactor", registry);

   private final YoDouble pPoseSpineYawKp = new YoDouble("pPoseSpineYawKp", registry);
   private final YoDouble pPoseSpineYawKdFactor = new YoDouble("pPoseSpineYawKdFactor", registry);

   private final YoDouble pPoseHipKp = new YoDouble("pPoseHipKp", registry);
   private final YoDouble pPoseHipKdFactor = new YoDouble("pPoseHipKdFactor", registry);

   private final YoDouble pPoseKneeKp = new YoDouble("pPoseKneeKp", registry);
   private final YoDouble pPoseKneeKdFactor = new YoDouble("pPoseKneeKdFactor", registry);

   private final YoBoolean useSpineRollPitchJointCommands = new YoBoolean("useSpineRollPitchJointCommands", registry);

   private final HashMap<OneDoFJointBasics, OneDoFJointPrivilegedConfigurationParameters> privilegedConfigurationMap = new HashMap<>();

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   private final OneDoFJointFeedbackControlCommand spinePitchCommand = new OneDoFJointFeedbackControlCommand();
   private final OneDoFJointFeedbackControlCommand spineRollCommand = new OneDoFJointFeedbackControlCommand();

   private final FullHumanoidRobotModel fullRobotModel;

   public NaturalPosturePrivilegedConfigurationController(FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;

      // privileged configuration for upper body
      useSpinePrivilegedCommand = true;
      useSpinePitchPrivilegedCommand = true;

      pPoseSpineRoll.set(0.0);
      pPoseSpinePitch.set(0.0);
      pPoseSpineYaw.set(0.0);
      pPoseShoulderPitch.set(0.1); //0.1 //0.2   // This is the first joint from the body. The more negative this number is, the more forward the left arm is
      pPoseShoulderRoll.set(0); // the smaller, the further away the arm is from the body   // start at -1 for hardware experiment to be safe
      pPoseShoulderYaw.set(0);
      pPoseElbow.set(-0.2); // 0 looks more natural, but -0.3 might avoid arm from colliding into the hydraulic manifold. //-0.5 //-1   // the smaller, the more bent the elbow is

      pPoseSpineYawWeight.set(1.0); // weight used to complete with other privileged joint position. Other joint default weights are 1
      pPoseShoulderRollWeight.set(1.0);
      pPoseShoulderPitchWeight.set(1.0);
      pPoseShoulderYawWeight.set(1.0); // this weight doesn't matter much
      pPoseElbowWeight.set(1.0);

      useSpineRollPitchJointCommands.set(true); // Can turn off joint limit for the spine when this is true.
      if (useSpineRollPitchJointCommands.getBooleanValue())
      {
         npPoseSpineRollPitchKp.set(100);

         pPoseSpinePitchGains.setKp(npPoseSpineRollPitchKp.getDoubleValue()); //25
         pPoseSpineRollGains.setKp(npPoseSpineRollPitchKp.getDoubleValue()); //25
         pPoseSpinePitchGains.setZeta(0.7);
         pPoseSpineRollGains.setZeta(0.7);
         pPoseSpinePitchGains.createDerivativeGainUpdater(true);
         pPoseSpineRollGains.createDerivativeGainUpdater(true);
      }

      pPoseSpineRollPitchKp.set(50.0);
      pPoseSpineRollPitchKdFactor.set(0.15);

      pPoseSpineYawKp.set(300.0);
      pPoseSpineYawKdFactor.set(0.15);

      pPoseShoulderKp.set(80.0);
      pPoseShoulderKdFactor.set(0.15);

      pPoseElbowKp.set(30.0);
      pPoseElbowKdFactor.set(0.15);

      // privileged configuration for lower body
      pPoseHipKp.set(100);
      pPoseHipKdFactor.set(0.2);

      pPoseKneeKp.set(100);
      pPoseKneeKdFactor.set(0.2);

      OneDoFJointBasics spineRoll = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);
      OneDoFJointBasics spinePitch = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);

      spinePitchCommand.clear();
      spinePitchCommand.setJoint(spinePitch);

      spineRollCommand.clear();
      spineRollCommand.setJoint(spineRoll);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {

      updatePrivilegedConfigurationCommand();
   }

   public void compute()
   {
      feedbackControlCommandList.clear();

      // Testing -- track spine joint x and y with highest priority
      if (useSpineRollPitchJointCommands.getBooleanValue())
      {
         pPoseSpinePitchGains.setKp(npPoseSpineRollPitchKp.getDoubleValue());
         pPoseSpineRollGains.setKp(npPoseSpineRollPitchKp.getDoubleValue());

         OneDoFJointBasics spineRoll = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);
         OneDoFJointBasics spinePitch = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);
         spinePitchCommand.setJoint(spinePitch);
         spinePitchCommand.setInverseDynamics(0.0, 0.0, 0.0);
         spinePitchCommand.setGains(pPoseSpinePitchGains);

         spineRollCommand.setJoint(spineRoll);
         spineRollCommand.setInverseDynamics(0.0, 0.0, 0.0);
         spineRollCommand.setGains(pPoseSpineRollGains);

         feedbackControlCommandList.addCommand(spinePitchCommand);
         feedbackControlCommandList.addCommand(spineRollCommand);
      }

      updatePrivilegedConfigurationCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackControlCommandList;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return privilegedConfigurationCommand;
   }

   private void updatePrivilegedConfigurationCommand()
   {
      privilegedConfigurationCommand.clear();
      privilegedConfigurationCommand.enable();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationCommand.PrivilegedConfigurationOption.AT_ZERO);

      //TODO: This is hardcoded here. It should be moved to a parameter setting instead. This is not the long term place for it.
      if (useSpinePrivilegedCommand)
      {
         spineYawPrivilegedConfigurationParameters();
      }

      for (RobotSide side : RobotSide.values)
      {
         createAndAddJointPrivilegedConfigurationParameters(side,
                                                            ArmJointName.SHOULDER_PITCH,
                                                            side.negateIfRightSide(pPoseShoulderPitch.getDoubleValue()),
                                                            pPoseShoulderPitchWeight.getDoubleValue(),
                                                            pPoseShoulderKp.getDoubleValue(),
                                                            pPoseShoulderKdFactor.getDoubleValue() * pPoseShoulderKp.getDoubleValue());
         createAndAddJointPrivilegedConfigurationParameters(side,
                                                            ArmJointName.SHOULDER_ROLL,
                                                            pPoseShoulderRoll.getDoubleValue(),
                                                            pPoseShoulderRollWeight.getDoubleValue(),
                                                            pPoseShoulderKp.getDoubleValue(),
                                                            pPoseShoulderKdFactor.getDoubleValue() * pPoseShoulderKp.getDoubleValue());
         createAndAddJointPrivilegedConfigurationParameters(side,
                                                            ArmJointName.SHOULDER_YAW,
                                                            pPoseShoulderYaw.getDoubleValue(),
                                                            pPoseShoulderYawWeight.getDoubleValue(),
                                                            pPoseShoulderKp.getDoubleValue(),
                                                            pPoseShoulderKdFactor.getDoubleValue() * pPoseShoulderKp.getDoubleValue());
         createAndAddJointPrivilegedConfigurationParameters(side,
                                                            ArmJointName.ELBOW_PITCH,
                                                            side.negateIfRightSide(pPoseElbow.getDoubleValue()),
                                                            pPoseElbowWeight.getDoubleValue(),
                                                            pPoseElbowKp.getDoubleValue(),
                                                            pPoseElbowKdFactor.getDoubleValue() * pPoseElbowKp.getDoubleValue());

         createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.WRIST_YAW, 0.0);
         createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.WRIST_ROLL, 0.0);
         createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.FIRST_WRIST_PITCH, 0.0);

         createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.ANKLE_ROLL, 0.0, 4.0, 0.6);
         createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.ANKLE_PITCH, 0.0, 4.0, 0.6);
      }
   }

   private OneDoFJointPrivilegedConfigurationParameters spineRollPrivilegedConfigurationParameters()
   {
      return createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL),
                                                                pPoseSpineRoll.getDoubleValue(),
                                                                1.0,
                                                                pPoseSpineRollPitchKp.getDoubleValue(),
                                                                pPoseSpineRollPitchKp.getDoubleValue() * pPoseSpineRollPitchKdFactor.getValue());
   }

   private OneDoFJointPrivilegedConfigurationParameters spinePitchPrivilegedConfigurationParameters()
   {

      return createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH),
                                                                pPoseSpinePitch.getDoubleValue(),
                                                                1.0,
                                                                pPoseSpineRollPitchKp.getDoubleValue(),
                                                                pPoseSpineRollPitchKp.getDoubleValue() * pPoseSpineRollPitchKdFactor.getValue());
   }

   private OneDoFJointPrivilegedConfigurationParameters spineYawPrivilegedConfigurationParameters()
   {
      return createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW),
                                                                pPoseSpineYaw.getDoubleValue(),
                                                                pPoseSpineYawWeight.getDoubleValue(),
                                                                pPoseSpineYawKp.getDoubleValue(),
                                                                pPoseSpineYawKp.getDoubleValue() * pPoseSpineYawKdFactor.getValue());
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide,
                                                                                                           ArmJointName armJointName,
                                                                                                           double privilegedAngle)
   {
      return createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getArmJoint(robotSide, armJointName), privilegedAngle, 1.0, 40.0, 6.0);
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide,
                                                                                                           ArmJointName armJointName,
                                                                                                           double privilegedAngle,
                                                                                                           double weight,
                                                                                                           double pgain,
                                                                                                           double dgain)
   {
      return createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getArmJoint(robotSide, armJointName), privilegedAngle, weight, pgain, dgain);
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide,
                                                                                                           ArmJointName armJointName,
                                                                                                           double privilegedAngle,
                                                                                                           double pgain,
                                                                                                           double dgain)
   {
      return createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getArmJoint(robotSide, armJointName), privilegedAngle, 1.0, pgain, dgain);
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide,
                                                                                                           LegJointName legJointName,
                                                                                                           double privilegedAngle,
                                                                                                           double pgain,
                                                                                                           double dgain)
   {
      return createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getLegJoint(robotSide, legJointName), privilegedAngle, 1.0, pgain, dgain);
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(OneDoFJointBasics joint,
                                                                                                           double privilegedAngle,
                                                                                                           double weight,
                                                                                                           double pgain,
                                                                                                           double dgain)
   {
      OneDoFJointPrivilegedConfigurationParameters jointParameters = privilegedConfigurationMap.get(joint);
      if (jointParameters == null)
      {
         jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
         privilegedConfigurationMap.put(joint, jointParameters);
      }

      jointParameters.setConfigurationGain(pgain);
      jointParameters.setVelocityGain(dgain);
      jointParameters.setWeight(weight);
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(privilegedAngle);

      privilegedConfigurationCommand.addJoint(joint, jointParameters);

      return jointParameters;
   }
}
