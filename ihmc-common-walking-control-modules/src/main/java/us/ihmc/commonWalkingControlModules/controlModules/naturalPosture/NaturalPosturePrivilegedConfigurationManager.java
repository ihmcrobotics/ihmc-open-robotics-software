package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
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

public class NaturalPosturePrivilegedConfigurationManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private boolean useSpinePrivilegedCommand;
   private boolean useSpinePitchPrivilegedCommand;

   private final YoDouble pPoseSpineRoll = new YoDouble("pPoseSpineRoll", registry);
   private final YoDouble pPoseSpinePitch = new YoDouble("pPoseSpinePitch", registry);
   private final YoDouble pPoseSpineYaw = new YoDouble("pPoseSpineYaw", registry);
   private final YoDouble pPoseSpineRollKp = new YoDouble("pPoseSpineRollKp", registry);
   private final YoDouble pPoseSpineRollKdFactor = new YoDouble("pPoseSpineRollKdFactor", registry);

   private final YoDouble pPoseSpinePitchKp = new YoDouble("pPoseSpinePitchKp", registry);
   private final YoDouble pPoseSpinePitchKdFactor = new YoDouble("pPoseSpinePitchKdFactor", registry);
   private final YoDouble pPoseSpineYawKp = new YoDouble("pPoseSpineYawKp", registry);
   private final YoDouble pPoseSpineYawKdFactor = new YoDouble("pPoseSpineYawKdFactor", registry);
   private final YoDouble pPoseShoulderPitch = new YoDouble("pPoseShoulderPitch", registry);
   private final YoDouble pPoseShoulderRoll = new YoDouble("pPoseShoulderRoll", registry);
   private final YoDouble pPoseShoulderYaw = new YoDouble("pPoseShoulderYaw", registry);
   private final YoDouble pPoseElbow = new YoDouble("pPoseElbow", registry);
   private final YoDouble pPoseShoulderPitchKp = new YoDouble("pPoseShoulderPitchKp", registry);
   private final YoDouble pPoseShoulderRollKp = new YoDouble("pPoseShoulderRollKp", registry);
   private final YoDouble pPoseShoulderYawKp = new YoDouble("pPoseShoulderYawKp", registry);
   private final YoDouble pPoseElbowWeight = new YoDouble("pPoseElbowWeight", registry);
   private final YoDouble pPoseElbowKp = new YoDouble("pPoseElbowKp", registry);
   private final YoDouble pPoseShoulderPitchKdFactor = new YoDouble("pPoseShoulderPitchKdFactor", registry);
   private final YoDouble pPoseShoulderRollKdFactor = new YoDouble("pPoseShoulderRollKdFactor", registry);
   private final YoDouble pPoseShoulderYawKdFactor = new YoDouble("pPoseShoulderYawKdFactor", registry);
   private final YoDouble pPoseElbowKdFactor = new YoDouble("pPoseElbowKdFactor", registry);

   private final YoDouble pPoseSpineYawWeight = new YoDouble("pPoseSpineYawWeight", registry);
   private final YoDouble pPoseShoulderYawWeight = new YoDouble("pPoseShoulderYawWeight", registry);

   //   private final YoDouble pPoseHipPitch = new YoDouble("pPoseHipPitch", registry);
   private final YoDouble pPoseHipPitchKp = new YoDouble("pPoseHipPitchKp", registry);
   private final YoDouble pPoseHipPitchKdFactor = new YoDouble("pPoseHipPitchKdFactor", registry);
   //   private final YoDouble pPoseHipRoll = new YoDouble("pPoseHipRoll", registry);
   private final YoDouble pPoseHipRollKp = new YoDouble("pPoseHipRollKp", registry);
   private final YoDouble pPoseHipRollKdFactor = new YoDouble("pPoseHipRollKdFactor", registry);
   //   private final YoDouble pPoseHipYaw = new YoDouble("pPoseHipYaw", registry);
   private final YoDouble pPoseHipYawKp = new YoDouble("pPoseHipYawKp", registry);
   private final YoDouble pPoseHipYawKdFactor = new YoDouble("pPoseHipYawKdFactor", registry);
   //   private final YoDouble pPoseKnee = new YoDouble("pPoseKnee", registry);
   private final YoDouble pPoseKneeKp = new YoDouble("pPoseKneeKp", registry);
   private final YoDouble pPoseKneeKdFactor = new YoDouble("pPoseKneeKdFactor", registry);

   private final String spineRollJointName = "SPINE_X";
   private final String spinePitchJointName = "SPINE_Y";
   private final String spineYawJointName = "SPINE_Z";



   private final YoPDGains pPoseSpinePitchGains = new YoPDGains("pPoseSpinePitch", registry);
   private final YoPDGains pPoseSpineRollGains = new YoPDGains("pPoseSpineRoll", registry);

   private final YoBoolean useSpineRollPitchJointCommands = new YoBoolean("useSpineRollPitchJointCommands", registry);

   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();

   private final OneDoFJointFeedbackControlCommand spinePitchCommand = new OneDoFJointFeedbackControlCommand();
   private final OneDoFJointFeedbackControlCommand spineRollCommand = new OneDoFJointFeedbackControlCommand();

   private final FullHumanoidRobotModel fullRobotModel;

   public NaturalPosturePrivilegedConfigurationManager(FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;

      OneDoFJointBasics spineRoll = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);
      OneDoFJointBasics spinePitch = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);

      spinePitchCommand.clear();
      spinePitchCommand.setJoint(spinePitch);

      spineRollCommand.clear();
      spineRollCommand.setJoint(spineRoll);

      useSpineRollPitchJointCommands.set(true); // Can turn off joint limit for the spine when this is true.
      if (useSpineRollPitchJointCommands.getBooleanValue())
      {
         pPoseSpinePitchGains.setKp(25.0);
         pPoseSpineRollGains.setKp(25.0);
         pPoseSpinePitchGains.setZeta(0.7);
         pPoseSpineRollGains.setZeta(0.7);
         pPoseSpinePitchGains.createDerivativeGainUpdater(true);
         pPoseSpineRollGains.createDerivativeGainUpdater(true);
      }


      // privileged configuration for upper body
      useSpinePrivilegedCommand = true;
      useSpinePitchPrivilegedCommand = true;

      pPoseSpineRoll.set(0.0);
      pPoseSpinePitch.set(0.0);
      pPoseSpineYaw.set(0.0);
      double delta = 0.0;
      pPoseShoulderPitch.set(0 + delta); //0.1 //0.2   // the bigger, the further away the arm is from the body
      pPoseShoulderRoll.set(0 - 1); // the smaller, the further away the arm is from the body   // start at -1 for hardware experiment to be safe
      pPoseShoulderYaw.set(0);
      pPoseElbow.set(-0.4); //-0.5 //-1   // the smaller, the more bent the elbow is

      pPoseSpineYawWeight.set(5.0); // weight used to complete with other privileged joint position. Other joint default weights are 1
      pPoseShoulderYawWeight.set(1.0); // this weight doesn't matter much

      pPoseSpineRollKp.set(50.0);
      pPoseSpinePitchKp.set(50.0);
      pPoseSpineYawKp.set(300.0);
      pPoseShoulderPitchKp.set(80.0);
      pPoseShoulderRollKp.set(80.0);
      pPoseShoulderYawKp.set(80.0);
      pPoseElbowKp.set(30.0);
      pPoseElbowWeight.set(10.0);


      pPoseSpineRollKdFactor.set(0.15);
      pPoseSpinePitchKdFactor.set(0.15);
      pPoseSpineYawKdFactor.set(0.15);
      pPoseShoulderPitchKdFactor.set(0.15);
      pPoseShoulderRollKdFactor.set(0.15);
      pPoseShoulderYawKdFactor.set(0.15);
      pPoseElbowKdFactor.set(0.15);

      // privileged configuration for lower body
      pPoseHipPitchKp.set(100);
      pPoseHipPitchKdFactor.set(0.2);
      pPoseHipRollKp.set(100);
      pPoseHipRollKdFactor.set(0.2);
      pPoseHipYawKp.set(100);
      pPoseHipYawKdFactor.set(0.2);
      pPoseKneeKp.set(100);
      pPoseKneeKdFactor.set(0.2);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      updatePrivilegedConfigurationCommand();
   }

   public void compute()
   {
      feedbackControlCommandList.clear();
      inverseDynamicsCommandList.clear();

      computeSpineControlCommands();
      updatePrivilegedConfigurationCommand();

      feedbackControlCommandList.addCommand(spinePitchCommand);
      feedbackControlCommandList.addCommand(spineRollCommand);

      inverseDynamicsCommandList.addCommand(privilegedConfigurationCommand);
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackControlCommandList;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return inverseDynamicsCommandList;
   }

   private void computeSpineControlCommands()
   {
      // Testing -- track spine joint x and y with highest priority
      if (useSpineRollPitchJointCommands.getBooleanValue())
      {
         OneDoFJointBasics spineRoll = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);
         OneDoFJointBasics spinePitch = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);
         spinePitchCommand.setJoint(spinePitch);
         spinePitchCommand.setInverseDynamics(0.0, 0.0, 0.0);
         spinePitchCommand.setGains(pPoseSpinePitchGains);

         spineRollCommand.setJoint(spineRoll);
         spineRollCommand.setInverseDynamics(0.0, 0.0, 0.0);
         spineRollCommand.setGains(pPoseSpineRollGains);
      }

   }

   private void updatePrivilegedConfigurationCommand()
   {
      privilegedConfigurationCommand.clear();
      privilegedConfigurationCommand.enable();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationCommand.PrivilegedConfigurationOption.AT_ZERO);

      //TODO: This is hardcoded here. It should be moved to a parameter setting instead. This is not the long term place for it.
      if (useSpinePrivilegedCommand)
      {
         //         spineRollPrivilegedConfigurationParameters();
         //         if (useSpinePitchPrivilegedCommand)
         //            spinePitchPrivilegedConfigurationParameters();
         spineYawPrivilegedConfigurationParameters();
      }

      for (RobotSide side : RobotSide.values)
      {
         createAndAddJointPrivilegedConfigurationParameters(side,
                                                            ArmJointName.SHOULDER_PITCH,
                                                            side.negateIfRightSide(pPoseShoulderPitch.getDoubleValue()),
                                                            pPoseShoulderPitchKp.getDoubleValue(),
                                                            pPoseShoulderPitchKdFactor.getDoubleValue() * pPoseShoulderPitchKp.getDoubleValue());
         createAndAddJointPrivilegedConfigurationParameters(side,
                                                            ArmJointName.SHOULDER_ROLL,
                                                            pPoseShoulderRoll.getDoubleValue(),
                                                            pPoseShoulderRollKp.getDoubleValue(),
                                                            pPoseShoulderRollKdFactor.getDoubleValue() * pPoseShoulderRollKp.getDoubleValue());
         createAndAddJointPrivilegedConfigurationParameters(side,
                                                            ArmJointName.SHOULDER_YAW,
                                                            pPoseShoulderYaw.getDoubleValue(),
                                                            pPoseShoulderYawWeight.getDoubleValue(),
                                                            pPoseShoulderYawKp.getDoubleValue(),
                                                            pPoseShoulderYawKdFactor.getDoubleValue() * pPoseShoulderYawKp.getDoubleValue());
         createAndAddJointPrivilegedConfigurationParameters(side,
                                                            ArmJointName.ELBOW_PITCH,
                                                            side.negateIfRightSide(pPoseElbow.getDoubleValue()),
                                                            pPoseElbowWeight.getDoubleValue(),
                                                            pPoseElbowKp.getDoubleValue(),
                                                            pPoseElbowKdFactor.getDoubleValue() * pPoseElbowKp.getDoubleValue());
      }


      createAndAddJointPrivilegedConfigurationParameters(RobotSide.LEFT, ArmJointName.WRIST_YAW, 0.0);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.RIGHT, ArmJointName.WRIST_YAW, 0.0);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.LEFT, ArmJointName.WRIST_ROLL, 0.0);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.RIGHT, ArmJointName.WRIST_ROLL, 0.0);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.LEFT, ArmJointName.FIRST_WRIST_PITCH, 0.0);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.RIGHT, ArmJointName.FIRST_WRIST_PITCH, 0.0);

      // GMN: Probably should have privileged configuration set for entire robot
      //      for (RobotSide robotSide : RobotSide.values)
      //      {
      //         OneDoFJointBasics kneeJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH);
      //
      //         privilegedConfigurationCommand.addJoint(kneeJoint, walkingControllerParameters.getKneePrivilegedConfigurationParameters());
      //      }

      //      side = RobotSide.LEFT;
      //      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_PITCH,
      //                                                         pPoseHipPitch.getDoubleValue(),
      //                                                         pPoseHipPitchKp.getDoubleValue(),
      //                                                         pPoseHipPitchKdFactor.getDoubleValue()*pPoseHipPitchKp.getDoubleValue());
      //      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_ROLL,
      //                                                         pPoseHipRoll.getDoubleValue(),
      //                                                         pPoseHipRollKp.getDoubleValue(),
      //                                                         pPoseHipRollKdFactor.getDoubleValue()*pPoseHipRollKp.getDoubleValue());
      //      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_YAW,
      //                                                         pPoseHipYaw.getDoubleValue(),
      //                                                         pPoseHipYawKp.getDoubleValue(),
      //                                                         pPoseHipYawKdFactor.getDoubleValue()*pPoseHipYawKp.getDoubleValue());
      //      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.KNEE_PITCH,
      //                                                         pPoseKnee.getDoubleValue(),
      //                                                         pPoseKneeKp.getDoubleValue(),
      //                                                         pPoseKneeKdFactor.getDoubleValue()*pPoseKneeKp.getDoubleValue());
      //
      //      side = RobotSide.RIGHT;
      //      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_PITCH,
      //                                                         -pPoseHipPitch.getDoubleValue(),
      //                                                         pPoseHipPitchKp.getDoubleValue(),
      //                                                         pPoseHipPitchKdFactor.getDoubleValue()*pPoseHipPitchKp.getDoubleValue());
      //      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_ROLL,
      //                                                         -pPoseHipRoll.getDoubleValue(),
      //                                                         pPoseHipRollKp.getDoubleValue(),
      //                                                         pPoseHipRollKdFactor.getDoubleValue()*pPoseHipRollKp.getDoubleValue());
      //      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_YAW,
      //                                                         -pPoseHipYaw.getDoubleValue(),
      //                                                         pPoseHipYawKp.getDoubleValue(),
      //                                                         pPoseHipYawKdFactor.getDoubleValue()*pPoseHipYawKp.getDoubleValue());
      //      createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.KNEE_PITCH,
      //                                                         -pPoseKnee.getDoubleValue(),
      //                                                         pPoseKneeKp.getDoubleValue(),
      //                                                         pPoseKneeKdFactor.getDoubleValue()*pPoseKneeKp.getDoubleValue());

      for (RobotSide robotSide : RobotSide.values)
      {
         createAndAddJointPrivilegedConfigurationParameters(robotSide,
                                                            LegJointName.HIP_PITCH,
                                                            -0.25,
                                                            pPoseHipPitchKp.getDoubleValue(),
                                                            pPoseHipPitchKdFactor.getDoubleValue() * pPoseHipPitchKp.getDoubleValue());
         createAndAddJointPrivilegedConfigurationParameters(robotSide,
                                                            LegJointName.HIP_ROLL,
                                                            0.0,
                                                            pPoseHipRollKp.getDoubleValue(),
                                                            pPoseHipRollKdFactor.getDoubleValue() * pPoseHipRollKp.getDoubleValue());
         createAndAddJointPrivilegedConfigurationParameters(robotSide,
                                                            LegJointName.HIP_YAW,
                                                            0.0,
                                                            pPoseHipYawKp.getDoubleValue(),
                                                            pPoseHipYawKdFactor.getDoubleValue() * pPoseHipYawKp.getDoubleValue());
         createAndAddJointPrivilegedConfigurationParameters(robotSide,
                                                            LegJointName.KNEE_PITCH,
                                                            0.5,
                                                            pPoseKneeKp.getDoubleValue(),
                                                            pPoseKneeKdFactor.getDoubleValue() * pPoseKneeKp.getDoubleValue());
      }

      createAndAddJointPrivilegedConfigurationParameters(RobotSide.LEFT, LegJointName.ANKLE_ROLL, 0.0, 4.0, 0.6);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.RIGHT, LegJointName.ANKLE_ROLL, 0.0, 4.0, 0.6);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.LEFT, LegJointName.ANKLE_PITCH, 0.0, 4.0, 0.6);
      createAndAddJointPrivilegedConfigurationParameters(RobotSide.RIGHT, LegJointName.ANKLE_PITCH, 0.0, 4.0, 0.6);
   }

   private OneDoFJointPrivilegedConfigurationParameters spineRollPrivilegedConfigurationParameters()
   {
      OneDoFJointBasics spineRoll = fullRobotModel.getOneDoFJointByName(spineRollJointName);
      //      System.out.println(spineRoll);

      OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
      jointParameters.setConfigurationGain(pPoseSpineRollKp.getValue());
      jointParameters.setVelocityGain(pPoseSpineRollKdFactor.getValue() * pPoseSpineRollKp.getValue());
      jointParameters.setWeight(1);
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(pPoseSpineRoll.getDoubleValue());

      privilegedConfigurationCommand.addJoint(spineRoll, jointParameters);

      return jointParameters;
   }

   private OneDoFJointPrivilegedConfigurationParameters spinePitchPrivilegedConfigurationParameters()
   {
      OneDoFJointBasics spinePitch = fullRobotModel.getOneDoFJointByName(spinePitchJointName);

      OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
      jointParameters.setConfigurationGain(pPoseSpinePitchKp.getValue());
      jointParameters.setVelocityGain(pPoseSpinePitchKdFactor.getValue() * pPoseSpinePitchKp.getValue());
      jointParameters.setWeight(1);
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(pPoseSpinePitch.getDoubleValue());

      privilegedConfigurationCommand.addJoint(spinePitch, jointParameters);

      return jointParameters;
   }

   private OneDoFJointPrivilegedConfigurationParameters spineYawPrivilegedConfigurationParameters()
   {
      OneDoFJointBasics spineYaw = fullRobotModel.getOneDoFJointByName(spineYawJointName);

      OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
      jointParameters.setConfigurationGain(pPoseSpineYawKp.getValue());
      jointParameters.setVelocityGain(pPoseSpineYawKdFactor.getValue() * pPoseSpineYawKp.getValue());
      jointParameters.setWeight(pPoseSpineYawWeight.getDoubleValue());
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(pPoseSpineYaw.getDoubleValue());

      privilegedConfigurationCommand.addJoint(spineYaw, jointParameters);

      return jointParameters;
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide,
                                                                                                           ArmJointName armJointName,
                                                                                                           double privilegedAngle)
   {
      OneDoFJointBasics armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);

      OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
      jointParameters.setConfigurationGain(40.0);//40.0);
      jointParameters.setVelocityGain(6.0);//6.0);
      jointParameters.setWeight(1);//5.0);
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(privilegedAngle);

      privilegedConfigurationCommand.addJoint(armJoint, jointParameters);

      return jointParameters;
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide,
                                                                                                           ArmJointName armJointName,
                                                                                                           double privilegedAngle,
                                                                                                           double weight,
                                                                                                           double pgain,
                                                                                                           double dgain)
   {
      OneDoFJointBasics armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);
      // System.out.println(armJoint);

      OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
      jointParameters.setConfigurationGain(pgain);
      jointParameters.setVelocityGain(dgain);
      jointParameters.setWeight(weight);
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(privilegedAngle);

      privilegedConfigurationCommand.addJoint(armJoint, jointParameters);

      return jointParameters;
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide,
                                                                                                           ArmJointName armJointName,
                                                                                                           double privilegedAngle,
                                                                                                           double pgain,
                                                                                                           double dgain)
   {
      return createAndAddJointPrivilegedConfigurationParameters(robotSide, armJointName, privilegedAngle, 1.0, pgain, dgain);
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide,
                                                                                                           LegJointName legJointName,
                                                                                                           double privilegedAngle,
                                                                                                           double pgain,
                                                                                                           double dgain)
   {
      OneDoFJointBasics legJoint = fullRobotModel.getLegJoint(robotSide, legJointName);

      OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
      jointParameters.setConfigurationGain(pgain);
      jointParameters.setVelocityGain(dgain);
      jointParameters.setWeight(1.0);
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(privilegedAngle);

      privilegedConfigurationCommand.addJoint(legJoint, jointParameters);

      return jointParameters;
   }

}
