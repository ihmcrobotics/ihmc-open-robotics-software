package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
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

   private final YoDouble pPoseSpineRoll = new YoDouble("pPoseSpineRoll", registry);
   private final YoDouble pPoseSpinePitch = new YoDouble("pPoseSpinePitch", registry);
   private final YoDouble pPoseSpineYaw = new YoDouble("pPoseSpineYaw", registry);

   private final YoDouble pPoseShoulderPitch = new YoDouble("pPoseShoulderPitch", registry);
   private final YoDouble pPoseShoulderRoll = new YoDouble("pPoseShoulderRoll", registry);
   private final YoDouble pPoseShoulderYaw = new YoDouble("pPoseShoulderYaw", registry);
   private final YoDouble pPoseElbow = new YoDouble("pPoseElbow", registry);

   private final YoDouble pPoseSpineYawWeight = new YoDouble("pPoseSpineYawWeight", registry);
   private final YoDouble pPoseShoulderYawWeight = new YoDouble("pPoseShoulderYawWeight", registry);
   private final YoDouble pPoseShoulderPitchWeight = new YoDouble("pPoseShoulderPitchWeight", registry);
   private final YoDouble pPoseElbowWeight = new YoDouble("pPoseElbowWeight", registry);
   private final YoDouble pPoseDefaultWeight = new YoDouble("pPoseDefaultWeight", registry);

   private final YoPDGains pPoseSpineRollPitchGains = new YoPDGains("_privPoseRollPitch", registry);
   private final YoPDGains pPoseSpineYawGains = new YoPDGains("_privPoseYaw", registry);

   private final YoPDGains pPoseShoulderGains = new YoPDGains("_privPoseShoulder", registry);
   private final YoPDGains pPoseElbowGains = new YoPDGains("_privPoseElbow", registry);
   private final YoPDGains pPoseWristGains = new YoPDGains("_privPoseWrist", registry);

   //   private final YoDouble pPoseHipPitch = new YoDouble("pPoseHipPitch", registry);
   private final YoPDGains pPoseHipGains = new YoPDGains("_privPoseHip", registry);
   private final YoPDGains pPoseKneeGains = new YoPDGains("_privPoseKnee", registry);
   private final YoPDGains pPoseAnkleGains = new YoPDGains("_privPoseAnkle", registry);
   //   private final YoDouble pPoseHipRoll = new YoDouble("pPoseHipRoll", registry);
   //   private final YoDouble pPoseHipYaw = new YoDouble("pPoseHipYaw", registry);
   //   private final YoDouble pPoseKnee = new YoDouble("pPoseKnee", registry);


   // These are the gains for the direct joint control. TODO Name them better.
   private final YoPDGains spineRollPitchGains = new YoPDGains("_SpineRollPitchGains", registry);
   private final YoDouble spineRollPitchWeight = new YoDouble("SpineRollPitchWeight", registry);

   private final YoBoolean useSpineRollPitchJointCommands = new YoBoolean("useSpineRollPitchJointCommands", registry);
   private final YoBoolean useSpineYawPrivilegedCommand = new YoBoolean("useSpineYawPrivilegedCommand", registry);

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

      spineRollPitchGains.setKp(25.0);
      spineRollPitchGains.setZeta(0.7);
      spineRollPitchGains.createDerivativeGainUpdater(true);

      spineRollPitchWeight.set(100.0);


      // privileged configuration for upper body
      useSpineYawPrivilegedCommand.set(false);

      pPoseSpineRoll.set(0.0);
      pPoseSpinePitch.set(0.0);
      pPoseSpineYaw.set(0.0);
      double delta = 0.0;
      pPoseShoulderPitch.set(0 + delta); //0.1 //0.2   // the bigger, the further away the arm is from the body
      pPoseShoulderRoll.set(-1.0); // the smaller, the further away the arm is from the body   // start at -1 for hardware experiment to be safe
      pPoseShoulderYaw.set(0);
      pPoseElbow.set(-0.4); //-0.5 //-1   // the smaller, the more bent the elbow is

      pPoseSpineYawWeight.set(5.0); // weight used to complete with other privileged joint position. Other joint default weights are 1
      pPoseShoulderYawWeight.set(1.0); // this weight doesn't matter much

      pPoseDefaultWeight.set(1.0);

      pPoseSpineRollPitchGains.setKp(50.0);
      pPoseSpineRollPitchGains.setZeta(0.5);
      pPoseSpineRollPitchGains.createDerivativeGainUpdater(true);

      pPoseSpineYawGains.setKp(300.0);
      pPoseSpineYawGains.setZeta(1.2);
      pPoseSpineYawGains.createDerivativeGainUpdater(true);

      pPoseShoulderGains.setKp(50.0);
      pPoseShoulderGains.setZeta(1.0);
      pPoseShoulderGains.createDerivativeGainUpdater(true);

      pPoseShoulderPitchWeight.set(5.0);
      pPoseElbowWeight.set(10.0);
      pPoseElbowGains.setKp(30.0);
      pPoseElbowGains.setZeta(0.4);
      pPoseElbowGains.createDerivativeGainUpdater(true);

      pPoseWristGains.setKp(40.0);
      pPoseWristGains.setZeta(0.5);
      pPoseWristGains.createDerivativeGainUpdater(true);

      // privileged configuration for lower body
      pPoseHipGains.setKp(100.0);
      pPoseHipGains.setZeta(1.0);
      pPoseHipGains.createDerivativeGainUpdater(true);

      pPoseKneeGains.setKp(200.0);
      pPoseKneeGains.setZeta(1.0);
      pPoseKneeGains.createDerivativeGainUpdater(true);

      pPoseAnkleGains.setKp(4.0);
      pPoseAnkleGains.setZeta(0.25);
      pPoseAnkleGains.createDerivativeGainUpdater(true);


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

      if (useSpineRollPitchJointCommands.getBooleanValue())
      {
         feedbackControlCommandList.addCommand(spinePitchCommand);
         feedbackControlCommandList.addCommand(spineRollCommand);
      }

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
         spinePitchCommand.setGains(spineRollPitchGains);
         spinePitchCommand.setWeightForSolver(spineRollPitchWeight.getDoubleValue());

         spineRollCommand.setJoint(spineRoll);
         spineRollCommand.setInverseDynamics(0.0, 0.0, 0.0);
         spineRollCommand.setGains(spineRollPitchGains);
         spineRollCommand.setWeightForSolver(spineRollPitchWeight.getDoubleValue());
      }
   }

   private void updatePrivilegedConfigurationCommand()
   {
      privilegedConfigurationCommand.clear();
      privilegedConfigurationCommand.enable();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationCommand.PrivilegedConfigurationOption.AT_ZERO);

      if (!useSpineRollPitchJointCommands.getValue())
      {
         spineRollPrivilegedConfigurationParameters();
         spinePitchPrivilegedConfigurationParameters();
      }
      if (useSpineYawPrivilegedCommand.getValue())
         spineYawPrivilegedConfigurationParameters();

      for (RobotSide side : RobotSide.values)
      {
         createAndAddJointPrivilegedConfigurationParameters(side,
                                                            ArmJointName.SHOULDER_PITCH,
                                                            side.negateIfRightSide(pPoseShoulderPitch.getDoubleValue()),
                                                            pPoseShoulderPitchWeight.getDoubleValue(),
                                                            pPoseShoulderGains);
         createAndAddJointPrivilegedConfigurationParameters(side,
                                                            ArmJointName.SHOULDER_ROLL,
                                                            pPoseShoulderRoll.getDoubleValue(),
                                                            pPoseShoulderGains);
         createAndAddJointPrivilegedConfigurationParameters(side,
                                                            ArmJointName.SHOULDER_YAW,
                                                            pPoseShoulderYaw.getDoubleValue(),
                                                            pPoseShoulderYawWeight.getDoubleValue(),
                                                            pPoseShoulderGains);
         createAndAddJointPrivilegedConfigurationParameters(side,
                                                            ArmJointName.ELBOW_PITCH,
                                                            side.negateIfRightSide(pPoseElbow.getDoubleValue()),
                                                            pPoseElbowWeight.getDoubleValue(),
                                                            pPoseElbowGains);

         // FIXME remove the hard-coded values
         createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.WRIST_YAW, 0.0, pPoseWristGains);
         createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.WRIST_ROLL, 0.0, pPoseWristGains);
         createAndAddJointPrivilegedConfigurationParameters(side, ArmJointName.FIRST_WRIST_PITCH, 0.0, pPoseWristGains);

         createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_PITCH, -0.25, pPoseHipGains);
         createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_ROLL, 0.0, pPoseHipGains);
         createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.HIP_YAW, 0.0, pPoseHipGains);

         createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.KNEE_PITCH, 0.5, pPoseKneeGains);

         createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.ANKLE_ROLL, 0.0, pPoseAnkleGains);
         createAndAddJointPrivilegedConfigurationParameters(side, LegJointName.ANKLE_PITCH, 0.0, pPoseAnkleGains);
      }

   }

   private OneDoFJointPrivilegedConfigurationParameters spineRollPrivilegedConfigurationParameters()
   {
      return createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL),
                                                                pPoseSpineRoll.getDoubleValue(),
                                                                pPoseDefaultWeight.getDoubleValue(),
                                                                pPoseSpineRollPitchGains);
   }

   private OneDoFJointPrivilegedConfigurationParameters spinePitchPrivilegedConfigurationParameters()
   {
      return createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH),
                                                                pPoseSpinePitch.getDoubleValue(),
                                                                pPoseDefaultWeight.getDoubleValue(),
                                                                pPoseSpineRollPitchGains);
   }

   private OneDoFJointPrivilegedConfigurationParameters spineYawPrivilegedConfigurationParameters()
   {
      return createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW),
                                                                pPoseSpineYaw.getDoubleValue(),
                                                                pPoseSpineYawWeight.getDoubleValue(),
                                                                pPoseSpineYawGains);
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide,
                                                                                                           ArmJointName armJointName,
                                                                                                           double privilegedAngle,
                                                                                                           PDGainsReadOnly gains)
   {
      return createAndAddJointPrivilegedConfigurationParameters(robotSide, armJointName, privilegedAngle, pPoseDefaultWeight.getDoubleValue(), gains);
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide,
                                                                                                           ArmJointName armJointName,
                                                                                                           double privilegedAngle,
                                                                                                           double weight,
                                                                                                           PDGainsReadOnly gains)
   {
      return createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getArmJoint(robotSide, armJointName),
                                                                privilegedAngle,
                                                                weight,
                                                                gains);
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide,
                                                                                                           LegJointName legJointName,
                                                                                                           double privilegedAngle,
                                                                                                           double weight,
                                                                                                           PDGainsReadOnly gains)
   {
      return createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getLegJoint(robotSide, legJointName),
                                                                privilegedAngle,
                                                                weight,
                                                                gains);
   }

   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(RobotSide robotSide,
                                                                                                           LegJointName legJointName,
                                                                                                           double privilegedAngle,
                                                                                                           PDGainsReadOnly gains)
   {
      return createAndAddJointPrivilegedConfigurationParameters(robotSide, legJointName, privilegedAngle, pPoseDefaultWeight.getDoubleValue(), gains);

   }


   private OneDoFJointPrivilegedConfigurationParameters createAndAddJointPrivilegedConfigurationParameters(OneDoFJointBasics joint,
                                                                                                           double privilegedAngle,
                                                                                                           double weight,
                                                                                                           PDGainsReadOnly gains)
   {
      OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();
      jointParameters.setConfigurationGain(gains.getKp());
      jointParameters.setVelocityGain(gains.getKp());
      jointParameters.setWeight(weight);
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(privilegedAngle);

      privilegedConfigurationCommand.addJoint(joint, jointParameters);

      return jointParameters;
   }


}
