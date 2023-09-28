package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.NaturalPostureParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.QPObjectiveCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;

public class NaturalPosturePrivilegedConfigurationController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final NaturalPostureParameters npParameters;

   private final YoFrameYawPitchRoll pelvisPrivilegedOrientation = new YoFrameYawPitchRoll("pPosePelvis", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll pelvisPrivilegedOrientationKp = new YoFrameYawPitchRoll("pPosePelvisKp", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll pelvisPrivilegedOrientationKd = new YoFrameYawPitchRoll("pPosePelvisKd", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll pelvisQPWeight = new YoFrameYawPitchRoll("pelvisQPWeight", ReferenceFrame.getWorldFrame(), registry);
   private final DMatrixRMaj yprDDot = new DMatrixRMaj(3, 1);

   private final QPObjectiveCommand pelvisQPObjectiveCommand = new QPObjectiveCommand();
   private final YoBoolean doNullSpaceProjectionForPelvis = new YoBoolean("doNullSpaceProjectionForPelvis", registry);
   private final DMatrixRMaj pelvisQPobjective = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj pelvisQPjacobian = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj pelvisQPweightMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj pelvisQPselectionMatrix = new DMatrixRMaj(1, 1);
   private final YawPitchRoll pelvisYPR = new YawPitchRoll();
   private final DMatrixRMaj pelvisYPRdot = new DMatrixRMaj(3, 1);
   private final FrameVector3D pelvisOmegaVec = new FrameVector3D();
   private final DMatrixRMaj pelvisOmega = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj Dpelvis = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj invDpelvis = new DMatrixRMaj(3, 3);
   private final YoFrameYawPitchRoll pelvisAngularAcceleration = new YoFrameYawPitchRoll("pelvisAngularAcceleration", ReferenceFrame.getWorldFrame(), registry);

   //TODO what are these 3 guys doing that we cant do with the next 4???
   private final YoFrameYawPitchRoll npPoseSpineKp = new YoFrameYawPitchRoll("npPoseSpineKp", ReferenceFrame.getWorldFrame(), registry);
   private final YoPDGains pPoseSpinePitchGains = new YoPDGains("pPoseSpinePitchGains", registry);
   private final YoPDGains pPoseSpineRollGains = new YoPDGains("pPoseSpineRollGains", registry);

   private final YoFrameYawPitchRoll spinePrivilegedOrientation = new YoFrameYawPitchRoll("pPoseSpine", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll pPoseSpineKp = new YoFrameYawPitchRoll("pPoseSpineKp", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll pPoseSpineKd = new YoFrameYawPitchRoll("pPoseSpineKd", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll pPoseSpineQPWeight = new YoFrameYawPitchRoll("pPoseSpineWeight", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameYawPitchRoll shoulderPrivilegedOrientation = new YoFrameYawPitchRoll("pPoseShoulder", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll pPoseShoulderKp = new YoFrameYawPitchRoll("pPoseShoulderKp", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll pPoseShoulderKd = new YoFrameYawPitchRoll("pPoseShoulderKd", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll pPoseShoulderQPWeight = new YoFrameYawPitchRoll("pPoseShoulderQPWeight", ReferenceFrame.getWorldFrame(), registry);

   private final YoDouble pPoseElbow = new YoDouble("pPoseElbow", registry);
   private final YoDouble pPoseElbowKp = new YoDouble("pPoseElbowKp", registry);
   private final YoDouble pPoseElbowKd = new YoDouble("pPoseElbowKd", registry);
   private final YoDouble pPoseElbowWeight = new YoDouble("pPoseElbowWeight", registry);

   //TODO These weren't used anywhere, do we need to keep them?
   //   private final YoDouble pPoseHipKp = new YoDouble("pPoseHipKp", registry);
   //   private final YoDouble pPoseHipKdFactor = new YoDouble("pPoseHipKdFactor", registry);
   //   private final YoDouble pPoseKneeKp = new YoDouble("pPoseKneeKp", registry);
   //   private final YoDouble pPoseKneeKdFactor = new YoDouble("pPoseKneeKdFactor", registry);

   private final YoBoolean useSpineRollPitchJointCommands = new YoBoolean("useSpineRollPitchJointCommands", registry);

   private final HashMap<OneDoFJointBasics, OneDoFJointPrivilegedConfigurationParameters> privilegedConfigurationMap = new HashMap<>();

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   private final OneDoFJointFeedbackControlCommand spinePitchCommand = new OneDoFJointFeedbackControlCommand();
   private final OneDoFJointFeedbackControlCommand spineRollCommand = new OneDoFJointFeedbackControlCommand();

   private final FullHumanoidRobotModel fullRobotModel;
   private final NaturalPostureParameters.SingleDOFJointPrivilegedParameters wristPrivilegedParameters;
   private final NaturalPostureParameters.SingleDOFJointPrivilegedParameters anklePrivilegedParameters;

   public NaturalPosturePrivilegedConfigurationController(NaturalPostureParameters npParameters,
                                                          FullHumanoidRobotModel fullRobotModel,
                                                          YoRegistry parentRegistry)
   {
      this.npParameters = npParameters;
      this.fullRobotModel = fullRobotModel;

      spinePrivilegedOrientation.set(npParameters.getSpinePrivilegedParameters().getPrivilegedOrientation());

      //0.1 //0.2 // Pitch is the first joint from the body. The more negative this number is, the more forward the left arm is
      // Roll: the smaller, the further away the arm is from the body  // start at -1 for hardware experiment to be safe
      shoulderPrivilegedOrientation.set(npParameters.getShoulderPrivilegedParameters().getPrivilegedOrientation());
      // 0 looks more natural, but -0.3 might avoid arm from colliding into the hydraulic manifold. //-0.5 //-1   // the smaller, the more bent the elbow is
      pPoseElbow.set(npParameters.getElbowPrivilegedParameters().getPrivilegedOrientation());

      pPoseSpineQPWeight.set(npParameters.getSpinePrivilegedParameters().getQPWeight()); // weight used to complete with other privileged joint position.
      pPoseShoulderQPWeight.set(npParameters.getShoulderPrivilegedParameters().getQPWeight());
      pPoseElbowWeight.set(npParameters.getElbowPrivilegedParameters().getQPWeight());

      useSpineRollPitchJointCommands.set(npParameters.getUseSpineRollPitchJointCommands()); // Can turn off joint limit for the spine when this is true.
      if (useSpineRollPitchJointCommands.getBooleanValue())
      {
         npPoseSpineKp.set(npParameters.getSpineNaturalPostureOrientationKp());

         pPoseSpinePitchGains.setKp(npPoseSpineKp.getPitch()); //25
         pPoseSpineRollGains.setKp(npPoseSpineKp.getRoll()); //25
         pPoseSpinePitchGains.setZeta(npParameters.getSpineDamping());
         pPoseSpineRollGains.setZeta(npParameters.getSpineDamping());
         pPoseSpinePitchGains.createDerivativeGainUpdater(true);
         pPoseSpineRollGains.createDerivativeGainUpdater(true);
      }

      pPoseSpineKp.set(npParameters.getSpinePrivilegedParameters().getKpGain());
      pPoseSpineKd.set(npParameters.getSpinePrivilegedParameters().getKdGain());

      pPoseShoulderKp.set(npParameters.getShoulderPrivilegedParameters().getKpGain());
      pPoseShoulderKd.set(npParameters.getShoulderPrivilegedParameters().getKdGain());

      pPoseElbowKp.set(npParameters.getElbowPrivilegedParameters().getKpGain());
      pPoseElbowKd.set(npParameters.getElbowPrivilegedParameters().getKdGain());

      //TODO These weren't used anywhere, do we need to keep them?
      // privileged configuration for lower body
      //      pPoseHipKp.set(100);
      //      pPoseHipKdFactor.set(0.2);
      //      pPoseKneeKp.set(100);
      //      pPoseKneeKdFactor.set(0.2);

      OneDoFJointBasics spineRoll = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);
      OneDoFJointBasics spinePitch = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);

      spinePitchCommand.clear();
      spinePitchCommand.setJoint(spinePitch);

      spineRollCommand.clear();
      spineRollCommand.setJoint(spineRoll);

      wristPrivilegedParameters = npParameters.getWristPrivilegedParameters();
      anklePrivilegedParameters = npParameters.getAnklePrivilegedParameters();

      // Pelvis privileged pose
      pelvisQPobjective.reshape(3, 1);
      pelvisQPjacobian.reshape(3, 6 + fullRobotModel.getOneDoFJoints().length);
      pelvisQPweightMatrix.reshape(3, 3);
      pelvisQPselectionMatrix.reshape(3, 3);
      CommonOps_DDRM.setIdentity(pelvisQPselectionMatrix);

      pelvisPrivilegedOrientation.set(npParameters.getPelvisPrivilegedParameters().getPrivilegedOrientation());
      pelvisPrivilegedOrientationKp.set(npParameters.getPelvisPrivilegedParameters().getKpGain());
      pelvisPrivilegedOrientationKd.set(npParameters.getPelvisPrivilegedParameters().getKdGain());
      pelvisQPWeight.set(npParameters.getPelvisPrivilegedParameters().getQPWeight());

      doNullSpaceProjectionForPelvis.set(true);

      parentRegistry.addChild(registry);

      //initialize
      updatePrivilegedConfigurationCommand();
   }

   public void compute()
   {
      feedbackControlCommandList.clear();

      // Set QP objective for pelvis privileged pose:
      pelvisPrivilegedPoseQPObjectiveCommand();

      // Testing -- track spine joint x and y with highest priority
      if (useSpineRollPitchJointCommands.getBooleanValue())
      {
         pPoseSpinePitchGains.setKp(npPoseSpineKp.getPitch());
         pPoseSpineRollGains.setKp(npPoseSpineKp.getRoll());

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

   //Implements a YPR servo on the pelvis, which is then used for the privileged
   //pose of the pelvis (via task null-space projection)
   private void pelvisPrivilegedPoseQPObjectiveCommand()
   {
      pelvisQPweightMatrix.set(0, 0, pelvisQPWeight.getRoll());
      pelvisQPweightMatrix.set(1, 1, pelvisQPWeight.getPitch());
      pelvisQPweightMatrix.set(2, 2, pelvisQPWeight.getYaw());

      // Get current pelvis YPR and omega:
      pelvisYPR.set(fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame().getRotation());
      pelvisOmegaVec.setIncludingFrame(fullRobotModel.getPelvis().getBodyFixedFrame().getTwistOfFrame().getAngularPart());
      // Ugh...
      pelvisOmega.set(0, 0, pelvisOmegaVec.getX());
      pelvisOmega.set(1, 0, pelvisOmegaVec.getY());
      pelvisOmega.set(2, 0, pelvisOmegaVec.getZ());

      double sbe = Math.sin(pelvisYPR.getPitch());
      double cbe = Math.cos(pelvisYPR.getPitch());
      double sal = Math.sin(pelvisYPR.getRoll());
      double cal = Math.cos(pelvisYPR.getRoll());
      Dpelvis.set(0, 0, -sbe);
      Dpelvis.set(0, 1, 0.0);
      Dpelvis.set(0, 2, 1.0);
      Dpelvis.set(1, 0, cbe * sal);
      Dpelvis.set(1, 1, cal);
      Dpelvis.set(1, 2, 0.0);
      Dpelvis.set(2, 0, cbe * cal);
      Dpelvis.set(2, 1, -sal);
      Dpelvis.set(2, 2, 0.0);

      CommonOps_DDRM.invert(Dpelvis, invDpelvis);
      CommonOps_DDRM.mult(invDpelvis, pelvisOmega, pelvisYPRdot); // pelvis YPR rates

      // The pelvis equilibrium pose servo:
      pelvisAngularAcceleration.setYaw(pelvisPrivilegedOrientationKp.getYaw() * (pelvisPrivilegedOrientation.getYaw() - pelvisYPR.getYaw())
                                       - pelvisPrivilegedOrientationKd.getYaw() * pelvisYPRdot.get(0, 0));
      pelvisAngularAcceleration.setPitch(pelvisPrivilegedOrientationKp.getPitch() * (pelvisPrivilegedOrientation.getPitch() - pelvisYPR.getPitch())
                                         - pelvisPrivilegedOrientationKd.getPitch() * pelvisYPRdot.get(1, 0));
      pelvisAngularAcceleration.setRoll(pelvisPrivilegedOrientationKp.getRoll() * (pelvisPrivilegedOrientation.getRoll() - pelvisYPR.getRoll())
                                        - pelvisPrivilegedOrientationKd.getRoll() * pelvisYPRdot.get(2, 0));

      yprDDot.set(0, 0, pelvisAngularAcceleration.getYaw());
      yprDDot.set(1, 0, pelvisAngularAcceleration.getPitch());
      yprDDot.set(2, 0, pelvisAngularAcceleration.getRoll());

      CommonOps_DDRM.mult(Dpelvis, yprDDot, pelvisQPobjective); // GMN: missing D-dot*yprDot term

      pelvisQPjacobian.zero(); // GMN: necessary??
      pelvisQPjacobian.set(0, 0, 1.0);
      pelvisQPjacobian.set(1, 1, 1.0);
      pelvisQPjacobian.set(2, 2, 1.0);

      // Populate the QPObjectiveCommand:
      pelvisQPObjectiveCommand.setDoNullSpaceProjection(doNullSpaceProjectionForPelvis.getBooleanValue());
      pelvisQPObjectiveCommand.getObjective().set(pelvisQPobjective);
      pelvisQPObjectiveCommand.getJacobian().set(pelvisQPjacobian);
      pelvisQPObjectiveCommand.getSelectionMatrix().set(pelvisQPselectionMatrix);
      pelvisQPObjectiveCommand.getWeightMatrix().set(pelvisQPweightMatrix);
   }

   public InverseDynamicsCommand<?> getPelvisPrivilegedPoseCommand()
   {
      return pelvisQPObjectiveCommand;
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

      if (npParameters.getUseSpinePrivilegedCommand())
      {
         spineYawPrivilegedConfigurationParameters();
      }

      for (RobotSide side : RobotSide.values)
      {
         createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getArmJoint(side, ArmJointName.SHOULDER_PITCH),
                                                            side.negateIfRightSide(shoulderPrivilegedOrientation.getPitch()),
                                                            pPoseShoulderQPWeight.getPitch(),
                                                            pPoseShoulderKp.getPitch(),
                                                            pPoseShoulderKd.getPitch());
         createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getArmJoint(side, ArmJointName.SHOULDER_ROLL),
                                                            shoulderPrivilegedOrientation.getRoll(),
                                                            pPoseShoulderQPWeight.getRoll(),
                                                            pPoseShoulderKp.getRoll(),
                                                            pPoseShoulderKd.getRoll());
         createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getArmJoint(side, ArmJointName.SHOULDER_YAW),
                                                            shoulderPrivilegedOrientation.getYaw(),
                                                            pPoseShoulderQPWeight.getYaw(),
                                                            pPoseShoulderKp.getYaw(),
                                                            pPoseShoulderKd.getYaw());

         createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getArmJoint(side, ArmJointName.ELBOW_PITCH),
                                                            side.negateIfRightSide(pPoseElbow.getDoubleValue()),
                                                            pPoseElbowWeight.getDoubleValue(),
                                                            pPoseElbowKp.getDoubleValue(),
                                                            pPoseElbowKd.getDoubleValue());

         createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getArmJoint(side, ArmJointName.WRIST_YAW),
                                                            wristPrivilegedParameters.getPrivilegedOrientation(),
                                                            wristPrivilegedParameters.getQPWeight(),
                                                            wristPrivilegedParameters.getKpGain(),
                                                            wristPrivilegedParameters.getKdGain());
         createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getArmJoint(side, ArmJointName.WRIST_ROLL),
                                                            wristPrivilegedParameters.getPrivilegedOrientation(),
                                                            wristPrivilegedParameters.getQPWeight(),
                                                            wristPrivilegedParameters.getKpGain(),
                                                            wristPrivilegedParameters.getKdGain());
         createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getArmJoint(side, ArmJointName.FIRST_WRIST_PITCH),
                                                            wristPrivilegedParameters.getPrivilegedOrientation(),
                                                            wristPrivilegedParameters.getQPWeight(),
                                                            wristPrivilegedParameters.getKpGain(),
                                                            wristPrivilegedParameters.getKdGain());

         createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getLegJoint(side, LegJointName.ANKLE_ROLL),
                                                            anklePrivilegedParameters.getPrivilegedOrientation(),
                                                            anklePrivilegedParameters.getQPWeight(),
                                                            anklePrivilegedParameters.getKpGain(),
                                                            anklePrivilegedParameters.getKdGain());
         createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getLegJoint(side, LegJointName.ANKLE_PITCH),
                                                            anklePrivilegedParameters.getPrivilegedOrientation(),
                                                            anklePrivilegedParameters.getQPWeight(),
                                                            anklePrivilegedParameters.getKpGain(),
                                                            anklePrivilegedParameters.getKdGain());
      }
   }

   private void spineRollPrivilegedConfigurationParameters()
   {
      createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL),
                                                         spinePrivilegedOrientation.getRoll(),
                                                         pPoseSpineQPWeight.getRoll(),
                                                         pPoseSpineKp.getRoll(),
                                                         pPoseSpineKd.getRoll());
   }

   private void spinePitchPrivilegedConfigurationParameters()
   {

      createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH),
                                                         spinePrivilegedOrientation.getPitch(),
                                                         pPoseSpineQPWeight.getPitch(),
                                                         pPoseSpineKp.getPitch(),
                                                         pPoseSpineKd.getPitch());
   }

   private void spineYawPrivilegedConfigurationParameters()
   {
      createAndAddJointPrivilegedConfigurationParameters(fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW),
                                                         spinePrivilegedOrientation.getYaw(),
                                                         pPoseSpineQPWeight.getYaw(),
                                                         pPoseSpineKp.getYaw(),
                                                         pPoseSpineKd.getYaw());
   }

   private void createAndAddJointPrivilegedConfigurationParameters(OneDoFJointBasics joint, double privilegedAngle, double weight, double pgain, double dgain)
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
   }
}
