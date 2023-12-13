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
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;

public class NaturalPosturePrivilegedConfigurationController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoVector3D pelvisPrivilegedOrientation = new YoVector3D("naturalPosturePrivilegedOrientationPelvis", registry);
   private final YoVector3D pelvisPrivilegedOrientationKp = new YoVector3D("naturalPosturePrivilegedOrientationPelvisKp", registry);
   private final YoVector3D pelvisPrivilegedOrientationKd = new YoVector3D("naturalPosturePrivilegedOrientationKd", registry);
   private final YoVector3D pelvisQPWeight = new YoVector3D("naturalPosturePrivilegedOrientationPelvisWeight", registry);
   private final DMatrixRMaj yprDDot = new DMatrixRMaj(3, 1);

   private final QPObjectiveCommand pelvisQPObjectiveCommand = new QPObjectiveCommand();
   private final YoBoolean doNullSpaceProjectionForPelvis = new YoBoolean("doNullSpaceProjectionForPelvisForNaturalPosture", registry);
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
   private final YoFrameYawPitchRoll pelvisAngularAcceleration = new YoFrameYawPitchRoll("naturalPosturePelvisAngularAcceleration",
                                                                                         ReferenceFrame.getWorldFrame(),
                                                                                         registry);

   private final ArrayList<YoJointPrivilegedConfigurationParameters> yoJointPrivilegedConfigurationParametersList = new ArrayList<>();
   private final OneDoFJointPrivilegedConfigurationParameters jointParameters = new OneDoFJointPrivilegedConfigurationParameters();

   private final ArrayList<OneDoFJointFeedbackControlCommand> primaryTaskCommandList = new ArrayList<>();
   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   private final FullHumanoidRobotModel fullRobotModel;

   public NaturalPosturePrivilegedConfigurationController(NaturalPostureParameters parameters, FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;

      ArrayList<NaturalPostureParameters.OneDofJointPrivilegedParameters> jointPrivilegedParametersList = parameters.getJointPrivilegedParametersList();

      for (int i = 0; i < jointPrivilegedParametersList.size(); i++)
      {
         yoJointPrivilegedConfigurationParametersList.add(new YoJointPrivilegedConfigurationParameters(jointPrivilegedParametersList.get(i), registry));
         OneDoFJointFeedbackControlCommand command = new OneDoFJointFeedbackControlCommand();
         primaryTaskCommandList.add(command);
      }

      OneDoFJointBasics spineRoll = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);
      OneDoFJointBasics spinePitch = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);

      // Pelvis privileged pose
      pelvisQPobjective.reshape(3, 1);
      pelvisQPjacobian.reshape(3, 6 + fullRobotModel.getOneDoFJoints().length);
      pelvisQPweightMatrix.reshape(3, 3);
      pelvisQPselectionMatrix.reshape(3, 3);
      CommonOps_DDRM.setIdentity(pelvisQPselectionMatrix);

      pelvisPrivilegedOrientation.set(parameters.getPelvisPrivilegedParameters().getPrivilegedOrientation());
      pelvisPrivilegedOrientationKp.set(parameters.getPelvisPrivilegedParameters().getKpGain());
      pelvisPrivilegedOrientationKd.set(parameters.getPelvisPrivilegedParameters().getKdGain());
      pelvisQPWeight.set(parameters.getPelvisPrivilegedParameters().getQPWeight());

      doNullSpaceProjectionForPelvis.set(parameters.getDoNullSpaceProjectionForPelvis());

      parentRegistry.addChild(registry);

      //initialize
      updatePrivilegedConfigurationCommand();
   }

   public void compute()
   {
      feedbackControlCommandList.clear();

      // Set QP objective for pelvis privileged pose:
      pelvisPrivilegedPoseQPObjectiveCommand();

      updatePrivilegedConfigurationCommand();
   }

   //Implements a YPR servo on the pelvis, which is then used for the privileged
   //pose of the pelvis (via task null-space projection)
   private void pelvisPrivilegedPoseQPObjectiveCommand()
   {
      pelvisQPweightMatrix.set(0, 0, pelvisQPWeight.getX());
      pelvisQPweightMatrix.set(1, 1, pelvisQPWeight.getY());
      pelvisQPweightMatrix.set(2, 2, pelvisQPWeight.getZ());

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
      pelvisAngularAcceleration.setYaw(pelvisPrivilegedOrientationKp.getZ() * (pelvisPrivilegedOrientation.getZ() - pelvisYPR.getYaw())
                                       - pelvisPrivilegedOrientationKd.getZ() * pelvisYPRdot.get(0, 0));
      pelvisAngularAcceleration.setPitch(pelvisPrivilegedOrientationKp.getY() * (pelvisPrivilegedOrientation.getY() - pelvisYPR.getPitch())
                                         - pelvisPrivilegedOrientationKd.getY() * pelvisYPRdot.get(1, 0));
      pelvisAngularAcceleration.setRoll(pelvisPrivilegedOrientationKp.getX() * (pelvisPrivilegedOrientation.getX() - pelvisYPR.getRoll())
                                        - pelvisPrivilegedOrientationKd.getX() * pelvisYPRdot.get(2, 0));

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

      int j = 0;
      for (int i = 0; i < yoJointPrivilegedConfigurationParametersList.size(); i++)
      {
         /* Here we separate the privileged tasks based on whether they are a primary task or secondary task.
            Primary tasks are sent as feedback commands, and secondary tasks are sent as privileged configuration commands. */
         if (yoJointPrivilegedConfigurationParametersList.get(i).isPrimaryTask())
         {
            createJointPrivilegedPrimaryTaskCommand(yoJointPrivilegedConfigurationParametersList.get(i), primaryTaskCommandList.get(i));
         }
         else
         {
            createJointPrivilegedCommand(yoJointPrivilegedConfigurationParametersList.get(i));
         }
      }
   }

   private void createJointPrivilegedPrimaryTaskCommand(YoJointPrivilegedConfigurationParameters privilegedParameters,
                                                        OneDoFJointFeedbackControlCommand command)
   {
      command.clear();
      command.setJoint(fullRobotModel.getOneDoFJointByName(privilegedParameters.getJointName()));
      command.setInverseDynamics(privilegedParameters.getYoPrivilegedOrientation().getDoubleValue(), 0.0, 0.0);
      command.setGains(privilegedParameters.getPDGains());
      feedbackControlCommandList.addCommand(command);
   }

   private void createJointPrivilegedCommand(YoJointPrivilegedConfigurationParameters privilegedParameters)
   {
      jointParameters.clear();

      jointParameters.setConfigurationGain(privilegedParameters.getPDGains().getKp());
      jointParameters.setVelocityGain(privilegedParameters.getPDGains().getKd());
      jointParameters.setWeight(privilegedParameters.getYoWeight().getDoubleValue());
      jointParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      jointParameters.setPrivilegedConfigurationOption(null);
      jointParameters.setPrivilegedConfiguration(privilegedParameters.getYoPrivilegedOrientation().getDoubleValue());

      privilegedConfigurationCommand.addJoint(fullRobotModel.getOneDoFJointByName(privilegedParameters.getJointName()), jointParameters);
   }
}
