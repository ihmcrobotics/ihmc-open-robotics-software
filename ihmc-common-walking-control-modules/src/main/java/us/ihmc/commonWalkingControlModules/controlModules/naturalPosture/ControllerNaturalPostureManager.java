package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.configurations.HumanoidRobotNaturalPosture;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.QPObjectiveCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.referenceFrames.OrientationFrame;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ControllerNaturalPostureManager
{
   private static final boolean useAxisAngleFeedbackController = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());


   private final double controlDT;

   private final QPObjectiveCommand naturalPostureControlCommand = new QPObjectiveCommand();

   HumanoidRobotNaturalPosture robotNaturalPosture;
   private final DMatrixRMaj npQPobjective = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj npQPweightMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj npQPselectionMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj yprDDot = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj Dnp = new DMatrixRMaj(3, 3);

   private final YoVector3D yoNPQPObjective = new YoVector3D("npQPObjective", registry);
   private final YoVector3D yoAltNPQPObjective = new YoVector3D("altNpQPObjective", registry);

   private final YoDouble npYaw = new YoDouble("npYaw", registry);
   private final YoDouble npPitch = new YoDouble("npPitch", registry);
   private final YoDouble npRoll = new YoDouble("npRoll", registry);

   private final YoDouble npVelocityAlpha = new YoDouble("npVelocityAlpha", registry);
   private final YoDouble npVelocityBreakFrequency = new YoDouble("npVelocityBreakFrequency", registry);

   private final FilteredVelocityYoVariable npYawVelocity, npPitchVelocity, npRollVelocity;

   private final YoDouble npKpYaw = new YoDouble("npKpYaw", registry);
   private final YoDouble npKpPitch = new YoDouble("npKpPitch", registry);
   private final YoDouble npKpRoll = new YoDouble("npKpRoll", registry);

   private final YoDouble npKdYaw = new YoDouble("npKdYaw", registry);
   private final YoDouble npKdPitch = new YoDouble("npKdPitch", registry);
   private final YoDouble npKdRoll = new YoDouble("npKdRoll", registry);

   private final YoDouble npQPWeightX = new YoDouble("npQPWeightX", registry);
   private final YoDouble npQPWeightY = new YoDouble("npQPWeightY", registry);
   private final YoDouble npQPWeightZ = new YoDouble("npQPWeightZ", registry);

   private final YoDouble npYawDesired = new YoDouble("npYawDesired", registry);
   private final YoDouble npPitchDesired = new YoDouble("npPitchDesired", registry);
   private final YoDouble npRollDesired = new YoDouble("npRollDesired", registry);


   private final YoFrameQuaternion yoCurrentNaturalPosture = new YoFrameQuaternion("currentNaturalPosture", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameQuaternion yoDesiredNaturalPosture = new YoFrameQuaternion("desiredNaturalPosture", ReferenceFrame.getWorldFrame(), registry);
   private final FrameQuaternion desiredNaturalPosture = new FrameQuaternion();


   // These are the variables used when using the axis angle error based controller;
   private final YoFrameVector3D yoProportionalFeedback;
   private final YoFrameVector3D yoDerivativeFeedback;

   private final YoFrameVector3D feedbackNPAcceleration;

   private final YoFrameVector3D errorRotationVector;

   // These are the variables when using the euler angle error based controller
   private final YoFrameVector3D yoDirectProportionalFeedback;
   private final YoFrameVector3D yoDirectDerivativeFeedback;

   private final YoDouble npYawAcceleration;
   private final YoDouble npPitchAcceleration;
   private final YoDouble npRollAcceleration;

   // Temp variales
   private final FrameVector3D proportionalFeedback = new FrameVector3D();
   private final FrameVector3D derivativeFeedback = new FrameVector3D();


   private final OrientationFrame naturalPostureFrame = new OrientationFrame(yoCurrentNaturalPosture);

   private final QPObjectiveCommand pelvisQPObjectiveCommand = new QPObjectiveCommand();

   private final YoDouble pelvisQPWeightX = new YoDouble("pelvisQPWeightX", registry);
   private final YoDouble pelvisQPWeightY = new YoDouble("pelvisQPWeightY", registry);
   private final YoDouble pelvisQPWeightZ = new YoDouble("pelvisQPWeightZ", registry);
   
   private final YoDouble pPosePelvisYaw = new YoDouble("pPosePelvisYaw", registry);
   private final YoDouble pPosePelvisPitch = new YoDouble("pPosePelvisPitch", registry);
   private final YoDouble pPosePelvisRoll = new YoDouble("pPosePelvisRoll", registry);
   private final YoDouble pPosePelvisYawKp = new YoDouble("pPosePelvisYawKp", registry);
   private final YoDouble pPosePelvisPitchKp = new YoDouble("pPosePelvisPitchKp", registry);
   private final YoDouble pPosePelvisRollKp = new YoDouble("pPosePelvisRollKp", registry);
   private final YoDouble pPosePelvisYawKdFactor = new YoDouble("pPosePelvisYawKdFactor", registry);
   private final YoDouble pPosePelvisPitchKdFactor = new YoDouble("pPosePelvisPitchKdFactor", registry);
   private final YoDouble pPosePelvisRollKdFactor = new YoDouble("pPosePelvisRollKdFactor", registry);

   private final YoVector3D yoPelvisQPObjective = new YoVector3D("pelvisQPObjective", registry);

   private final DMatrixRMaj pelvisQPobjective = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj pelvisQPjacobian = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj pelvisQPweightMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj pelvisQPselectionMatrix = new DMatrixRMaj(1, 1);
   private final YawPitchRoll pelvisYPR = new YawPitchRoll();
   private final DMatrixRMaj pelvisYPRdot = new DMatrixRMaj(3, 1);
   private final FrameVector3D pelvisOmegaVec = new FrameVector3D();
   private final DMatrixRMaj pelvisOmega = new DMatrixRMaj(3, 1);
   //   private final DMatrixRMaj pelvisAlpha = new DMatrixRMaj(3,1);
   private final DMatrixRMaj Dpelvis = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj invDpelvis = new DMatrixRMaj(3, 3);

   private final YoDouble pelvisYawAcceleration = new YoDouble("pelvisYawAcceleration", registry);
   private final YoDouble pelvisPitchAcceleration = new YoDouble("pelvisPitchAcceleration", registry);
   private final YoDouble pelvisRollAcceleration = new YoDouble("pelvisRollAcceleration", registry);

   private final YoBoolean doNullSpaceProjectionForNaturalPosture = new YoBoolean("doNullSpaceProjectionForNaturalPosture", registry);
   private final YoBoolean doNullSpaceProjectionForPelvis = new YoBoolean("doNullSpaceProjectionForPelvis", registry);

   private final FullHumanoidRobotModel fullRobotModel;

   public ControllerNaturalPostureManager(HumanoidRobotNaturalPosture robotNaturalPosture,
                                          PID3DGainsReadOnly gains,
                                          HighLevelHumanoidControllerToolbox controllerToolbox,
                                          YoRegistry parentRegistry)
   {
      controlDT = controllerToolbox.getControlDT();

      npVelocityBreakFrequency.addListener(v -> npVelocityAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(npVelocityBreakFrequency.getDoubleValue(), controlDT), false));
      npVelocityAlpha.addListener(v -> npVelocityBreakFrequency.set(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(npVelocityAlpha.getDoubleValue(), controlDT), false));
      npVelocityBreakFrequency.set(50.0);

      npYawVelocity = new FilteredVelocityYoVariable("npYawVelocity", "", npVelocityAlpha, npYaw, controlDT, registry);
      npPitchVelocity = new FilteredVelocityYoVariable("npPitchVelocity", "", npVelocityAlpha, npPitch, controlDT, registry);
      npRollVelocity = new FilteredVelocityYoVariable("npRollVelocity", "", npVelocityAlpha, npRoll, controlDT, registry);

      if (useAxisAngleFeedbackController)
      {
         yoProportionalFeedback = new YoFrameVector3D("npProportionalFeedback", ReferenceFrame.getWorldFrame(), registry);
         yoDerivativeFeedback = new YoFrameVector3D("npDerivativeFeedback", ReferenceFrame.getWorldFrame(), registry);

         feedbackNPAcceleration = new YoFrameVector3D("feedbackNPAcceleration", ReferenceFrame.getWorldFrame(), registry);

         errorRotationVector = new YoFrameVector3D("npErrorRotationVector", ReferenceFrame.getWorldFrame(), registry);

         yoDirectProportionalFeedback = null;
         yoDirectDerivativeFeedback = null;

         npYawAcceleration = null;
         npPitchAcceleration = null;
         npRollAcceleration = null;
      }
      else
      {
         yoProportionalFeedback = null;
         yoDerivativeFeedback = null;

         feedbackNPAcceleration = null;

         errorRotationVector = null;

         yoDirectProportionalFeedback = new YoFrameVector3D("npDirectProportionalFeedback", ReferenceFrame.getWorldFrame(), registry);
         yoDirectDerivativeFeedback = new YoFrameVector3D("npDirectDerivativeFeedback", ReferenceFrame.getWorldFrame(), registry);

         npYawAcceleration = new YoDouble("npYawAcceleration", registry);
         npPitchAcceleration = new YoDouble("npPitchAcceleration", registry);
         npRollAcceleration = new YoDouble("npRollAcceleration", registry);
      }
      //      this.gains = gains;
      fullRobotModel = controllerToolbox.getFullRobotModel();

      this.robotNaturalPosture = robotNaturalPosture;
      naturalPostureControlCommand.getObjective().reshape(3, 1);

      npQPobjective.reshape(3, 1);
      npQPweightMatrix.reshape(3, 3);
      npQPselectionMatrix.reshape(3, 3);
      CommonOps_DDRM.setIdentity(npQPselectionMatrix);

      // switches 
      doNullSpaceProjectionForNaturalPosture.set(false);
      doNullSpaceProjectionForPelvis.set(true);

      // Desired NP values (wrt world)
      npPitchDesired.set(-0.03);

      double qpWeight = 5.0; //5.0;
      npQPWeightX.set(1); //1
      npQPWeightY.set(1); //1
      npQPWeightZ.set(1);

      npKpYaw.set(50.0);
      npKpPitch.set(25.0);
      npKpRoll.set(50.0);

      npKdYaw.set(3.0);
      npKdPitch.set(1.0);
      npKdRoll.set(3.0);


      // Pelvis privileged pose
      double scale1 = 1;//100;
      double scale2 = 1;//0.1;

      pelvisQPObjectiveCommand.getObjective().reshape(3, 1);
      pelvisQPjacobian.reshape(3, 6 + fullRobotModel.getOneDoFJoints().length);
      pelvisQPweightMatrix.reshape(3, 3);
      pelvisQPselectionMatrix.reshape(3, 3);
      CommonOps_DDRM.setIdentity(pelvisQPselectionMatrix);
      
      pelvisQPWeightX.set(1.0 * scale1);
      pelvisQPWeightY.set(1.0 * scale1);
      pelvisQPWeightZ.set(1.0 * scale1);
      
      pPosePelvisYaw.set(0.0);
      pPosePelvisPitch.set(0.02);
      pPosePelvisRoll.set(0.0);
      pPosePelvisYawKp.set(1000.0 * scale2);
      pPosePelvisPitchKp.set(3000 * scale2);
      pPosePelvisRollKp.set(1500.0 * scale2);
      pPosePelvisYawKdFactor.set(0.15);
      pPosePelvisPitchKdFactor.set(0.15);
      pPosePelvisRollKdFactor.set(0.15);

      parentRegistry.addChild(registry);
   }

   //   public void setWeights(Vector3DReadOnly naturalPostureAngularWeight)
   //   {
   //      this.naturalPostureAngularWeight = naturalPostureAngularWeight;
   //   }

   //   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
   //   {
   //      this.selectionMatrix.set(selectionMatrix);
   //   }

   //   @Override
   //   public void doAction(double timeInState)
   public void compute()
   {
      // Update NP values
      robotNaturalPosture.compute(fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame().getRotation());

      // Set QP objective for pelvis privileged pose:
      pelvisPrivilegedPoseQPObjectiveCommand();

      // POPULATE QP MATRICES HERE:

      npQPweightMatrix.set(0, 0, npQPWeightX.getValue());
      npQPweightMatrix.set(1, 1, npQPWeightY.getValue());
      npQPweightMatrix.set(2, 2, npQPWeightZ.getValue());

      // Get current NP:   GMN - we're assuming NP compute() is getting called somewhere else?
      yoCurrentNaturalPosture.set(robotNaturalPosture.getNaturalPostureQuaternion());
      yoDesiredNaturalPosture.setYawPitchRoll(npYawDesired.getDoubleValue(), npPitchDesired.getDoubleValue(), npRollDesired.getDoubleValue());

      // Update the measured natural posture frame
      naturalPostureFrame.setOrientationAndUpdate(yoCurrentNaturalPosture);

      // The NP servo:
      npYaw.set(yoCurrentNaturalPosture.getYaw());
      npPitch.set(yoCurrentNaturalPosture.getPitch());
      npRoll.set(yoCurrentNaturalPosture.getRoll());

//      npYawDesired.set(midFeetZUpFrame.getTransformToWorldFrame().getRotation().getYaw());

      npYawVelocity.update();
      npPitchVelocity.update();
      npRollVelocity.update();



      //      double[] kp = gains.getProportionalGains();
      //      double[] kp = new double[] {100.0, 0.0, 0.0};

      if (useAxisAngleFeedbackController)
      {
         computeProportionalNPFeedback(proportionalFeedback);
         computeDerivativeNPFeedback(derivativeFeedback);

         yoProportionalFeedback.setMatchingFrame(proportionalFeedback);
         yoDerivativeFeedback.setMatchingFrame(derivativeFeedback);

         feedbackNPAcceleration.add(yoProportionalFeedback, yoDerivativeFeedback);

         yprDDot.set(0, 0, feedbackNPAcceleration.getZ());
         yprDDot.set(1, 0, feedbackNPAcceleration.getY());
         yprDDot.set(2, 0, feedbackNPAcceleration.getX());
      }
      else
      {
         yoDirectProportionalFeedback.set(npRollDesired.getDoubleValue(), npPitchDesired.getDoubleValue(), npYawDesired.getDoubleValue());
         yoDirectProportionalFeedback.sub(npRoll.getDoubleValue(), npPitch.getDoubleValue(), npYaw.getDoubleValue());
         yoDirectProportionalFeedback.scale(npKpRoll.getDoubleValue(), npKpPitch.getDoubleValue(), npKpYaw.getDoubleValue());

         yoDirectDerivativeFeedback.set(npRollVelocity.getDoubleValue(), npPitchVelocity.getDoubleValue(), npYawVelocity.getDoubleValue());
         yoDirectDerivativeFeedback.scale(-npKdRoll.getDoubleValue(), -npKdPitch.getDoubleValue(), -npKdYaw.getDoubleValue());

         npYawAcceleration.set(yoDirectProportionalFeedback.getZ() + yoDirectDerivativeFeedback.getZ());
         npPitchAcceleration.set(yoDirectProportionalFeedback.getY() + yoDirectDerivativeFeedback.getY());
         npRollAcceleration.set(yoDirectProportionalFeedback.getX() + yoDirectDerivativeFeedback.getX());

         yprDDot.set(0, 0, npYawAcceleration.getValue());
         yprDDot.set(1, 0, npPitchAcceleration.getValue());
         yprDDot.set(2, 0, npRollAcceleration.getValue());
      }

      double sbe = Math.sin(npPitch.getValue());
      double cbe = Math.cos(npPitch.getValue());
      double sal = Math.sin(npRoll.getValue());
      double cal = Math.cos(npRoll.getValue());

      // Dnp:
      //   + is *not* a 3x3 orthogonal transformation
      //   + is a velocity transformation: omega = Dnp * (Euler_rates)
      //   + the order of the Euler angles is required to derive the correct transformation
      //   + (i.e. we cannot find this transform from the quaternion only, since you also need the choice of Euler set)
      //   + the transformations for RPY & YPR are different, as the RPY Euler set is a different orientation representation
      //   + to avoid confusion, keep variable name & math consistent: keep the Y,P,R = 0,1,2 order when using YPR Euler sets
      //   + BUT the result: omega will be {x,y,z}
      Dnp.set(0,0,-sbe);     Dnp.set(0,1,0.0);   Dnp.set(0,2,1.0);
      Dnp.set(1,0, cbe*sal); Dnp.set(1,1, cal);  Dnp.set(1,2,0.0);
      Dnp.set(2,0, cbe*cal); Dnp.set(2,1,-sal);  Dnp.set(2,2,0.0);

      // TODO is this correct? I'm not sure.
      if (useAxisAngleFeedbackController)
         yoCurrentNaturalPosture.transform(feedbackNPAcceleration, yoAltNPQPObjective);
      // GMN: derivative terms???

      CommonOps_DDRM.mult(Dnp, yprDDot, npQPobjective); // GMN: missing D-dot term (since InvDyn takes accels)

      yoNPQPObjective.set(npQPobjective);

      // Populate the QPObjectiveCommand:
      naturalPostureControlCommand.setDoNullSpaceProjection(doNullSpaceProjectionForNaturalPosture.getBooleanValue());
      yoNPQPObjective.get(naturalPostureControlCommand.getObjective());
//      naturalPostureControlCommand.getObjective().set(npQPobjective);
      naturalPostureControlCommand.getJacobian().set(robotNaturalPosture.getNaturalPostureJacobian());
      naturalPostureControlCommand.getSelectionMatrix().set(npQPselectionMatrix);
      naturalPostureControlCommand.getWeightMatrix().set(npQPweightMatrix);
   }

   private final Matrix3D tempGainMatrix = new Matrix3D();

   private void computeProportionalNPFeedback(FrameVector3D feedbackTermToPack)
   {
      desiredNaturalPosture.setIncludingFrame(yoDesiredNaturalPosture);
      desiredNaturalPosture.changeFrame(naturalPostureFrame);

      desiredNaturalPosture.normalizeAndLimitToPi();
      desiredNaturalPosture.getRotationVector(feedbackTermToPack);

      errorRotationVector.setMatchingFrame(feedbackTermToPack);

      tempGainMatrix.setM00(npKpRoll.getDoubleValue());
      tempGainMatrix.setM11(npKpPitch.getDoubleValue());
      tempGainMatrix.setM22(npKpYaw.getDoubleValue());

      tempGainMatrix.transform(feedbackTermToPack);
      feedbackTermToPack.changeFrame(ReferenceFrame.getWorldFrame());
   }

   private void computeDerivativeNPFeedback(FrameVector3D feedbackTermToPack)
   {
      feedbackTermToPack.set(-npRollVelocity.getDoubleValue(), -npPitchVelocity.getDoubleValue(), -npYawVelocity.getDoubleValue());

      // TODO update the gain matrix
      tempGainMatrix.setM00(npKdRoll.getDoubleValue());
      tempGainMatrix.setM11(npKdPitch.getDoubleValue());
      tempGainMatrix.setM22(npKdYaw.getDoubleValue());

      tempGainMatrix.transform(feedbackTermToPack);
      feedbackTermToPack.changeFrame(ReferenceFrame.getWorldFrame());
   }

   //   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return naturalPostureControlCommand;
   }

   //   @Override
   //   public QPObjectiveCommand getQPObjectiveCommand()
   //   {
   //      return naturalPostureControlCommand;
   //   }

   // Implements a YPR servo on the pelvis, which is then used for the privileged
   // pose of the pelvis (via task null-space projection)
   private void pelvisPrivilegedPoseQPObjectiveCommand()
   {
      pelvisQPweightMatrix.set(0, 0, pelvisQPWeightX.getValue());
      pelvisQPweightMatrix.set(1, 1, pelvisQPWeightY.getValue());
      pelvisQPweightMatrix.set(2, 2, pelvisQPWeightZ.getValue());

      // Get current pelvis YPR and omega:
      pelvisYPR.set(fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame().getRotation());
      fullRobotModel.getPelvis().getBodyFixedFrame().getTwistOfFrame().getAngularPart().get(pelvisOmega);

      double sbe = Math.sin(pelvisYPR.getPitch());
      double cbe = Math.cos(pelvisYPR.getPitch());
      double sal = Math.sin(pelvisYPR.getRoll());
      double cal = Math.cos(pelvisYPR.getRoll());
      Dpelvis.set(0, 0, -sbe); Dpelvis.set(0, 1, 0.0);
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
      pelvisYawAcceleration.set(pPosePelvisYawKp.getValue() * (pPosePelvisYaw.getValue() - pelvisYPR.getYaw())
            - pPosePelvisYawKdFactor.getValue() * pPosePelvisYawKp.getValue() * pelvisYPRdot.get(0, 0));
      pelvisPitchAcceleration.set(pPosePelvisPitchKp.getValue() * (pPosePelvisPitch.getValue() - pelvisYPR.getPitch())
            - pPosePelvisPitchKdFactor.getValue() * pPosePelvisPitchKp.getValue() * pelvisYPRdot.get(1, 0));
      pelvisRollAcceleration.set(pPosePelvisRollKp.getValue() * (pPosePelvisRoll.getValue() - pelvisYPR.getRoll())
            - pPosePelvisRollKdFactor.getValue() * pPosePelvisRollKp.getValue() * pelvisYPRdot.get(2, 0));

      yprDDot.set(0, 0, pelvisYawAcceleration.getValue());
      yprDDot.set(1, 0, pelvisPitchAcceleration.getValue());
      yprDDot.set(2, 0, pelvisRollAcceleration.getValue());

      CommonOps_DDRM.mult(Dpelvis, yprDDot, pelvisQPobjective); // GMN: missing D-dot*yprDot term

      yoPelvisQPObjective.set(pelvisQPobjective);

      pelvisQPjacobian.zero(); // GMN: necessary??
      pelvisQPjacobian.set(0, 0, 1.0);
      pelvisQPjacobian.set(1, 1, 1.0);
      pelvisQPjacobian.set(2, 2, 1.0);

      // Populate the QPObjectiveCommand:
      yoPelvisQPObjective.get(pelvisQPObjectiveCommand.getObjective());
      pelvisQPObjectiveCommand.setDoNullSpaceProjection(doNullSpaceProjectionForPelvis.getBooleanValue());
      pelvisQPObjectiveCommand.getJacobian().set(pelvisQPjacobian);
      pelvisQPObjectiveCommand.getSelectionMatrix().set(pelvisQPselectionMatrix);
      pelvisQPObjectiveCommand.getWeightMatrix().set(pelvisQPweightMatrix);
   }

   public InverseDynamicsCommand<?> getPelvisPrivilegedPoseCommand()
   {
      return pelvisQPObjectiveCommand;
   }
}