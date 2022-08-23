package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.configurations.HumanoidRobotNaturalPosture;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.QPObjectiveCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.YawPitchRollTools;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.referenceFrames.OrientationFrame;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ControllerNaturalPostureManager
{
   private static final boolean useAxisAngleFeedbackController = true;
   private static final boolean createBothFeedbackControllers = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double controlDT;

   private final QPObjectiveCommand naturalPostureControlCommand = new QPObjectiveCommand();

   HumanoidRobotNaturalPosture robotNaturalPosture;
   private final DMatrixRMaj npQPobjective = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj npQPweightMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj npQPselectionMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj yprDDot = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj Dnp = new DMatrixRMaj(3, 3);
   private final RotationMatrix eulerAngleToBodyAxisTransform = new RotationMatrix();

   private final YoVector3D yoNPQPObjective = new YoVector3D("npQPObjective", registry);
   private final YoVector3D yoNPQPAchieved = new YoVector3D("yoNPQPAchieved", registry);
   private final YoVector3D yoAxisAngleNPQPObjective = new YoVector3D("axisAngleNpQPObjective", registry);

   private final YoDouble npYaw = new YoDouble("npYaw", registry);
   private final YoDouble npPitch = new YoDouble("npPitch", registry);
   private final YoDouble npRoll = new YoDouble("npRoll", registry);

   private final YoDouble npVelocityAlpha = new YoDouble("npVelocityAlpha", registry);
   private final YoDouble npVelocityBreakFrequency = new YoDouble("npVelocityBreakFrequency", registry);

   private final FilteredVelocityYoVariable npYawVelocity, npPitchVelocity, npRollVelocity;

   private final YoDouble npKpYaw = new YoDouble("npKpYaw", registry);
   private final YoDouble npKpPitch = new YoDouble("npKpPitch", registry);
   private final YoDouble npKpRoll = new YoDouble("npKpRoll", registry);

   private final YoDouble npZetaYaw = new YoDouble("npZetaYaw", registry);
   private final YoDouble npZetaPitch = new YoDouble("npZetaPitch", registry);
   private final YoDouble npZetaRoll = new YoDouble("npZetaRoll", registry);

   private final YoDouble npQPWeightX = new YoDouble("npQPWeightX", registry);
   private final YoDouble npQPWeightY = new YoDouble("npQPWeightY", registry);
   private final YoDouble npQPWeightZ = new YoDouble("npQPWeightZ", registry);

   private final YoFrameQuaternion yoCurrentNaturalPosture;
   private final YoFrameYawPitchRoll yoDesiredNaturalPosture;
   private final FrameQuaternion desiredNaturalPosture = new FrameQuaternion();

   // These are the variables used when using the axis angle error based controller;
   private final YoFrameVector3D yoProportionalFeedback;
   private final YoFrameVector3D yoDerivativeFeedback;

   private final YoFrameVector3D feedbackNPAcceleration;

   private final YoFrameVector3D errorRotationVector;
   private final YoFrameVector3D errorAngularVelocity;

   // These are the variables when using the euler angle error based controller
   private final YoFrameVector3D yoDirectEulerProportionalFeedback;
   private final YoFrameVector3D yoDirectEulerDerivativeFeedback;

   private final YoDouble npYawAcceleration;
   private final YoDouble npPitchAcceleration;
   private final YoDouble npRollAcceleration;

   // Temp variales
   private final FrameVector3D proportionalFeedback = new FrameVector3D();
   private final FrameVector3D derivativeFeedback = new FrameVector3D();

   private final OrientationFrame naturalPostureFrame;

   private final QPObjectiveCommand pelvisQPObjectiveCommand = new QPObjectiveCommand();

   private final YoDouble pelvisQPWeightX = new YoDouble("pelvisQPWeightX", registry);
   private final YoDouble pelvisQPWeightY = new YoDouble("pelvisQPWeightY", registry);
   private final YoDouble pelvisQPWeightZ = new YoDouble("pelvisQPWeightZ", registry);

   private final YoFrameYawPitchRoll pPosePelvis = new YoFrameYawPitchRoll("pPosePelvis", ReferenceFrame.getWorldFrame(), registry);
   private final FrameQuaternion desiredPelvisPose = new FrameQuaternion();

   private final YoDouble pPosePelvisYawKp = new YoDouble("pPosePelvisYawKp", registry);
   private final YoDouble pPosePelvisPitchKp = new YoDouble("pPosePelvisPitchKp", registry);
   private final YoDouble pPosePelvisRollKp = new YoDouble("pPosePelvisRollKp", registry);
   private final YoDouble pPosePelvisYawKdFactor = new YoDouble("pPosePelvisYawKdFactor", registry);
   private final YoDouble pPosePelvisPitchKdFactor = new YoDouble("pPosePelvisPitchKdFactor", registry);
   private final YoDouble pPosePelvisRollKdFactor = new YoDouble("pPosePelvisRollKdFactor", registry);

   private final YoVector3D yoPelvisProportionalFeedback = new YoVector3D("pelvisProportionalFeedback", registry);
   private final YoVector3D yoPelvisDerivativeFeedback = new YoVector3D("pelvisDerivativeFeedback", registry);
   private final YoVector3D yoPelvisQPObjective = new YoVector3D("pelvisQPObjective", registry);

   private final DMatrixRMaj pelvisQPselectionMatrix = new DMatrixRMaj(1, 1);

   private final FrameVector3D pelvisProportionalFeedback = new FrameVector3D();
   private final FrameVector3D pelvisDerivativeFeedback = new FrameVector3D();

   private final YoBoolean doNullSpaceProjectionForNaturalPosture = new YoBoolean("doNullSpaceProjectionForNaturalPosture", registry);
   private final YoBoolean doNullSpaceProjectionForPelvis = new YoBoolean("doNullSpaceProjectionForPelvis", registry);

   private final FullHumanoidRobotModel fullRobotModel;
   private final ReferenceFrame walkingTrajectoryFrame;


   public ControllerNaturalPostureManager(HumanoidRobotNaturalPosture robotNaturalPosture,
                                          PID3DGainsReadOnly gains,
                                          HighLevelHumanoidControllerToolbox controllerToolbox,
                                          YoRegistry parentRegistry)
   {
      controlDT = controllerToolbox.getControlDT();

      ReferenceFrame pelvisZUpFrame = controllerToolbox.getPelvisZUpFrame();
      walkingTrajectoryFrame = controllerToolbox.getWalkingTrajectoryPath().getWalkingTrajectoryPathFrame();
      yoCurrentNaturalPosture = new YoFrameQuaternion("currentNaturalPosture", walkingTrajectoryFrame, registry);
      yoDesiredNaturalPosture = new YoFrameYawPitchRoll("desiredNaturalPosture", walkingTrajectoryFrame, registry);

      yoDesiredNaturalPosture.setToPitchOrientation(-0.1);
      naturalPostureFrame = new OrientationFrame(yoCurrentNaturalPosture);

      npVelocityBreakFrequency.addListener(v -> npVelocityAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(npVelocityBreakFrequency.getDoubleValue(),
                                                                                                                                    controlDT), false));
      npVelocityAlpha.addListener(v -> npVelocityBreakFrequency.set(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(npVelocityAlpha.getDoubleValue(),
                                                                                                                            controlDT), false));
      npVelocityBreakFrequency.set(10.0);

      npYawVelocity = new FilteredVelocityYoVariable("npYawVelocity", "", npVelocityAlpha, npYaw, controlDT, registry);
      npPitchVelocity = new FilteredVelocityYoVariable("npPitchVelocity", "", npVelocityAlpha, npPitch, controlDT, registry);
      npRollVelocity = new FilteredVelocityYoVariable("npRollVelocity", "", npVelocityAlpha, npRoll, controlDT, registry);

      if (useAxisAngleFeedbackController || createBothFeedbackControllers)
      {
         yoProportionalFeedback = new YoFrameVector3D("npProportionalFeedback", ReferenceFrame.getWorldFrame(), registry);
         yoDerivativeFeedback = new YoFrameVector3D("npDerivativeFeedback",  ReferenceFrame.getWorldFrame(), registry);

         feedbackNPAcceleration = new YoFrameVector3D("feedbackNPAcceleration", ReferenceFrame.getWorldFrame(), registry);

         errorRotationVector = new YoFrameVector3D("npErrorRotationVector", naturalPostureFrame, registry);
         errorAngularVelocity = new YoFrameVector3D("npErrorAngularVelocity", naturalPostureFrame, registry);
      }
      else
      {
         yoProportionalFeedback = null;
         yoDerivativeFeedback = null;

         feedbackNPAcceleration = null;

         errorRotationVector = null;
      }

      if (!useAxisAngleFeedbackController || createBothFeedbackControllers)
      {
         yoDirectEulerProportionalFeedback = new YoFrameVector3D("npDirectEulerProportionalFeedback", ReferenceFrame.getWorldFrame(), registry);
         yoDirectEulerDerivativeFeedback = new YoFrameVector3D("npDirectEulerDerivativeFeedback", ReferenceFrame.getWorldFrame(), registry);

         npYawAcceleration = new YoDouble("npYawAcceleration", registry);
         npPitchAcceleration = new YoDouble("npPitchAcceleration", registry);
         npRollAcceleration = new YoDouble("npRollAcceleration", registry);
      }
      else
      {

         yoDirectEulerProportionalFeedback = null;
         yoDirectEulerDerivativeFeedback = null;

         npYawAcceleration = null;
         npPitchAcceleration = null;
         npRollAcceleration = null;
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
      desiredNaturalPosture.setToPitchOrientation(-0.0);

      double qpWeight = 5.0; //5.0;
      npQPWeightX.set(1); //1
      npQPWeightY.set(1); //1
      npQPWeightZ.set(0.2);

      npKpYaw.set(50.0);
      npKpPitch.set(25.0);
      npKpRoll.set(50.0);

      npZetaYaw.set(0.4);
      npZetaPitch.set(0.4);
      npZetaRoll.set(0.4);

      // Pelvis privileged pose
      double scale1 = 1;//100;
      double scale2 = 1;//0.1;

      pelvisQPObjectiveCommand.getObjective().reshape(3, 1);
      pelvisQPselectionMatrix.reshape(3, 3);
      CommonOps_DDRM.setIdentity(pelvisQPselectionMatrix);

      pelvisQPWeightX.set(1.0 * scale1);
      pelvisQPWeightY.set(1.0 * scale1);
      pelvisQPWeightZ.set(1.0 * scale1);

      pPosePelvis.setYawPitchRoll(0.0, 0.02, 0.0);

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

   private final FrameQuaternion naturalPosture = new FrameQuaternion();
   private final FrameQuaternion baseOrientation = new FrameQuaternion();
   //   @Override
   //   public void doAction(double timeInState)
   public void compute()
   {
      baseOrientation.setToZero(fullRobotModel.getPelvis().getBodyFixedFrame());
      // TODO this is likely wrong. We want it to turn.
      baseOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      // Update NP values
      robotNaturalPosture.compute(baseOrientation);

      // Set QP objective for pelvis privileged pose:
      pelvisPrivilegedPoseQPObjectiveCommand();

      // POPULATE QP MATRICES HERE:

      npQPweightMatrix.set(0, 0, npQPWeightX.getValue());
      npQPweightMatrix.set(1, 1, npQPWeightY.getValue());
      npQPweightMatrix.set(2, 2, npQPWeightZ.getValue());

      // Get current NP:   GMN - we're assuming NP compute() is getting called somewhere else?
      naturalPosture.setIncludingFrame(ReferenceFrame.getWorldFrame(), robotNaturalPosture.getNaturalPostureQuaternion());
      yoCurrentNaturalPosture.set(robotNaturalPosture.getNaturalPostureQuaternion());

      // Update the measured natural posture frame
      naturalPostureFrame.setOrientationAndUpdate(yoCurrentNaturalPosture);

      // The NP servo:
      npYaw.set(yoCurrentNaturalPosture.getYaw());
      npPitch.set(yoCurrentNaturalPosture.getPitch());
      npRoll.set(yoCurrentNaturalPosture.getRoll());

      npYawVelocity.update();
      npPitchVelocity.update();
      npRollVelocity.update();

      if (useAxisAngleFeedbackController || createBothFeedbackControllers)
      {
         computeEulerAngleRateToBodyAxisRateTransform(npYaw.getDoubleValue(), npPitch.getDoubleValue(), npRoll.getDoubleValue(), eulerAngleToBodyAxisTransform);

         computeProportionalNPFeedback(proportionalFeedback);
         computeDerivativeNPFeedback(derivativeFeedback);

         yoProportionalFeedback.setMatchingFrame(proportionalFeedback);
         yoDerivativeFeedback.setMatchingFrame(derivativeFeedback);

         feedbackNPAcceleration.add(yoProportionalFeedback, yoDerivativeFeedback);

         yoAxisAngleNPQPObjective.set(feedbackNPAcceleration);
      }

      if (!useAxisAngleFeedbackController || createBothFeedbackControllers)
      {
         yoDirectEulerProportionalFeedback.set(yoDesiredNaturalPosture.getRoll(), yoDesiredNaturalPosture.getPitch(), yoDesiredNaturalPosture.getYaw());
         yoDirectEulerProportionalFeedback.sub(npRoll.getDoubleValue(), npPitch.getDoubleValue(), npYaw.getDoubleValue());
         yoDirectEulerProportionalFeedback.scale(npKpRoll.getDoubleValue(), npKpPitch.getDoubleValue(), npKpYaw.getDoubleValue());

         yoDirectEulerDerivativeFeedback.set(npRollVelocity.getDoubleValue(), npPitchVelocity.getDoubleValue(), npYawVelocity.getDoubleValue());
         yoDirectEulerDerivativeFeedback.scale(-GainCalculator.computeDerivativeGain(npKpRoll.getDoubleValue(), npZetaRoll.getDoubleValue()),
                                               -GainCalculator.computeDerivativeGain(npKpPitch.getDoubleValue(), npZetaPitch.getDoubleValue()),
                                               -GainCalculator.computeDerivativeGain(npKpYaw.getDoubleValue(), npZetaYaw.getDoubleValue()));

         npYawAcceleration.set(yoDirectEulerProportionalFeedback.getZ() + yoDirectEulerDerivativeFeedback.getZ());
         npPitchAcceleration.set(yoDirectEulerProportionalFeedback.getY() + yoDirectEulerDerivativeFeedback.getY());
         npRollAcceleration.set(yoDirectEulerProportionalFeedback.getX() + yoDirectEulerDerivativeFeedback.getX());

         double sbe = Math.sin(npPitch.getValue());
         double cbe = Math.cos(npPitch.getValue());
         double sal = Math.sin(npRoll.getValue());
         double cal = Math.cos(npRoll.getValue());

         // Dnp:
         //  http://www.stengel.mycpanel.princeton.edu/Quaternions.pdf
         //   + is *not* a 3x3 orthogonal transformation
         //   + is a velocity transformation: omega = Dnp * (Euler_rates)
         //   + the order of the Euler angles is required to derive the correct transformation
         //   + (i.e. we cannot find this transform from the quaternion only, since you also need the choice of Euler set)
         //   + the transformations for RPY & YPR are different, as the RPY Euler set is a different orientation representation
         //   + to avoid confusion, keep variable name & math consistent: keep the Y,P,R = 0,1,2 order when using YPR Euler sets
         //   + BUT the result: omega will be {x,y,z}

         Dnp.set(0, 0, -sbe);
         Dnp.set(0, 1, 0.0);
         Dnp.set(0, 2, 1.0);
         Dnp.set(1, 0, cbe * sal);
         Dnp.set(1, 1, cal);
         Dnp.set(1, 2, 0.0);
         Dnp.set(2, 0, cbe * cal);
         Dnp.set(2, 1, -sal);
         Dnp.set(2, 2, 0.0);

         yprDDot.set(0, 0, npYawAcceleration.getValue());
         yprDDot.set(1, 0, npPitchAcceleration.getValue());
         yprDDot.set(2, 0, npRollAcceleration.getValue());

         // GMN: derivative terms???
         CommonOps_DDRM.mult(Dnp, yprDDot, npQPobjective); // GMN: missing D-dot term (since InvDyn takes accels)

         yoNPQPObjective.set(npQPobjective);
      }




      // Populate the QPObjectiveCommand:
      naturalPostureControlCommand.setDoNullSpaceProjection(doNullSpaceProjectionForNaturalPosture.getBooleanValue());
      if (useAxisAngleFeedbackController)
      {
         yoAxisAngleNPQPObjective.get(naturalPostureControlCommand.getObjective());
      }
      else
      {
         yoNPQPObjective.get(naturalPostureControlCommand.getObjective());
      }
      //      naturalPostureControlCommand.getObjective().set(npQPobjective);
      naturalPostureControlCommand.getJacobian().set(robotNaturalPosture.getNaturalPostureJacobian());
      naturalPostureControlCommand.getSelectionMatrix().set(npQPselectionMatrix);
      naturalPostureControlCommand.getWeightMatrix().set(npQPweightMatrix);
   }

   private final Matrix3D tempGainMatrix = new Matrix3D();

   private void computeProportionalNPFeedback(FrameVector3D feedbackTermToPack)
   {
      // compute the rotation error from the current to the desired expressed in the natural posture frame.
      desiredNaturalPosture.setIncludingFrame(yoDesiredNaturalPosture);
      desiredNaturalPosture.changeFrame(naturalPostureFrame);

      desiredNaturalPosture.normalizeAndLimitToPi();
      desiredNaturalPosture.getRotationVector(feedbackTermToPack);

      errorRotationVector.set(feedbackTermToPack);

      // change to the world frame, as that's the frame of the gains
      feedbackTermToPack.changeFrame(ReferenceFrame.getWorldFrame());

      // These are the roll-pitch-yaw gains expressed in the world
      tempGainMatrix.setM00(npKpRoll.getDoubleValue());
      tempGainMatrix.setM11(npKpPitch.getDoubleValue());
      tempGainMatrix.setM22(npKpYaw.getDoubleValue());

      tempGainMatrix.transform(feedbackTermToPack);

      // Change the feedback term back to the natural posture frame, as that's the frame of expression for the task Jacobian
      feedbackTermToPack.changeFrame(naturalPostureFrame);
   }

   private void computeDerivativeNPFeedback(FrameVector3D feedbackTermToPack)
   {
      feedbackTermToPack.set(-npRollVelocity.getDoubleValue(), -npPitchVelocity.getDoubleValue(), -npYawVelocity.getDoubleValue());
      // These rates are currently in euler angle frame, we need to transform them to the natural posture frame.
      eulerAngleToBodyAxisTransform.transform(feedbackTermToPack);
      feedbackTermToPack.setReferenceFrame(naturalPostureFrame);

      errorAngularVelocity.set(feedbackTermToPack);

      // change to the world frame, as that's the frame of the gains.
      feedbackTermToPack.changeFrame(ReferenceFrame.getWorldFrame());

      // These are the roll-pitch-yaw gains expressed in the world
      tempGainMatrix.setM00(-GainCalculator.computeDerivativeGain(npKpRoll.getDoubleValue(), npZetaRoll.getDoubleValue()));
      tempGainMatrix.setM11(-GainCalculator.computeDerivativeGain(npKpPitch.getDoubleValue(), npZetaPitch.getDoubleValue()));
      tempGainMatrix.setM22(-GainCalculator.computeDerivativeGain(npKpYaw.getDoubleValue(), npZetaYaw.getDoubleValue()));

      tempGainMatrix.transform(feedbackTermToPack);

      // Change the feedback term back to the natural posture frame, as that's the frame of expression for the task Jacobian
      feedbackTermToPack.changeFrame(naturalPostureFrame);
   }

   private final DMatrixRMaj acceleration = new DMatrixRMaj(3, 1);

   public void computeAchievedAcceleration(DMatrixRMaj jointAccelerations)
   {
      CommonOps_DDRM.mult(robotNaturalPosture.getNaturalPostureJacobian(), jointAccelerations, acceleration);
      yoNPQPAchieved.set(acceleration);
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
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      //---- Compute the proportional feedback ----//
      desiredPelvisPose.setIncludingFrame(pPosePelvis);
      desiredPelvisPose.changeFrame(pelvis.getBodyFixedFrame());

      desiredPelvisPose.normalizeAndLimitToPi();
      desiredPelvisPose.getRotationVector(pelvisProportionalFeedback);

      // change to the world frame, as that's the frame of the gains.
      pelvisProportionalFeedback.changeFrame(ReferenceFrame.getWorldFrame());

      // These are the roll-pitch-yaw gains expressed in the world
      tempGainMatrix.setM00(pPosePelvisRollKp.getDoubleValue());
      tempGainMatrix.setM11(pPosePelvisPitchKp.getDoubleValue());
      tempGainMatrix.setM22(pPosePelvisYawKp.getDoubleValue());

      tempGainMatrix.transform(pelvisProportionalFeedback);

      // change feedback back to pelvis frame
      pelvisProportionalFeedback.changeFrame(pelvis.getBodyFixedFrame());

      //---- Compute the derivative feedback ----//
      pelvisDerivativeFeedback.setIncludingFrame(fullRobotModel.getPelvis().getBodyFixedFrame().getTwistOfFrame().getAngularPart());
      pelvisDerivativeFeedback.scale(-1.0);

      pelvisDerivativeFeedback.changeFrame(ReferenceFrame.getWorldFrame());

      // These are the roll-pitch-yaw gains expressed in the world
      tempGainMatrix.setM00(pPosePelvisRollKp.getDoubleValue() * pPosePelvisRollKdFactor.getValue());
      tempGainMatrix.setM11(pPosePelvisPitchKp.getDoubleValue() * pPosePelvisPitchKdFactor.getValue());
      tempGainMatrix.setM22(pPosePelvisYawKp.getDoubleValue() * pPosePelvisYawKdFactor.getValue());

      tempGainMatrix.transform(pelvisDerivativeFeedback);

      // change feedback back to pelvis frame
      pelvisDerivativeFeedback.changeFrame(pelvis.getBodyFixedFrame());

      // add the feedback terms in
      yoPelvisProportionalFeedback.set(pelvisProportionalFeedback);
      yoPelvisDerivativeFeedback.set(pelvisDerivativeFeedback);
      yoPelvisQPObjective.add(yoPelvisProportionalFeedback, yoPelvisDerivativeFeedback);

      //---- Populate the QPObjectiveCommand:
      yoPelvisQPObjective.get(pelvisQPObjectiveCommand.getObjective());
      pelvisQPObjectiveCommand.setDoNullSpaceProjection(doNullSpaceProjectionForPelvis.getBooleanValue());

      pelvisQPObjectiveCommand.getJacobian().reshape(3, 3);
      CommonOps_DDRM.setIdentity(pelvisQPObjectiveCommand.getJacobian());

      pelvisQPObjectiveCommand.getWeightMatrix().reshape(3, 3);
      pelvisQPObjectiveCommand.getWeightMatrix().set(0, 0, pelvisQPWeightX.getValue());
      pelvisQPObjectiveCommand.getWeightMatrix().set(1, 1, pelvisQPWeightY.getValue());
      pelvisQPObjectiveCommand.getWeightMatrix().set(2, 2, pelvisQPWeightZ.getValue());

      pelvisQPObjectiveCommand.getSelectionMatrix().set(pelvisQPselectionMatrix);
   }

   public InverseDynamicsCommand<?> getPelvisPrivilegedPoseCommand()
   {
      return pelvisQPObjectiveCommand;
   }

   private static void computeEulerAngleRateToBodyAxisRateTransform(double yaw, double pitch, double roll, CommonMatrix3DBasics matrixToPack)
   {
      if (EuclidCoreTools.containsNaN(yaw, pitch, roll))
      {
         matrixToPack.setToNaN();
         return;
      }

      if (YawPitchRollTools.isZero(yaw, pitch, roll, RotationMatrixConversion.EPS))
      {
         matrixToPack.setIdentity();
         return;
      }

      double sinb = EuclidCoreTools.sin(pitch);
      double cosb = EuclidCoreTools.cos(pitch);

      double cosa = EuclidCoreTools.cos(roll);
      double sina = EuclidCoreTools.sin(roll);

      // http://www.stengel.mycpanel.princeton.edu/Quaternions.pdf
      double m00 = 1.0;
      double m01 = 0.0;
      double m02 = -sinb;
      double m10 = 0.0;
      double m11 = cosa;
      double m12 = sina * cosb;
      double m20 = 0.0;
      double m21 = -sina;
      double m22 = cosa * cosb;
      if (matrixToPack instanceof RotationMatrixBasics)
         ((RotationMatrixBasics) matrixToPack).setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      else
         matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }
}