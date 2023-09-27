package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import org.ejml.data.DMatrixRBlock;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.block.linsol.qr.QrHouseHolderSolver_DDRB;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.ops.ConvertDMatrixStruct;

import us.ihmc.commonWalkingControlModules.configurations.HumanoidRobotNaturalPosture;
import us.ihmc.commonWalkingControlModules.configurations.NaturalPostureParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.QPObjectiveCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.referenceFrames.OrientationFrame;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class NaturalPostureController
{
   private static final boolean generateDataForPaper = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double controlDT;

   private final QPObjectiveCommand naturalPostureControlCommand = new QPObjectiveCommand();

   private final HumanoidRobotNaturalPosture robotNaturalPosture;
   private final NaturalPostureParameters npParameters;

   private final DMatrixRMaj npQPobjective = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj npQPweightMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj npQPselectionMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj yprDDot = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj Dnp = new DMatrixRMaj(3, 3);

   private final YoDouble npVelocityAlpha = new YoDouble("npVelocityAlpha", registry);
   private final YoDouble npVelocityBreakFrequency = new YoDouble("npVelocityBreakFrequency", registry);

   private final FilteredVelocityYoVariable npYawVelocity, npPitchVelocity, npRollVelocity;

   private final YoFrameYawPitchRoll comAngle = new YoFrameYawPitchRoll("npCenterOfMassAngle", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll KpComAngle = new YoFrameYawPitchRoll("npCenterOfMassAngleKpGains", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll KdComAngle = new YoFrameYawPitchRoll("npCenterOfMassAngleKdGains", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll comAngleDesired = new YoFrameYawPitchRoll("npCenterOfMassAngleDesired", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll comAngularAcceleration = new YoFrameYawPitchRoll("npCenterOfMassAngularAcceleration",
                                                                                      ReferenceFrame.getWorldFrame(),
                                                                                      registry);

   private final YoFrameQuaternion yoCurrentNaturalPosture = new YoFrameQuaternion("currentNaturalPosture", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameQuaternion yoDesiredNaturalPosture = new YoFrameQuaternion("desiredNaturalPosture", ReferenceFrame.getWorldFrame(), registry);
   private final FrameQuaternion desiredNaturalPosture = new FrameQuaternion();

   private final YoFrameVector3D errorRotationVector = new YoFrameVector3D("npErrorRotationVector", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector3D yoProportionalFeedback = new YoFrameVector3D("npProportionalFeedback", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D yoDerivativeFeedback = new YoFrameVector3D("npDerivativeFeedback", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector3D feedbackNPAcceleration = new YoFrameVector3D("feedbackNPAcceleration", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector3D yoDirectProportionalFeedback = new YoFrameVector3D("npDirectProportionalFeedback", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D yoDirectDerivativeFeedback = new YoFrameVector3D("npDirectDerivativeFeedback", ReferenceFrame.getWorldFrame(), registry);

   private final FrameVector3D proportionalFeedback = new FrameVector3D();
   private final FrameVector3D derivativeFeedback = new FrameVector3D();

   private final OrientationFrame naturalPostureFrame = new OrientationFrame(yoCurrentNaturalPosture);

   private final QPObjectiveCommand pelvisQPObjectiveCommand = new QPObjectiveCommand();
   private final YoVector3D pelvisQPWeight = new YoVector3D("pelvisQPWeight", registry);
   private final YoFrameYawPitchRoll pelvisPrivilegedOrientation = new YoFrameYawPitchRoll("pPosePelvis",
                                                                                           ReferenceFrame.getWorldFrame(),
                                                                                           registry);
   private final YoFrameYawPitchRoll pelvisPrivilegedOrientationKp = new YoFrameYawPitchRoll("pPosePelvisKp",
                                                                                             ReferenceFrame.getWorldFrame(),
                                                                                             registry);
   private final YoFrameYawPitchRoll pelvisPrivilegedOrientationKdFactor = new YoFrameYawPitchRoll("pPosePelvisKdFactor",
                                                                                                   ReferenceFrame.getWorldFrame(),
                                                                                                   registry);
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

   private final YoBoolean doNullSpaceProjectionForNaturalPosture = new YoBoolean("doNullSpaceProjectionForNaturalPosture", registry);
   private final YoBoolean doNullSpaceProjectionForPelvis = new YoBoolean("doNullSpaceProjectionForPelvis", registry);

   private final FullHumanoidRobotModel fullRobotModel;

   // For generating data for the paper
   private final YoVector3D relativeAngularVel = new YoVector3D("relativeAngularVelZ", registry);
   private final YoVector3D omega_bc = new YoVector3D("omega_bc", registry);
   private final YoVector3D centroidalAngularMomentumApproxByACOM = new YoVector3D("centroidalAngularMomentumApproxByACOM", registry);

   public NaturalPostureController(HumanoidRobotNaturalPosture robotNaturalPosture,
                                   NaturalPostureParameters npParameters,
                                   HighLevelHumanoidControllerToolbox controllerToolbox,
                                   YoRegistry parentRegistry)
   {
      controlDT = controllerToolbox.getControlDT();
      this.robotNaturalPosture = robotNaturalPosture;
      this.npParameters = npParameters;

      if (robotNaturalPosture.getRegistry() != null)
         registry.addChild(robotNaturalPosture.getRegistry());
      robotNaturalPosture.createVisuals(controllerToolbox.getYoGraphicsListRegistry());

      npVelocityBreakFrequency.addListener(v -> npVelocityAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(npVelocityBreakFrequency.getDoubleValue(),
                                                                                                                                    controlDT), false));
      npVelocityAlpha.addListener(v -> npVelocityBreakFrequency.set(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(npVelocityAlpha.getDoubleValue(),
                                                                                                                            controlDT), false));
      npVelocityBreakFrequency.set(npParameters.getVelocityBreakFrequency());  //50

      //TODO Is there a Yaw, Pitch, Roll equivalent here? Something like FilteredVelocityYoFrameVector?
      npYawVelocity = new FilteredVelocityYoVariable("npYawVelocity", "", npVelocityAlpha, comAngle.getYoYaw(), controlDT, registry);
      npPitchVelocity = new FilteredVelocityYoVariable("npPitchVelocity", "", npVelocityAlpha, comAngle.getYoPitch(), controlDT, registry);
      npRollVelocity = new FilteredVelocityYoVariable("npRollVelocity", "", npVelocityAlpha, comAngle.getYoRoll(), controlDT, registry);

      fullRobotModel = controllerToolbox.getFullRobotModel();

      npQPobjective.reshape(3, 1);
      npQPweightMatrix.reshape(3, 3);
      npQPselectionMatrix.reshape(3, 3);
      CommonOps_DDRM.setIdentity(npQPselectionMatrix);

      //TODO wasnt this already decided in NaturalPostureManager?
      //switches
      doNullSpaceProjectionForNaturalPosture.set(true);
      doNullSpaceProjectionForPelvis.set(true);

      // Desired NP values (wrt world)
      comAngleDesired.set(npParameters.getComAngleDesired());   // (0.0, -0.03, 0.0)

      KpComAngle.set(npParameters.getAngularComKpGains());
      KdComAngle.set(npParameters.getAngularComKdGains());

      // Pelvis privileged pose
      pelvisQPobjective.reshape(3, 1);
      pelvisQPjacobian.reshape(3, 6 + fullRobotModel.getOneDoFJoints().length);
      pelvisQPweightMatrix.reshape(3, 3);
      pelvisQPselectionMatrix.reshape(3, 3);
      CommonOps_DDRM.setIdentity(pelvisQPselectionMatrix);

      pelvisQPWeight.set(npParameters.getPelvisQPWeights());
      pelvisPrivilegedOrientation.set(npParameters.getPelvisPrivilegedOrientation());
      pelvisPrivilegedOrientationKp.set(npParameters.getPelvisPrivilegedOrientation());
      pelvisPrivilegedOrientationKdFactor.set(npParameters.getPelvisPrivilegedOrientationKdFactor());

      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      // Update NP values
      robotNaturalPosture.compute(fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame().getRotation());

      // Set QP objective for pelvis privileged pose:
      pelvisPrivilegedPoseQPObjectiveCommand();

      // POPULATE QP MATRICES HERE:
      npQPweightMatrix.set(0, 0, npParameters.getQPWeights().getX());
      npQPweightMatrix.set(1, 1, npParameters.getQPWeights().getY());
      npQPweightMatrix.set(2, 2, npParameters.getQPWeights().getZ());

      // Get current NP:   GMN - we're assuming NP compute() is getting called somewhere else?
      yoCurrentNaturalPosture.set(robotNaturalPosture.getNaturalPostureQuaternion());
      yoDesiredNaturalPosture.setYawPitchRoll(comAngleDesired.getYaw(), comAngleDesired.getPitch(), comAngleDesired.getRoll());

      // Update the measured natural posture frame
      naturalPostureFrame.setOrientationAndUpdate(yoCurrentNaturalPosture);

      // The NP servo:
      comAngle.setYaw(yoCurrentNaturalPosture.getYaw());
      comAngle.setPitch(yoCurrentNaturalPosture.getPitch());
      comAngle.setRoll(yoCurrentNaturalPosture.getRoll());

      npYawVelocity.update();
      npPitchVelocity.update();
      npRollVelocity.update();

      double sbe = Math.sin(comAngle.getPitch());
      double cbe = Math.cos(comAngle.getPitch());
      double sal = Math.sin(comAngle.getRoll());
      double cal = Math.cos(comAngle.getRoll());

      // Dnp:
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

      //      double[] kp = gains.getProportionalGains();
      //      double[] kp = new double[] {100.0, 0.0, 0.0};

      yoDirectProportionalFeedback.set(comAngleDesired.getRoll(), comAngleDesired.getPitch(), comAngleDesired.getYaw());
      yoDirectProportionalFeedback.sub(comAngle.getRoll(), comAngle.getPitch(), comAngle.getYaw());
      yoDirectProportionalFeedback.scale(KpComAngle.getRoll(), KpComAngle.getPitch(), KpComAngle.getYaw());

      yoDirectDerivativeFeedback.set(npRollVelocity.getDoubleValue(), npPitchVelocity.getDoubleValue(), npYawVelocity.getDoubleValue());
      yoDirectDerivativeFeedback.scale(-KdComAngle.getRoll(), -KdComAngle.getPitch(), -KdComAngle.getYaw());

      comAngularAcceleration.setYaw(KpComAngle.getYaw() * (comAngleDesired.getYaw() - comAngle.getYaw()) - KdComAngle.getYaw() * npYawVelocity.getValue());
      comAngularAcceleration.setPitch(
            KpComAngle.getPitch() * (comAngleDesired.getPitch() - comAngle.getPitch()) - KdComAngle.getPitch() * npPitchVelocity.getValue());
      comAngularAcceleration.setRoll(
            KpComAngle.getRoll() * (comAngleDesired.getRoll() - comAngle.getRoll()) - KdComAngle.getRoll() * npRollVelocity.getValue());

      computeProportionalNPFeedback(proportionalFeedback);
      computeDerivativeNPFeedback(derivativeFeedback);

      yoProportionalFeedback.setMatchingFrame(proportionalFeedback);
      yoDerivativeFeedback.setMatchingFrame(derivativeFeedback);

      feedbackNPAcceleration.add(yoProportionalFeedback, yoDerivativeFeedback);

      // TODO use these if you want to use the axis angle feedback controller
      //            yprDDot.set(0, 0, feedbackNPAcceleration.getZ());
      //            yprDDot.set(1, 0, feedbackNPAcceleration.getY());
      //            yprDDot.set(2, 0, feedbackNPAcceleration.getX());

      yprDDot.set(0, 0, comAngularAcceleration.getYaw());
      yprDDot.set(1, 0, comAngularAcceleration.getPitch());
      yprDDot.set(2, 0, comAngularAcceleration.getRoll());

      // GMN: derivative terms???

      CommonOps_DDRM.mult(Dnp, yprDDot, npQPobjective); // GMN: missing D-dot term (since InvDyn takes accels)

      // Populate the QPObjectiveCommand:
      naturalPostureControlCommand.setDoNullSpaceProjection(doNullSpaceProjectionForNaturalPosture.getBooleanValue());
      naturalPostureControlCommand.getObjective().set(npQPobjective);
      naturalPostureControlCommand.getJacobian().set(robotNaturalPosture.getNaturalPostureJacobian());
      naturalPostureControlCommand.getSelectionMatrix().set(npQPselectionMatrix);
      naturalPostureControlCommand.getWeightMatrix().set(npQPweightMatrix);

      // For testing (data for paper)
      if (generateDataForPaper)
         computeDataForPaper();
   }

   private final Matrix3D tempGainMatrix = new Matrix3D();

   private void computeProportionalNPFeedback(FrameVector3D feedbackTermToPack)
   {
      desiredNaturalPosture.setIncludingFrame(yoDesiredNaturalPosture);
      desiredNaturalPosture.changeFrame(naturalPostureFrame);

      desiredNaturalPosture.normalizeAndLimitToPi();
      desiredNaturalPosture.getRotationVector(feedbackTermToPack);

      errorRotationVector.setMatchingFrame(feedbackTermToPack);

      tempGainMatrix.setM00(KpComAngle.getRoll());
      tempGainMatrix.setM11(KpComAngle.getPitch());
      tempGainMatrix.setM22(KpComAngle.getYaw());

      tempGainMatrix.transform(feedbackTermToPack);
      feedbackTermToPack.changeFrame(ReferenceFrame.getWorldFrame());
   }

   private void computeDerivativeNPFeedback(FrameVector3D feedbackTermToPack)
   {
      feedbackTermToPack.set(-npRollVelocity.getDoubleValue(), -npPitchVelocity.getDoubleValue(), -npYawVelocity.getDoubleValue());

      // TODO update the gain matrix
      tempGainMatrix.setM00(KdComAngle.getRoll());
      tempGainMatrix.setM11(KdComAngle.getPitch());
      tempGainMatrix.setM22(KdComAngle.getYaw());

      tempGainMatrix.transform(feedbackTermToPack);
      feedbackTermToPack.changeFrame(ReferenceFrame.getWorldFrame());
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return naturalPostureControlCommand;
   }

   //TODO should this and its getter live in NaturalPosturePrivilegedController?
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
      pelvisAngularAcceleration.setYaw(pelvisPrivilegedOrientationKp.getYaw() * (pelvisPrivilegedOrientation.getYaw() - pelvisYPR.getYaw())
                                       - pelvisPrivilegedOrientationKdFactor.getYaw() * pelvisPrivilegedOrientationKp.getYaw() * pelvisYPRdot.get(0, 0));
      pelvisAngularAcceleration.setPitch(pelvisPrivilegedOrientationKp.getPitch() * (pelvisPrivilegedOrientation.getPitch() - pelvisYPR.getPitch())
                                         - pelvisPrivilegedOrientationKdFactor.getPitch() * pelvisPrivilegedOrientationKp.getPitch() * pelvisYPRdot.get(1, 0));
      pelvisAngularAcceleration.setRoll(pelvisPrivilegedOrientationKp.getRoll() * (pelvisPrivilegedOrientation.getRoll() - pelvisYPR.getRoll())
                                        - pelvisPrivilegedOrientationKdFactor.getRoll() * pelvisPrivilegedOrientationKp.getRoll() * pelvisYPRdot.get(2, 0));

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

   ///////////////////// methods for generating data for paper

   //TODO pull this out into its own class
   private void computeDataForPaper()
   {
      // We would like to compare the relative angular velocity to the angular velocity of the ACOM frame
      // We also want to compare the CAM to the one approximated by ACOM

      fullRobotModel.getElevator().updateFramesRecursively();
      MomentumData momentumData = computeMomentum(fullRobotModel);

      DMatrixRMaj relativeVel = MatrixTools.mult(momentumData.connectionMatrix, momentumData.jointVelocity);

      relativeAngularVel.setX(relativeVel.get(0));
      relativeAngularVel.setY(relativeVel.get(1));
      relativeAngularVel.setZ(relativeVel.get(2));

      DMatrixRMaj b_omega_bc = MatrixTools.mult(robotNaturalPosture.getNaturalPostureJacobianRtBaseEwrtBase(), momentumData.jointVelocity);
      omega_bc.setX(b_omega_bc.get(0));
      omega_bc.setY(b_omega_bc.get(1));
      omega_bc.setZ(b_omega_bc.get(2));

      DMatrixRMaj b_omega_wb = new DMatrixRMaj(3, 1);
      b_omega_wb.set(0, momentumData.jointVelocityWithFloatingBase.get(0));
      b_omega_wb.set(1, momentumData.jointVelocityWithFloatingBase.get(1));
      b_omega_wb.set(2, momentumData.jointVelocityWithFloatingBase.get(2));
      DMatrixRMaj b_omega_wc = new DMatrixRMaj(3, 1);
      b_omega_wc.set(0, b_omega_wb.get(0) + b_omega_bc.get(0));
      b_omega_wc.set(1, b_omega_wb.get(1) + b_omega_bc.get(1));
      b_omega_wc.set(2, b_omega_wb.get(2) + b_omega_bc.get(2));

      DMatrixRMaj centroidalMomentumApproxByACOM = new DMatrixRMaj(6, 1);
      DMatrixRMaj Mbase = new DMatrixRMaj(6, 3);
      int[] srcColumnsBase = {0, 1, 2};
      MatrixTools.extractColumns(momentumData.momentumMatrix, srcColumnsBase, Mbase, 0);
      centroidalMomentumApproxByACOM = MatrixTools.mult(Mbase, b_omega_wc);

      centroidalAngularMomentumApproxByACOM.setX(centroidalMomentumApproxByACOM.get(0));
      centroidalAngularMomentumApproxByACOM.setY(centroidalMomentumApproxByACOM.get(1));
      centroidalAngularMomentumApproxByACOM.setZ(centroidalMomentumApproxByACOM.get(2));
   }

   private static class MomentumData
   {
      public DMatrixRMaj jointPositionWithFloatingBase;
      public DMatrixRMaj jointVelocityWithFloatingBase;
      public DMatrixRMaj jointPosition;
      public DMatrixRMaj jointVelocity;
      public DMatrixRMaj momentumMatrix;
      public DMatrixRMaj momentumVector;
      public DMatrixRMaj connectionMatrix;

      public MomentumData(DMatrixRMaj jointPositionWithFloatingBase,
                          DMatrixRMaj jointVelocityWithFloatingBase,
                          DMatrixRMaj jointPosition,
                          DMatrixRMaj jointVelocity,
                          DMatrixRMaj momentumMatrix,
                          DMatrixRMaj momentumVector,
                          DMatrixRMaj connectionMatrix)
      {
         this.jointPositionWithFloatingBase = jointPositionWithFloatingBase;
         this.jointVelocityWithFloatingBase = jointVelocityWithFloatingBase;
         this.jointPosition = jointPosition;
         this.jointVelocity = jointVelocity;
         this.momentumMatrix = momentumMatrix;
         this.momentumVector = momentumVector;
         this.connectionMatrix = connectionMatrix;
      }
   }

   private MomentumData computeMomentum(FullHumanoidRobotModel fullRobotModel)
   {
      // TODO: double check if the joint ordering is the same as centroidal momentum matirx's.
      // TODO: also double check if you should use pelvis as root or elevator
      //       Ans: I believe if we want to get the momentum rt world frame, then we should use elevator, because the floating base vel also contribute to the centroidal angular momentum

      JointBasics[] jointListWithFloatingBase = MultiBodySystemTools.collectSubtreeJoints(fullRobotModel.getElevator());
      DMatrixRMaj jointPositionWithFloatingBase = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(jointListWithFloatingBase) + 1, 1);
      MultiBodySystemTools.extractJointsState(jointListWithFloatingBase, JointStateType.CONFIGURATION, jointPositionWithFloatingBase);
      DMatrixRMaj jointVelocityWithFloatingBase = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(jointListWithFloatingBase), 1);
      MultiBodySystemTools.extractJointsState(jointListWithFloatingBase, JointStateType.VELOCITY, jointVelocityWithFloatingBase);
      System.out.println(jointPositionWithFloatingBase);

      JointBasics[] jointList = MultiBodySystemTools.collectSubtreeJoints(fullRobotModel.getPelvis());
      DMatrixRMaj jointPosition = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(jointList), 1);
      MultiBodySystemTools.extractJointsState(jointList, JointStateType.CONFIGURATION, jointPosition);
      DMatrixRMaj jointVelocity = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(jointList), 1);
      MultiBodySystemTools.extractJointsState(jointList, JointStateType.VELOCITY, jointVelocity);

      //TODO we can get this from the HLHControllerToolbox via the contained internal objects.
      DMatrixRMaj momentumMatrix = computeMomentumMatrix(fullRobotModel);
      DMatrixRMaj comMomentum = MatrixTools.mult(momentumMatrix, jointVelocityWithFloatingBase);

      // Get connection matrix
      DMatrixRMaj Mbase = new DMatrixRMaj(6, 6);
      DMatrixRMaj Mq = new DMatrixRMaj(6, jointPosition.getNumRows());
      DMatrixRMaj Mconnection = new DMatrixRMaj(6, jointPosition.getNumRows());
      DMatrixRBlock MbaseBlock = new DMatrixRBlock(6, 6);
      DMatrixRBlock MqBlock = new DMatrixRBlock(6, jointPosition.getNumRows());
      DMatrixRBlock MconnectionBlock = new DMatrixRBlock(6, jointPosition.getNumRows());

      int[] srcColumnsBase = {0, 1, 2, 3, 4, 5};
      int[] srcColumnsQ = {6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34};
      MatrixTools.extractColumns(momentumMatrix, srcColumnsBase, Mbase, 0);
      MatrixTools.extractColumns(momentumMatrix, srcColumnsQ, Mq, 0);

      // Ref:
      // https://www.tabnine.com/code/java/methods/org.ejml.ops.ConvertDMatrixStruct/convert
      ConvertDMatrixStruct.convert(Mbase, MbaseBlock);
      ConvertDMatrixStruct.convert(Mq, MqBlock);

      // Ref:
      // https://ejml.org/javadoc/org/ejml/dense/block/linsol/qr/QrHouseHolderSolver_DDRB.html
      QrHouseHolderSolver_DDRB solver = new QrHouseHolderSolver_DDRB();
      solver.setA(MbaseBlock);
      solver.solve(MqBlock, MconnectionBlock);
      ConvertDMatrixStruct.convert(MconnectionBlock, Mconnection);

      //		System.out.println("----------");
      //		System.out.println(Mq);
      //		System.out.println(MatrixTools.mult(Mbase, Mconnection));

      //		System.out.println("----------");
      //		System.out.println(Mbase);

      //		System.out.println("jointListWithFloatingBase");
      //		for (int i = 0; i < jointListWithFloatingBase.length; i++) {
      //			System.out.print(jointListWithFloatingBase[i]);
      //			System.out.println("");
      //		}
      //		System.out.println("jointList");
      //		for (int i = 0; i < jointList.length; i++) {
      //			System.out.print(jointList[i]);
      //			System.out.println("");
      //		}

      //		System.out.println(jointVelocities);
      //		System.out.println(Mbase);
      //		System.out.println(Mq);
      //    System.out.println(comMomentum);

      System.out.println("momentumMatrix = ");
      System.out.println(momentumMatrix);
      System.out.println("Mconnection = ");
      System.out.println(Mconnection);

      //		System.out.println("");
      //		System.out.println(jointPositionWithFloatingBase.getNumRows());
      //		System.out.println(jointPositionWithFloatingBase.getNumCols());
      //		System.out.println(jointVelocityWithFloatingBase.getNumRows());
      //		System.out.println(jointVelocityWithFloatingBase.getNumCols());
      //		System.out.println(jointPosition.getNumRows());
      //		System.out.println(jointPosition.getNumCols());
      //		System.out.println(jointVelocity.getNumRows());
      //		System.out.println(jointVelocity.getNumCols());
      //		System.out.println(momentumMatrix.getNumRows());
      //		System.out.println(momentumMatrix.getNumCols());
      //		System.out.println(comMomentum.getNumRows());
      //		System.out.println(comMomentum.getNumCols());

      return new MomentumData(jointPositionWithFloatingBase,
                              jointVelocityWithFloatingBase,
                              jointPosition,
                              jointVelocity,
                              momentumMatrix,
                              comMomentum,
                              Mconnection);
   }

   private DMatrixRMaj computeMomentumMatrix(FullHumanoidRobotModel fullRobotModel)
   {
      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());
      centerOfMassFrame.update();
      CentroidalMomentumCalculator centroidalMomentumMatrix = new CentroidalMomentumCalculator(fullRobotModel.getElevator(), centerOfMassFrame);
      //      CentroidalMomentumCalculator centroidalMomentumMatrix = new CentroidalMomentumCalculator(fullRobotModel.getPelvis(), centerOfMassFrame);
      fullRobotModel.getElevator().updateFramesRecursively();

      // System.out.println(centroidalMomentumMatrix.getCentroidalMomentumMatrix());

      return centroidalMomentumMatrix.getCentroidalMomentumMatrix();
   }

   public HumanoidRobotNaturalPosture getHumanoidRobotNaturalPosture()
   {
      return robotNaturalPosture;
   }

   public NaturalPostureParameters getNaturalPostureParameters()
   {
      return npParameters;
   }
}