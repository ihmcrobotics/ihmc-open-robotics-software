package us.ihmc.commonWalkingControlModules.controlModules.naturalPosture;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.configurations.HumanoidRobotNaturalPosture;
import us.ihmc.commonWalkingControlModules.configurations.NaturalPostureParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.QPObjectiveCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.referenceFrames.OrientationFrame;
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
   private final YoBoolean doNullSpaceProjectionForNaturalPosture = new YoBoolean("doNullSpaceProjectionForNaturalPosture", registry);

   private final FullHumanoidRobotModel fullRobotModel;

   // This is used for generating data for the paper.
   private NaturalPosturePaperDataComputanator naturalPosturePaperDataComputanator;

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

      // Desired NP values (wrt world)
      comAngleDesired.set(npParameters.getComAngleDesired());   // (0.0, -0.03, 0.0)

      KpComAngle.set(npParameters.getAngularComKpGains());
      KdComAngle.set(npParameters.getAngularComKdGains());

      if (generateDataForPaper)
      {
         naturalPosturePaperDataComputanator = new NaturalPosturePaperDataComputanator(robotNaturalPosture, controllerToolbox, parentRegistry);
      }

      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      // Update NP values
      robotNaturalPosture.compute(fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame().getRotation());

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
         naturalPosturePaperDataComputanator.computeDataForPaper();
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
}