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
   private final NaturalPostureParameters parameters;

   private final DMatrixRMaj qpObjective = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj weightMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj selectionMatrix = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj yawPitchRollDoubleDot = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj Dnp = new DMatrixRMaj(3, 3);

   private final YoDouble velocityAlpha = new YoDouble("naturalPostureVelocityAlpha", registry);
   private final YoDouble velocityBreakFrequency = new YoDouble("naturalPostureVelocityBreakFrequency", registry);

   private final FilteredVelocityYoVariable comAngularVelocityYaw, comAngularVelocityPitch, comAngularVelocityRoll;

   private final YoFrameYawPitchRoll comAngle = new YoFrameYawPitchRoll("naturalPostureCenterOfMassAngle", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll kpComAngle = new YoFrameYawPitchRoll("naturalPostureCenterOfMassAngleKpGains", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll kdComAngle = new YoFrameYawPitchRoll("naturalPostureCenterOfMassAngleKdGains", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll comAngleDesired = new YoFrameYawPitchRoll("naturalPostureCenterOfMassAngleDesired",
                                                                               ReferenceFrame.getWorldFrame(),
                                                                               registry);
   private final YoFrameYawPitchRoll comAngularAcceleration = new YoFrameYawPitchRoll("naturalPostureCenterOfMassAngularAcceleration",
                                                                                      ReferenceFrame.getWorldFrame(),
                                                                                      registry);

   private final YoFrameQuaternion comAngleQuaternion = new YoFrameQuaternion("naturalPostureCenterOfMassAngleQuaternion",
                                                                              ReferenceFrame.getWorldFrame(),
                                                                              registry);
   private final YoFrameQuaternion comAngleDesiredQuaternion = new YoFrameQuaternion("naturalPostureCenterOfMassAngleDesiredQuaternion",
                                                                                     ReferenceFrame.getWorldFrame(),
                                                                                     registry);
   private final FrameQuaternion desiredNaturalPosture = new FrameQuaternion();

   private final YoFrameVector3D errorRotationVector = new YoFrameVector3D("naturalPostureErrorRotationVector", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector3D yoProportionalFeedback = new YoFrameVector3D("naturalPostureProportionalFeedback", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D yoDerivativeFeedback = new YoFrameVector3D("naturalPostureDerivativeFeedback", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector3D feedbackNPAcceleration = new YoFrameVector3D("naturalPostureFeedbackAcceleration", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector3D yoDirectProportionalFeedback = new YoFrameVector3D("naturalPostureDirectProportionalFeedback",
                                                                                    ReferenceFrame.getWorldFrame(),
                                                                                    registry);
   private final YoFrameVector3D yoDirectDerivativeFeedback = new YoFrameVector3D("naturalPostureDirectDerivativeFeedback",
                                                                                  ReferenceFrame.getWorldFrame(),
                                                                                  registry);

   private final FrameVector3D proportionalFeedback = new FrameVector3D();
   private final FrameVector3D derivativeFeedback = new FrameVector3D();

   private final OrientationFrame naturalPostureFrame = new OrientationFrame(comAngleQuaternion);
   private final YoBoolean doNullSpaceProjectionForNaturalPosture = new YoBoolean("doNullSpaceProjectionForNaturalPosture", registry);

   private final FullHumanoidRobotModel fullRobotModel;

   // This is used for generating data for the paper.
   private NaturalPosturePaperDataComputanator naturalPosturePaperDataComputanator;

   public NaturalPostureController(NaturalPostureParameters parameters,
                                   HighLevelHumanoidControllerToolbox controllerToolbox,
                                   YoRegistry parentRegistry)
   {
      controlDT = controllerToolbox.getControlDT();
      robotNaturalPosture = parameters.getNaturalPosture(controllerToolbox.getFullRobotModel());
      this.parameters = parameters;

      if (robotNaturalPosture.getRegistry() != null)
         registry.addChild(robotNaturalPosture.getRegistry());
      robotNaturalPosture.createVisuals(controllerToolbox.getYoGraphicsListRegistry());

      velocityBreakFrequency.addListener(v -> velocityAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(velocityBreakFrequency.getDoubleValue(),
                                                                                                                                controlDT), false));
      velocityAlpha.addListener(v -> velocityBreakFrequency.set(AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(velocityAlpha.getDoubleValue(),
                                                                                                                        controlDT), false));
      velocityBreakFrequency.set(parameters.getVelocityBreakFrequency());  //50

      //TODO This is likely not equivalent to filtering the 3D angular velocity and should probably be corrected.
      // We should filter the angular velocity and then compute yaw pitch roll rates from it.
      comAngularVelocityYaw = new FilteredVelocityYoVariable("npYawVelocity", "", velocityAlpha, comAngle.getYoYaw(), controlDT, registry);
      comAngularVelocityPitch = new FilteredVelocityYoVariable("npPitchVelocity", "", velocityAlpha, comAngle.getYoPitch(), controlDT, registry);
      comAngularVelocityRoll = new FilteredVelocityYoVariable("npRollVelocity", "", velocityAlpha, comAngle.getYoRoll(), controlDT, registry);

      fullRobotModel = controllerToolbox.getFullRobotModel();

      qpObjective.reshape(3, 1);
      weightMatrix.reshape(3, 3);
      selectionMatrix.reshape(3, 3);
      CommonOps_DDRM.setIdentity(selectionMatrix);

      //switches
      doNullSpaceProjectionForNaturalPosture.set(parameters.getDoNullSpaceProjectionForNaturalPosture());

      // Desired NP values (wrt world)
      comAngleDesired.set(parameters.getComAngleDesired());   // (0.0, -0.03, 0.0)

      kpComAngle.set(parameters.getAngularComKpGains());
      kdComAngle.set(parameters.getAngularComKdGains());

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
      weightMatrix.set(0, 0, parameters.getWeights().getX());
      weightMatrix.set(1, 1, parameters.getWeights().getY());
      weightMatrix.set(2, 2, parameters.getWeights().getZ());

      // Get current NP:   GMN - we're assuming NP compute() is getting called somewhere else?
      comAngleQuaternion.set(robotNaturalPosture.getCenterOfMassOrientation());
      comAngleDesiredQuaternion.setYawPitchRoll(comAngleDesired.getYaw(), comAngleDesired.getPitch(), comAngleDesired.getRoll());

      // Update the measured natural posture frame
      naturalPostureFrame.setOrientationAndUpdate(comAngleQuaternion);

      // The NP servo:
      comAngle.setYaw(comAngleQuaternion.getYaw());
      comAngle.setPitch(comAngleQuaternion.getPitch());
      comAngle.setRoll(comAngleQuaternion.getRoll());

      comAngularVelocityYaw.update();
      comAngularVelocityPitch.update();
      comAngularVelocityRoll.update();

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

      yoDirectProportionalFeedback.set(comAngleDesired.getRoll(), comAngleDesired.getPitch(), comAngleDesired.getYaw());
      yoDirectProportionalFeedback.sub(comAngle.getRoll(), comAngle.getPitch(), comAngle.getYaw());
      yoDirectProportionalFeedback.scale(kpComAngle.getRoll(), kpComAngle.getPitch(), kpComAngle.getYaw());

      yoDirectDerivativeFeedback.set(comAngularVelocityRoll.getDoubleValue(), comAngularVelocityPitch.getDoubleValue(), comAngularVelocityYaw.getDoubleValue());
      yoDirectDerivativeFeedback.scale(-kdComAngle.getRoll(), -kdComAngle.getPitch(), -kdComAngle.getYaw());

      comAngularAcceleration.setYaw(
            kpComAngle.getYaw() * (comAngleDesired.getYaw() - comAngle.getYaw()) - kdComAngle.getYaw() * comAngularVelocityYaw.getValue());
      comAngularAcceleration.setPitch(
            kpComAngle.getPitch() * (comAngleDesired.getPitch() - comAngle.getPitch()) - kdComAngle.getPitch() * comAngularVelocityPitch.getValue());
      comAngularAcceleration.setRoll(
            kpComAngle.getRoll() * (comAngleDesired.getRoll() - comAngle.getRoll()) - kdComAngle.getRoll() * comAngularVelocityRoll.getValue());

      computeProportionalNPFeedback(proportionalFeedback);
      computeDerivativeNPFeedback(derivativeFeedback);

      yoProportionalFeedback.setMatchingFrame(proportionalFeedback);
      yoDerivativeFeedback.setMatchingFrame(derivativeFeedback);

      feedbackNPAcceleration.add(yoProportionalFeedback, yoDerivativeFeedback);

      // TODO use these if you want to use the axis angle feedback controller
      //      yawPitchRollDoubleDot.set(0, 0, feedbackNPAcceleration.getZ());
      //      yawPitchRollDoubleDot.set(1, 0, feedbackNPAcceleration.getY());
      //      yawPitchRollDoubleDot.set(2, 0, feedbackNPAcceleration.getX());

      yawPitchRollDoubleDot.set(0, 0, comAngularAcceleration.getYaw());
      yawPitchRollDoubleDot.set(1, 0, comAngularAcceleration.getPitch());
      yawPitchRollDoubleDot.set(2, 0, comAngularAcceleration.getRoll());

      CommonOps_DDRM.mult(Dnp, yawPitchRollDoubleDot, qpObjective); // GMN: missing D-dot term (since InvDyn takes accels)

      // Populate the QPObjectiveCommand:
      naturalPostureControlCommand.setDoNullSpaceProjection(doNullSpaceProjectionForNaturalPosture.getBooleanValue());
      naturalPostureControlCommand.getObjective().set(qpObjective);
      naturalPostureControlCommand.getJacobian().set(robotNaturalPosture.getCenterOfMassOrientationJacobian());
      naturalPostureControlCommand.getSelectionMatrix().set(selectionMatrix);
      naturalPostureControlCommand.getWeightMatrix().set(weightMatrix);

      // For testing (data for paper)
      if (generateDataForPaper)
         naturalPosturePaperDataComputanator.computeDataForPaper();
   }

   private final Matrix3D tempGainMatrix = new Matrix3D();

   private void computeProportionalNPFeedback(FrameVector3D feedbackTermToPack)
   {
      desiredNaturalPosture.setIncludingFrame(comAngleDesiredQuaternion);
      desiredNaturalPosture.changeFrame(naturalPostureFrame);

      desiredNaturalPosture.normalizeAndLimitToPi();
      desiredNaturalPosture.getRotationVector(feedbackTermToPack);

      errorRotationVector.setMatchingFrame(feedbackTermToPack);

      tempGainMatrix.setM00(kpComAngle.getRoll());
      tempGainMatrix.setM11(kpComAngle.getPitch());
      tempGainMatrix.setM22(kpComAngle.getYaw());

      tempGainMatrix.transform(feedbackTermToPack);
      feedbackTermToPack.changeFrame(ReferenceFrame.getWorldFrame());
   }

   private void computeDerivativeNPFeedback(FrameVector3D feedbackTermToPack)
   {
      feedbackTermToPack.set(-comAngularVelocityRoll.getDoubleValue(), -comAngularVelocityPitch.getDoubleValue(), -comAngularVelocityYaw.getDoubleValue());

      // TODO update the gain matrix
      tempGainMatrix.setM00(kdComAngle.getRoll());
      tempGainMatrix.setM11(kdComAngle.getPitch());
      tempGainMatrix.setM22(kdComAngle.getYaw());

      tempGainMatrix.transform(feedbackTermToPack);
      feedbackTermToPack.changeFrame(ReferenceFrame.getWorldFrame());
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return naturalPostureControlCommand;
   }
}