package us.ihmc.commonWalkingControlModules.donkeyKick;

import us.ihmc.commonWalkingControlModules.donkeyKick.KickParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.ALIPTools;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.donkeyKick.KickInputParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CenterOfMassDynamicsTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class KickDynamicPlanner
{
   private static final double assumedFinalShiftDuration = 0.2;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   ///////// State variables
   private final ReferenceFrame centerOfMassControlFrame;
   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   ///////// Inputs
   private final KickParameters kickParameters;
   private final YoDouble estimatedTouchdownDuration = new YoDouble("estimatedTouchdownDuration", registry);
   private final double gravityZ;
   private final double totalMass;

   ///////// Resulting output variables that are computed
   private final YoDouble angularMomentumFromImpact = new YoDouble("angularMomentumFromImpact", registry);
   private final YoDouble equivalentVelocityChangeFromImpact = new YoDouble("equivalentVelocityChangeFromImpact", registry);
   private final YoDouble acpShiftFromImpact = new YoDouble("acpShiftFromImpact", registry);

   private final FramePoint2D desiredSwingFootStart = new FramePoint2D();
   private final FramePoint2D desiredShiftCoP = new FramePoint2D();
   private final FramePoint2D desiredStanceCoP = new FramePoint2D();
   private final FramePoint2D goalFootPositionAtTouchdown = new FramePoint2D();

   private final FramePoint2D goalACPPosition = new FramePoint2D();
   private final FramePoint2D acpInWorldAtTouchdown = new FramePoint2D();
   private final FramePoint2D acpInWorldAfterImpact = new FramePoint2D();
   private final FramePoint2D acpInWorldBeforeImpact = new FramePoint2D();
   private final FramePoint2D acpAfterShift = new FramePoint2D();

   private final FramePoint2D comPositionBeforeImpact = new FramePoint2D();
   private final FramePoint2D comPositionAfterShift = new FramePoint2D();
   private final FramePoint2D comPositionBeforeShift = new FramePoint2D();

   private final FrameVector2D angularMomentumBeforeImpact = new FrameVector2D();
   private final FrameVector2D angularMomentumAfterShift = new FrameVector2D();
   private final FrameVector2D angularMomentumBeforeShift = new FrameVector2D();

   // temp variables
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FramePoint3D acpInWorldAtTouchdown3D = new FramePoint3D();
   private final FramePoint3D desiredStanceCoP3D = new FramePoint3D();

   public KickDynamicPlanner(KickParameters kickParameters,
                             SideDependentList<? extends ReferenceFrame> soleFrames,
                             ReferenceFrame centerOfMassControlFrame,
                             double gravityZ,
                             double totalMass,
                             YoRegistry parentRegistry)
   {
      this.kickParameters = kickParameters;
      this.centerOfMassControlFrame = centerOfMassControlFrame;
      this.soleFrames = soleFrames;
      // figure out the touchdown duration after the fact.
      this.gravityZ = Math.abs(gravityZ);
      this.totalMass = totalMass;

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public FramePoint2DReadOnly getGoalFootPositionAtTouchdown()
   {
      return goalFootPositionAtTouchdown;
   }

   public FramePoint2DReadOnly getGoalACPPosition()
   {
      return goalACPPosition;
   }

   public FramePoint2DReadOnly getACPPositionAtTouchdown()
   {
      return acpInWorldAtTouchdown;
   }

   public FramePoint2DReadOnly getACPPositionAfterImpact()
   {
      return acpInWorldAfterImpact;
   }

   public FramePoint2DReadOnly getACPPositionBeforeImpact()
   {
      return acpInWorldBeforeImpact;
   }

   public FramePoint2DReadOnly getACPPositionAfterShift()
   {
      return acpAfterShift;
   }

   public FramePoint2DReadOnly getDesiredStanceFootCoP()
   {
      return desiredStanceCoP;
   }

   public FramePoint2DReadOnly getDesiredShiftCoP()
   {
      return desiredShiftCoP;
   }

   public FramePoint2DReadOnly getDesiredSwingFootStartNominal()
   {
      return desiredSwingFootStart;
   }

   public FramePoint2DReadOnly getCoMPositionBeforeImpact()
   {
      return comPositionBeforeImpact;
   }

   public FramePoint2DReadOnly getCoMPositionAfterShift()
   {
      return comPositionAfterShift;
   }

   public FramePoint2DReadOnly getCoMPositionBeforeShift()
   {
      return comPositionBeforeShift;
   }

   public FrameVector2DReadOnly getAngularMomentumBeforeImpact()
   {
      return angularMomentumBeforeImpact;
   }

   public FrameVector2DReadOnly getAngularMomentumAfterShift()
   {
      return angularMomentumAfterShift;
   }

   public void compute(KickInputParameters inputParameters)
   {
      double touchdownDuration = inputParameters.getKickHeight() / kickParameters.getTouchdownHeightSpeed();
      estimatedTouchdownDuration.set(touchdownDuration);
      kickParameters.setPreShiftWeightDistribution(inputParameters.getPrekickWeightDistribution());

      RobotSide supportSide = inputParameters.getKickFootSide().getOppositeSide();
      double omega = Math.sqrt(gravityZ / kickParameters.getDesiredCoMHeight());
      angularMomentumFromImpact.set(getChangeInAngularMomentumFromImpact(inputParameters));
      equivalentVelocityChangeFromImpact.set(angularMomentumFromImpact.getDoubleValue() / (kickParameters.getDesiredCoMHeight() * totalMass));
      acpShiftFromImpact.set(equivalentVelocityChangeFromImpact.getDoubleValue() / omega);

      // This is the desired CoP position of the stance foot throughout the kick
      desiredStanceCoP3D.setToZero(soleFrames.get(supportSide));
      desiredStanceCoP3D.setY(supportSide.negateIfLeftSide(kickParameters.getCopShiftInside()));
      desiredStanceCoP3D.changeFrame(ReferenceFrame.getWorldFrame());
      desiredStanceCoP.setIncludingFrame(desiredStanceCoP3D);

      computeGoalFootstepPosition(inputParameters);
      computeACPAtTouchdown(supportSide, omega);
      computePositionAfterImpactFromGoal(omega);
      computePositionBeforeImpact(omega, inputParameters);
      computePositionAfterShift(omega);
      computeInitialCoPAndCoM(inputParameters, omega);
   }

   private void computeGoalFootstepPosition(KickInputParameters inputParameters)
   {
      RobotSide kickSide = inputParameters.getKickFootSide();
      RobotSide stanceSide = kickSide.getOppositeSide();

      // Compute the foot position at touchdown, as given by the kick position. This is the foot position in the world once the kick is complete.
      double distanceOfTouchdown = inputParameters.getKickTargetDistance() + kickParameters.getDesiredTouchdownPositionRelativeToTarget();

      tempPoint.setToZero(soleFrames.get(kickSide));
      tempPoint.changeFrame(soleFrames.get(stanceSide));
      tempPoint.setX(-distanceOfTouchdown);

      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
      goalFootPositionAtTouchdown.set(tempPoint);
   }

   private void computeACPAtTouchdown(RobotSide supportSide, double omega)
   {
      ///// This method computes where the ACP should be after the final touchdown. When the foot hits the ground, the weight is shifted from one foot to the
      ///// goal position, which is where the robot should be stably resting.
      goalACPPosition.setIncludingFrame(goalFootPositionAtTouchdown);
      goalACPPosition.changeFrameAndProjectToXYPlane(soleFrames.get(supportSide));
      goalACPPosition.scale(0.5);
      goalACPPosition.addY(supportSide.negateIfLeftSide(kickParameters.getEndACPCheatInside()));
      goalACPPosition.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      tempPoint.setIncludingFrame(goalACPPosition, 0.0);
      acpInWorldAtTouchdown3D.setToZero(ReferenceFrame.getWorldFrame());

      CenterOfMassDynamicsTools.computeDesiredDCMPositionBackwardTime(omega,
                                                                      assumedFinalShiftDuration,
                                                                      assumedFinalShiftDuration,
                                                                      tempPoint,
                                                                      desiredStanceCoP3D,
                                                                      tempPoint,
                                                                      acpInWorldAtTouchdown3D);
      acpInWorldAtTouchdown.set(acpInWorldAtTouchdown3D);
   }

   // Back calculate the position of the ACP after the impact using the ACP at touchdown and the desired stance CoP
   private void computePositionAfterImpactFromGoal(double omega)
   {
      double touchdownDuration = estimatedTouchdownDuration.getValue();

      CapturePointTools.computeDesiredCapturePointPosition(omega, -touchdownDuration, acpInWorldAtTouchdown, desiredStanceCoP, acpInWorldAfterImpact);
   }

   // Back calculate the position of the ACP before the impact using the ACP after the impact and the shift from the impact
   private void computePositionBeforeImpact(double omega, KickInputParameters inputParameters)
   {
      tempPoint.setIncludingFrame(acpInWorldAfterImpact, 0.0);
      tempPoint.changeFrame(centerOfMassControlFrame);
      // this is the velocity change from the impact itself
      tempPoint.subX(acpShiftFromImpact.getValue());

      double angularMomentumFromImpact = getChangeInAngularMomentumFromImpact(inputParameters);
      double velocityEquivalent = angularMomentumFromImpact / (totalMass * kickParameters.getDesiredCoMHeight());

      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
      acpInWorldBeforeImpact.set(tempPoint);

      // get the com position right before impact
      tempPoint.changeFrame(centerOfMassControlFrame);
      tempPoint.addX(velocityEquivalent / omega);
      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
      comPositionBeforeImpact.set(tempPoint);

      tempPoint.setToZero(centerOfMassControlFrame);
      tempPoint.setY(-angularMomentumFromImpact);
      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
      angularMomentumBeforeImpact.set(tempPoint);
   }

   // Back calculate the position of the ACP after the shift using the ACP before the impact
   private void computePositionAfterShift(double omega)
   {
      // Compute the ACP translation during the kick itself
      double kickMotionDuration = kickParameters.getPushDuration() + kickParameters.getChamberDuration();

      CapturePointTools.computeDesiredCapturePointPosition(omega, -kickMotionDuration, acpInWorldBeforeImpact, desiredStanceCoP, acpAfterShift);

      ALIPTools.computeCenterOfMassPosition(-kickMotionDuration,
                                            omega,
                                            totalMass,
                                            gravityZ,
                                            comPositionBeforeImpact,
                                            angularMomentumBeforeImpact,
                                            desiredStanceCoP,
                                            comPositionAfterShift);
      ALIPTools.computeAngularMomentum(-kickMotionDuration,
                                       omega,
                                       totalMass,
                                       gravityZ,
                                       comPositionBeforeImpact,
                                       angularMomentumBeforeImpact,
                                       desiredStanceCoP,
                                       angularMomentumAfterShift);
   }

   private void computeInitialCoPAndCoM(KickInputParameters inputParameters, double omega)
   {
      RobotSide kickSide = inputParameters.getKickFootSide();
      RobotSide supportSide = kickSide.getOppositeSide();
      double shiftDuration = kickParameters.getShiftDuration();
      double exponential = Math.exp(omega * shiftDuration);

      ReferenceFrame supportSoleFrame = soleFrames.get(supportSide);
      // from the y icp dynamics, we can figure out what the necessary distribution between the stance and swing foot are for weight for the constant cop
      acpAfterShift.changeFrameAndProjectToXYPlane(supportSoleFrame);
      comPositionAfterShift.changeFrameAndProjectToXYPlane(supportSoleFrame);
      tempPoint.setToZero(soleFrames.get(kickSide));
      tempPoint.changeFrame(supportSoleFrame);

      double stanceWidth = tempPoint.getY();
      double initialWeightDistribution = kickParameters.getPreShiftWeightDistribution();
      double initialICPY = initialWeightDistribution * stanceWidth;
      double desiredShiftCoPY = (-exponential * initialICPY + acpAfterShift.getY()) / (1.0 - exponential);

      // this is the fraction to go from the
      double shiftWeightDistribution = desiredShiftCoPY / stanceWidth;

      // compute the necessary swing foot initial position
      double desiredSwingFootInitialPositionX = comPositionAfterShift.getX()  / ((initialWeightDistribution - shiftWeightDistribution) * Math.cosh(omega * kickParameters.getShiftDuration()) + shiftWeightDistribution);
      double desiredShiftCoPX = shiftWeightDistribution * desiredSwingFootInitialPositionX;

      desiredShiftCoP.setIncludingFrame(supportSoleFrame, desiredShiftCoPX, desiredShiftCoPY);
      desiredSwingFootStart.setIncludingFrame(supportSoleFrame, desiredSwingFootInitialPositionX, tempPoint.getY());

      acpAfterShift.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      comPositionAfterShift.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      desiredShiftCoP.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      desiredSwingFootStart.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      ALIPTools.computeCenterOfMassPosition(-shiftDuration,
                                            omega,
                                            totalMass,
                                            gravityZ,
                                            comPositionAfterShift,
                                            angularMomentumAfterShift,
                                            desiredStanceCoP,
                                            comPositionBeforeShift);
      ALIPTools.computeAngularMomentum(-shiftDuration,
                                       omega,
                                       totalMass,
                                       gravityZ,
                                       comPositionAfterShift,
                                       angularMomentumAfterShift,
                                       desiredStanceCoP,
                                       angularMomentumBeforeShift);
   }

   private static double getChangeInAngularMomentumFromImpact(KickInputParameters inputParameters)
   {
      return inputParameters.getKickImpulse() * inputParameters.getKickHeight();
   }
}
