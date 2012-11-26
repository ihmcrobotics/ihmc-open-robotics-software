package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FootPolygonEnum;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodyOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.LimbControlMode;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.trajectories.CenterOfMassHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.FlatThenPolynomialCoMHeightTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.EndEffectorPoseTwistAndSpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PDController;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionAction;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;
import com.yobotics.simulationconstructionset.util.trajectory.CartesianTrajectoryGenerator;

public class WalkingHighLevelHumanoidController extends AbstractHighLevelHumanoidController
{
   private static enum WalkingState {LEFT_SUPPORT, RIGHT_SUPPORT, TRANSFER_TO_LEFT_SUPPORT, TRANSFER_TO_RIGHT_SUPPORT, DOUBLE_SUPPORT}

   private final StateMachine stateMachine;
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final CenterOfMassHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator;
   private final PDController centerOfMassHeightController;
   private final SideDependentList<WalkingState> singleSupportStateEnums =
      new SideDependentList<WalkingHighLevelHumanoidController.WalkingState>(WalkingState.LEFT_SUPPORT, WalkingState.RIGHT_SUPPORT);

   private final SideDependentList<WalkingState> transferStateEnums =
      new SideDependentList<WalkingHighLevelHumanoidController.WalkingState>(WalkingState.TRANSFER_TO_LEFT_SUPPORT, WalkingState.TRANSFER_TO_RIGHT_SUPPORT);

   private final double doubleSupportTime = 0.2;    // 0.6;    // 0.3

   private final DoubleYoVariable singleSupportICPGlideScaleFactor = new DoubleYoVariable("singleSupportICPGlideScaleFactor", registry);
   private final BooleanYoVariable walk = new BooleanYoVariable("walk", registry);
   private final BooleanYoVariable liftUpHeels = new BooleanYoVariable("liftUpHeels", registry);
   private final DoubleYoVariable trailingFootPitch = new DoubleYoVariable("trailingFootPitch", registry);
   private final DoubleYoVariable leadingFootPitch = new DoubleYoVariable("leadingFootPitch", registry);

   private final double swingNullspaceMultiplier = 500.0;    // needs to be pretty high to fight the limit stops...
   private final SideDependentList<BooleanYoVariable> trajectoryInitialized = new SideDependentList<BooleanYoVariable>();

   private final BagOfBalls footTrajectoryBagOfBalls;
   private final BagOfBalls comTrajectoryBagOfBalls;
   private int comTrajectoryCounter = 0;

   // private final BooleanYoVariable transferICPTrajectoryDone = new BooleanYoVariable("transferICPTrajectoryDone", registry);
   private final DoubleYoVariable minOrbitalEnergyForSingleSupport = new DoubleYoVariable("minOrbitalEnergyForSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideSingleSupport = new DoubleYoVariable("amountToBeInsideSingleSupport", registry);
   private final DoubleYoVariable amountToBeInsideDoubleSupport = new DoubleYoVariable("amountToBeInsideDoubleSupport", registry);

   private final YoFrameOrientation desiredSwingFootOrientation = new YoFrameOrientation("desiredSwingFootOrientation", worldFrame, registry);

   private final SideDependentList<CartesianTrajectoryGenerator> footCartesianTrajectoryGenerators;
   private final SideDependentList<EndEffectorPoseTwistAndSpatialAccelerationCalculator> footPoseTwistAndSpatialAccelerationCalculators =
      new SideDependentList<EndEffectorPoseTwistAndSpatialAccelerationCalculator>();
   private final SideDependentList<RigidBodySpatialAccelerationControlModule> footSpatialAccelerationControlModules =
      new SideDependentList<RigidBodySpatialAccelerationControlModule>();

   private final RigidBodyOrientationControlModule chestOrientationControlModule;


   public WalkingHighLevelHumanoidController(FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames, TwistCalculator twistCalculator,
           CenterOfMassJacobian centerOfMassJacobian, SideDependentList<BipedFootInterface> bipedFeet, BipedSupportPolygons bipedSupportPolygons,
           SideDependentList<FootSwitchInterface> footSwitches, double gravityZ, DoubleYoVariable yoTime, double controlDT, YoVariableRegistry registry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, DesiredFootstepCalculator desiredFootstepCalculator,
           CenterOfMassHeightTrajectoryGenerator centerOfMassHeightTrajectoryGenerator, SideDependentList<CartesianTrajectoryGenerator> footCartesianTrajectoryGenerators, boolean liftUpHeels, FrameOrientation desiredPelvisOrientation, ArrayList<Updatable> updatables)
   {
      super(fullRobotModel, referenceFrames, yoTime, gravityZ, twistCalculator, bipedFeet, bipedSupportPolygons, controlDT, footSwitches, updatables, registry);

      this.desiredFootstepCalculator = desiredFootstepCalculator;

      // this.centerOfMassHeightTrajectoryGenerator = new LinearFootstepCalculatorBasedCoMHeightTrajectoryGenerator(processedSensors, desiredFootstepCalculator,
      // referenceFrames, desiredHeadingControlModule.getDesiredHeadingFrame(), registry);

      // this.centerOfMassHeightTrajectoryGenerator = new ConstantCenterOfMassHeightTrajectoryGenerator(registry);
      this.centerOfMassJacobian = centerOfMassJacobian;    // new CenterOfMassJacobian(fullRobotModel.getElevator());
      this.centerOfMassHeightTrajectoryGenerator = centerOfMassHeightTrajectoryGenerator;

      this.centerOfMassHeightController = new PDController("comHeight", registry);
      centerOfMassHeightController.setProportionalGain(40.0);    // about 4000 / totalMass
      double zeta = 1.0;
      centerOfMassHeightController.setDerivativeGain(2.0 * zeta * Math.sqrt(centerOfMassHeightController.getProportionalGain()));

      String namePrefix = "walking";
      this.stateMachine = new StateMachine(namePrefix + "State", namePrefix + "SwitchTime", WalkingState.class, yoTime, registry);
      upcomingSupportLeg.set(RobotSide.RIGHT);

      singleSupportICPGlideScaleFactor.set(0.9);
      FramePoint2d finalDesiredICPForDoubleSupportStance = getDoubleSupportFinalDesiredICPForDoubleSupportStance();
      finalDesiredICPForDoubleSupportStance.changeFrame(desiredICP.getReferenceFrame());

      this.chestOrientationControlModule = new RigidBodyOrientationControlModule("chest", fullRobotModel.getPelvis(), fullRobotModel.getChest(),
              twistCalculator, registry);
      chestOrientationControlModule.setProportionalGains(100.0, 100.0, 100.0);
      chestOrientationControlModule.setDerivativeGains(20.0, 20.0, 20.0);

      this.footCartesianTrajectoryGenerators = footCartesianTrajectoryGenerators;

      for (RobotSide robotSide : RobotSide.values())
      {
         trajectoryInitialized.put(robotSide, new BooleanYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "TrajectoryInitialized", registry));
         EndEffectorPoseTwistAndSpatialAccelerationCalculator feetPoseTwistAndSpatialAccelerationCalculator =
            new EndEffectorPoseTwistAndSpatialAccelerationCalculator(fullRobotModel.getEndEffector(robotSide, LimbName.LEG),
               fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG), twistCalculator);
         footPoseTwistAndSpatialAccelerationCalculators.put(robotSide, feetPoseTwistAndSpatialAccelerationCalculator);

         String controlModuleNamePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "Leg";
         RigidBodySpatialAccelerationControlModule rigidBodySpatialAccelerationControlModule =
            new RigidBodySpatialAccelerationControlModule(controlModuleNamePrefix, twistCalculator, fullRobotModel.getEndEffector(robotSide, LimbName.LEG),
               fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG), registry);

         footSpatialAccelerationControlModules.put(robotSide, rigidBodySpatialAccelerationControlModule);
      }

      footTrajectoryBagOfBalls = new BagOfBalls(100, 0.01, "footBagOfBalls", YoAppearance.Black(), registry, dynamicGraphicObjectsListRegistry);
      comTrajectoryBagOfBalls = new BagOfBalls(500, 0.01, "comBagOfBalls", YoAppearance.Red(), registry, dynamicGraphicObjectsListRegistry);

      setUpStateMachine();

      walk.set(false);
      this.liftUpHeels.set(liftUpHeels);
      trailingFootPitch.set(0.05);
      leadingFootPitch.set(0.8);
      minOrbitalEnergyForSingleSupport.set(0.007);    // 0.008
      amountToBeInsideSingleSupport.set(0.0);
      amountToBeInsideDoubleSupport.set(0.05);
      this.desiredPelvisOrientation.set(desiredPelvisOrientation);

      for (RobotSide robotSide : RobotSide.values())
      {
         controlModes.get(robotSide).get(LimbName.LEG).set(LimbControlMode.CARTESIAN_SPACE);
         controlModes.get(robotSide).get(LimbName.ARM).set(LimbControlMode.JOINT_SPACE);
      }
   }

   private void setUpStateMachine()
   {
      DoubleSupportState doubleSupportState = new DoubleSupportState(null);

      stateMachine.addState(doubleSupportState);

      for (RobotSide robotSide : RobotSide.values())
      {
         State transferState = new DoubleSupportState(robotSide);
         StateTransition toDoubleSupport = new StateTransition(doubleSupportState.getStateEnum(), new StopWalkingCondition(), new ResetICPTrajectoryAction());
         transferState.addStateTransition(toDoubleSupport);
         StateTransition toSingleSupport = new StateTransition(singleSupportStateEnums.get(robotSide), new DoneWithTransferCondition());
         transferState.addStateTransition(toSingleSupport);
         stateMachine.addState(transferState);

         State singleSupportState = new SingleSupportState(robotSide);
         StateTransition toDoubleSupport2 = new StateTransition(doubleSupportState.getStateEnum(), new StopWalkingCondition(), new ResetICPTrajectoryAction());
         singleSupportState.addStateTransition(toDoubleSupport2);
         StateTransition toTransfer = new StateTransition(transferStateEnums.get(robotSide.getOppositeSide()), new DoneWithSingleSupportCondition());
         singleSupportState.addStateTransition(toTransfer);
         stateMachine.addState(singleSupportState);
      }

      for (RobotSide robotSide : RobotSide.values())
      {
         StateTransition toTransfer = new StateTransition(transferStateEnums.get(robotSide), new DoneWithDoubleSupportCondition(robotSide));
         doubleSupportState.addStateTransition(toTransfer);
      }
   }

   public void initialize()
   {
      stateMachine.setCurrentState(WalkingState.DOUBLE_SUPPORT);
   }

   private class DoubleSupportState extends State
   {
      private final RobotSide transferToSide;

      public DoubleSupportState(RobotSide transferToSide)
      {
         super((transferToSide == null) ? WalkingState.DOUBLE_SUPPORT : transferStateEnums.get(transferToSide));
         this.transferToSide = transferToSide;
      }

      @Override
      public void doAction()
      {
         evaluateCoMTrajectory();

         for (RobotSide robotSide : RobotSide.values())
         {
            BipedFootInterface bipedFoot = bipedFeet.get(robotSide);

            if (liftUpHeels.getBooleanValue())
            {
               bipedFoot.setFootPolygonInUse(FootPolygonEnum.ONTOES);
               bipedFoot.setShift(1.0);

               double footPitch = (robotSide == getUpcomingSupportLeg().getOppositeSide())
                                  ? leadingFootPitch.getDoubleValue() : trailingFootPitch.getDoubleValue();
               controlDesiredFootPosVelAccForSupportSide(robotSide, footPitch);
            }
            else
            {
               bipedFoot.setFootPolygonInUse(FootPolygonEnum.FLAT);
               controlDesiredFootPosVelAccForSupportSide(robotSide, 0.0);
            }
         }

         // if ((transferToSide != null) && icpTrajectoryGenerator.isDone() && icpTrajectoryHasBeenInitialized.getBooleanValue())
         // {
         // transferICPTrajectoryDone.set(true);
         // }

         if (icpTrajectoryGenerator.isDone() && (transferToSide == null))
         {
            // keep desiredICP the same
            desiredICPVelocity.set(0.0, 0.0);
         }
         else
         {
            FramePoint2d desiredICPLocal = new FramePoint2d(desiredICP.getReferenceFrame());
            FrameVector2d desiredICPVelocityLocal = new FrameVector2d(desiredICPVelocity.getReferenceFrame());
            icpTrajectoryGenerator.pack(desiredICPLocal, desiredICPVelocityLocal, omega0);
            desiredICP.set(desiredICPLocal);
            desiredICPVelocity.set(desiredICPVelocityLocal);
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         if (liftUpHeels.getBooleanValue())
         {
            for (RobotSide robotSide : RobotSide.values())
            {
               BipedFootInterface bipedFoot = bipedFeet.get(robotSide);
               bipedFoot.setFootPolygonInUse(FootPolygonEnum.ONTOES);
               bipedFoot.setShift(1.0);
            }
         }

         setSupportLeg(null);

         FramePoint2d finalDesiredICP;

         desiredICP.set(capturePoint);

         if (transferToSide == null)
         {
            finalDesiredICP = getDoubleSupportFinalDesiredICPForDoubleSupportStance();
         }
         else
         {
            Footstep currentFootstep = new Footstep(transferToSide, new FramePose(referenceFrames.getFootFrame(transferToSide)));
            finalDesiredICP = getDoubleSupportFinalDesiredICPForWalking(currentFootstep, bipedFeet.get(transferToSide).getFootPolygonInSoleFrame());
         }

         finalDesiredICP.changeFrame(desiredICP.getReferenceFrame());
         icpTrajectoryGenerator.initialize(desiredICP.getFramePoint2dCopy(), finalDesiredICP, doubleSupportTime, omega0,
                                           amountToBeInsideDoubleSupport.getDoubleValue());

         if (transferToSide == null)
            centerOfMassHeightTrajectoryGenerator.initialize(getSupportLeg(), getUpcomingSupportLeg());
         else
         {
            desiredFootstepCalculator.initializeDesiredFootstep(transferToSide);
            centerOfMassHeightTrajectoryGenerator.initialize(getSupportLeg(), upcomingSupportLeg.getEnumValue());
         }

         for (RobotSide robotSide : RobotSide.values())
         {
            limbNullspaceMultipliers.get(robotSide).get(LimbName.LEG).set(0.0);
            setStanceControlGains(robotSide);
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {
         desiredICPVelocity.set(0.0, 0.0);
      }
   }


   private class SingleSupportState extends State
   {
      private final RobotSide swingSide;
      private final ReferenceFrame supportAnkleZUpFrame;

      private final FramePoint positionToPack;
      private final FrameVector velocityToPack;
      private final FrameVector accelerationToPack;

      private final FrameVector angularVelocityToPack;
      private final FrameVector angularAccelerationToPack;

      private int counter = 0;

      public SingleSupportState(RobotSide robotSide)
      {
         super(singleSupportStateEnums.get(robotSide));
         this.swingSide = robotSide.getOppositeSide();
         supportAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide());
         this.positionToPack = new FramePoint(supportAnkleZUpFrame);
         this.velocityToPack = new FrameVector(supportAnkleZUpFrame);
         this.accelerationToPack = new FrameVector(supportAnkleZUpFrame);

         this.angularVelocityToPack = new FrameVector(worldFrame);
         this.angularAccelerationToPack = new FrameVector(worldFrame);

      }

      @Override
      public void doAction()
      {
         evaluateCoMTrajectory();

         CartesianTrajectoryGenerator cartesianTrajectoryGenerator = footCartesianTrajectoryGenerators.get(swingSide);
         if (!cartesianTrajectoryGenerator.isDone() && trajectoryInitialized.get(swingSide).getBooleanValue())
         {
            cartesianTrajectoryGenerator.computeNextTick(positionToPack, velocityToPack, accelerationToPack, controlDT);
            FramePoint2d desiredICPLocal = new FramePoint2d(desiredICP.getReferenceFrame());
            FrameVector2d desiredICPVelocityLocal = new FrameVector2d(desiredICPVelocity.getReferenceFrame());
            icpTrajectoryGenerator.pack(desiredICPLocal, desiredICPVelocityLocal, omega0);
            desiredICP.set(desiredICPLocal);
            desiredICPVelocity.set(desiredICPVelocityLocal);
         }
         else
         {
            positionToPack.setToZero(referenceFrames.getFootFrame(swingSide));
            velocityToPack.setToZero(referenceFrames.getFootFrame(swingSide));
            accelerationToPack.setToZero(referenceFrames.getFootFrame(swingSide));

            // don't change desiredICP
            desiredICPVelocity.set(0.0, 0.0);
         }

         positionToPack.changeFrame(worldFrame);
         velocityToPack.changeFrame(worldFrame);
         accelerationToPack.changeFrame(worldFrame);

         EndEffectorPoseTwistAndSpatialAccelerationCalculator footPoseTwistAndSpatialAccelerationCalculator =
            footPoseTwistAndSpatialAccelerationCalculators.get(swingSide);
         RigidBody elevator = fullRobotModel.getElevator();

         FramePose desiredPose = footPoseTwistAndSpatialAccelerationCalculator.calculateDesiredEndEffectorPoseFromDesiredPositions(positionToPack,
                                    desiredSwingFootOrientation.getFrameOrientationCopy());
         Twist desiredTwist = footPoseTwistAndSpatialAccelerationCalculator.calculateDesiredEndEffectorTwistFromDesiredVelocities(velocityToPack,
                                 angularVelocityToPack, elevator);
         SpatialAccelerationVector feedForwardSpatialAcceleration =
            footPoseTwistAndSpatialAccelerationCalculator.calculateDesiredEndEffectorSpatialAccelerationFromDesiredAccelerations(accelerationToPack,
               angularAccelerationToPack, elevator);

         footSpatialAccelerationControlModules.get(swingSide).doPositionControl(desiredPose, desiredTwist, feedForwardSpatialAcceleration, elevator);
         SpatialAccelerationVector footAcceleration = new SpatialAccelerationVector();
         footSpatialAccelerationControlModules.get(swingSide).packAcceleration(footAcceleration);
         setEndEffectorSpatialAcceleration(swingSide, LimbName.LEG, footAcceleration);

         controlDesiredFootPosVelAccForSupportSide(swingSide.getOppositeSide(), trailingFootPitch.getDoubleValue());    // FIXME: make it a smooth trajectory

         counter++;

         if (counter >= 100)
         {
            positionToPack.changeFrame(worldFrame);
            footTrajectoryBagOfBalls.setBallLoop(positionToPack);
            counter = 0;
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         RobotSide supportSide = swingSide.getOppositeSide();
         setSupportLeg(supportSide);

         centerOfMassHeightTrajectoryGenerator.initialize(getSupportLeg(), upcomingSupportLeg.getEnumValue());
         limbNullspaceMultipliers.get(swingSide).get(LimbName.LEG).set(swingNullspaceMultiplier);
         limbNullspaceMultipliers.get(supportSide).get(LimbName.LEG).set(0.0);
         trajectoryInitialized.get(swingSide).set(false);

         desiredSwingFootOrientation.setYawPitchRoll(0.0, 0.0, 0.0);

         setStanceControlGains(supportSide);
         setSwingControlGains(swingSide);

      }

      @Override
      public void doTransitionOutOfAction()
      {
         upcomingSupportLeg.set(upcomingSupportLeg.getEnumValue().getOppositeSide());
      }
   }


   public class DoneWithDoubleSupportCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public DoneWithDoubleSupportCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         boolean doubleSupportTimeHasPassed = stateMachine.timeInCurrentState() > doubleSupportTime;
         boolean transferringToThisRobotSide = robotSide == upcomingSupportLeg.getEnumValue();

         return walk.getBooleanValue() && transferringToThisRobotSide && doubleSupportTimeHasPassed;
      }
   }


   public class DoneWithTransferCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         // TODO: not really nice, but it'll do:
         if (centerOfMassHeightTrajectoryGenerator instanceof FlatThenPolynomialCoMHeightTrajectoryGenerator)
         {
            FlatThenPolynomialCoMHeightTrajectoryGenerator flatThenPolynomialCoMHeightTrajectoryGenerator =
               (FlatThenPolynomialCoMHeightTrajectoryGenerator) centerOfMassHeightTrajectoryGenerator;
            double orbitalEnergy = flatThenPolynomialCoMHeightTrajectoryGenerator.computeOrbitalEnergyIfInitializedNow(getUpcomingSupportLeg());

            // return transferICPTrajectoryDone.getBooleanValue() && orbitalEnergy > minOrbitalEnergy;
            return orbitalEnergy > minOrbitalEnergyForSingleSupport.getDoubleValue();
         }
         else
         {
            return icpTrajectoryGenerator.isDone();
         }
      }
   }


   public class DoneWithSingleSupportCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         RobotSide swingSide = supportLeg.getEnumValue().getOppositeSide();
         double minimumSwingFraction = 0.5;
         double minimumSwingTime = footCartesianTrajectoryGenerators.get(swingSide).getFinalTime() * minimumSwingFraction;
         boolean footHitGround = (stateMachine.timeInCurrentState() > minimumSwingTime) && footSwitches.get(swingSide).hasFootHitGround();

         return trajectoryInitialized.get(swingSide).getBooleanValue() && (footCartesianTrajectoryGenerators.get(swingSide).isDone() || footHitGround);
      }
   }


   public class StopWalkingCondition extends DoneWithSingleSupportCondition
   {
      public boolean checkCondition()
      {
         return !walk.getBooleanValue() && ((supportLeg.getEnumValue() == null) || super.checkCondition());
      }
   }


   public class ResetICPTrajectoryAction implements StateTransitionAction
   {
      public void doTransitionAction()
      {
         icpTrajectoryGenerator.reset();
      }
   }


   public void setPreviousCoP(FramePoint2d previousCoP)
   {
      previousCoP.changeFrame(this.previousCoP.getReferenceFrame());
      this.previousCoP.set(previousCoP);
   }

   public void setCapturePoint(FramePoint2d capturePoint)
   {
      capturePoint.changeFrame(this.capturePoint.getReferenceFrame());
      this.capturePoint.set(capturePoint);
   }

   public void setOmega0(double omega0)
   {
      this.omega0 = omega0;
   }

   private FramePoint2d getDoubleSupportFinalDesiredICPForDoubleSupportStance()
   {
      FramePoint2d ret = new FramePoint2d(worldFrame);
      double trailingFootToLeadingFootFactor = 0.5;    // 0.25;
      for (RobotSide robotSide : RobotSide.values())
      {
         FramePoint2d centroid = new FramePoint2d(ret.getReferenceFrame());
         bipedFeet.get(robotSide).getFootPolygonInUseInAnkleZUp().getCentroid(centroid);
         centroid.changeFrame(ret.getReferenceFrame());
         if (robotSide == getUpcomingSupportLeg())
            centroid.scale(trailingFootToLeadingFootFactor);
         else
            centroid.scale(1.0 - trailingFootToLeadingFootFactor);
         ret.add(centroid);
      }

      return ret;
   }

   private FramePoint2d getDoubleSupportFinalDesiredICPForWalking(Footstep footstep, FrameConvexPolygon2d footPolygonInSoleFrame)
   {
      RobotSide robotSide = footstep.getFootstepSide();
      if (footPolygonInSoleFrame.getReferenceFrame() != referenceFrames.getSoleFrame(robotSide))
      {
         throw new RuntimeException("not in sole frame");
      }

      ReferenceFrame stairDirectionFrame = worldFrame;
      footstep.changeFrame(stairDirectionFrame);

      Transform3D footstepTransform = new Transform3D();
      footstep.getFootstepPose().getTransform3D(footstepTransform);

      FramePoint2d centroid2d = footPolygonInSoleFrame.getCentroidCopy();
      FramePoint centroid = centroid2d.toFramePoint();
      centroid.changeFrame(referenceFrames.getFootFrame(robotSide));
      centroid.changeFrameUsingTransform(stairDirectionFrame, footstepTransform);
      FramePoint2d ret = centroid.toFramePoint2d();

      double extraX = 0.0;    // 0.02
      double extraY = robotSide.negateIfLeftSide(0.04);
      FrameVector2d offset = new FrameVector2d(stairDirectionFrame, extraX, extraY);
      offset.changeFrame(ret.getReferenceFrame());
      ret.add(offset);

      return ret;
   }

   private FramePoint2d getSingleSupportFinalDesiredICPForWalking(Footstep desiredFootstep, RobotSide swingSide)
   {
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint2d initialDesiredICP = desiredICP.getFramePoint2dCopy();
      initialDesiredICP.changeFrame(referenceFrame);

      // TODO: think about setting the shift factor here
      FramePoint2d finalDesiredICP = getDoubleSupportFinalDesiredICPForWalking(desiredFootstep, bipedFeet.get(swingSide).getFootPolygonInSoleFrame());    // yes, swing side
      finalDesiredICP.changeFrame(referenceFrame);

      FrameLineSegment2d initialToFinal = new FrameLineSegment2d(initialDesiredICP, finalDesiredICP);

      return initialToFinal.pointBetweenEndPointsGivenParameter(singleSupportICPGlideScaleFactor.getDoubleValue());
   }

   private void controlDesiredFootPosVelAccForSupportSide(RobotSide supportSide, double footPitch)
   {
      FramePoint desiredEndEffectorPosition = new FramePoint(referenceFrames.getFootFrame(supportSide));
      desiredEndEffectorPosition.changeFrame(worldFrame);

      FrameOrientation desiredEndEffectorOrientation = new FrameOrientation(worldFrame, 0.0, footPitch, 0.0);

      FrameVector desiredLinearVelocity = new FrameVector(worldFrame);
      FrameVector desiredAngularVelocity = new FrameVector(worldFrame);

      FrameVector desiredLinearAcceleration = new FrameVector(worldFrame);
      FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);

      EndEffectorPoseTwistAndSpatialAccelerationCalculator footPoseTwistAndSpatialAccelerationCalculator =
         footPoseTwistAndSpatialAccelerationCalculators.get(supportSide);
      RigidBody elevator = fullRobotModel.getElevator();
      FramePose desiredPose = footPoseTwistAndSpatialAccelerationCalculator.calculateDesiredEndEffectorPoseFromDesiredPositions(desiredEndEffectorPosition,
                                 desiredEndEffectorOrientation);
      Twist desiredTwist = footPoseTwistAndSpatialAccelerationCalculator.calculateDesiredEndEffectorTwistFromDesiredVelocities(desiredLinearVelocity,
                              desiredAngularVelocity, elevator);
      SpatialAccelerationVector feedForwardSpatialAcceleration =
         footPoseTwistAndSpatialAccelerationCalculator.calculateDesiredEndEffectorSpatialAccelerationFromDesiredAccelerations(desiredLinearAcceleration,
            desiredAngularAcceleration, elevator);

      footSpatialAccelerationControlModules.get(supportSide).doPositionControl(desiredPose, desiredTwist, feedForwardSpatialAcceleration,
              fullRobotModel.getElevator());
      SpatialAccelerationVector footAcceleration = new SpatialAccelerationVector();
      footSpatialAccelerationControlModules.get(supportSide).packAcceleration(footAcceleration);
      setEndEffectorSpatialAcceleration(supportSide, LimbName.LEG, footAcceleration);
   }

   private void setSupportLeg(RobotSide supportLeg)
   {
      this.supportLeg.set(supportLeg);

      for (RobotSide robotSide : RobotSide.values())
      {
         boolean isSupportingFoot = (supportLeg == robotSide) || (supportLeg == null);
         bipedFeet.get(robotSide).setIsSupportingFoot(isSupportingFoot);
      }

      bipedSupportPolygons.update(bipedFeet.get(RobotSide.LEFT), bipedFeet.get(RobotSide.RIGHT));
   }

   public void initializeTrajectory(RobotSide swingSide, SpatialAccelerationVector taskSpaceAcceleration)
   {
      RobotSide supportSide = swingSide.getOppositeSide();
      ReferenceFrame swingAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(swingSide);
      ReferenceFrame trajectoryGeneratorFrame = footCartesianTrajectoryGenerators.get(swingSide).getReferenceFrame();
      ReferenceFrame swingFootFrame = referenceFrames.getFootFrame(swingSide);
      FramePoint initialPosition = new FramePoint(swingAnkleZUpFrame);
      initialPosition.changeFrame(trajectoryGeneratorFrame);

      Twist footTwist = new Twist();
      twistCalculator.packTwistOfBody(footTwist, fullRobotModel.getFoot(swingSide));

      SpatialAccelerationVector taskSpaceAccelerationWithRespectToWorld = new SpatialAccelerationVector();
      spatialAccelerationCalculator.compute();
      spatialAccelerationCalculator.packAccelerationOfBody(taskSpaceAccelerationWithRespectToWorld, fullRobotModel.getPelvis());

      Twist pelvisTwist = new Twist();
      twistCalculator.packTwistOfBody(pelvisTwist, fullRobotModel.getPelvis());
      taskSpaceAccelerationWithRespectToWorld.changeFrame(taskSpaceAccelerationWithRespectToWorld.getBaseFrame(), pelvisTwist, pelvisTwist);

      Twist footPelvisTwist = new Twist(pelvisTwist);
      footPelvisTwist.invert();
      footPelvisTwist.changeFrame(footTwist.getExpressedInFrame());
      footPelvisTwist.add(footTwist);

      taskSpaceAcceleration.changeFrame(taskSpaceAccelerationWithRespectToWorld.getExpressedInFrame(), footTwist, footPelvisTwist);
      taskSpaceAccelerationWithRespectToWorld.add(taskSpaceAcceleration);
      FramePoint swingAnkle = new FramePoint(swingFootFrame);
      swingAnkle.changeFrame(taskSpaceAccelerationWithRespectToWorld.getBaseFrame());
      footTwist.changeFrame(taskSpaceAccelerationWithRespectToWorld.getExpressedInFrame());

      FrameVector initialAcceleration = new FrameVector(worldFrame);
      taskSpaceAccelerationWithRespectToWorld.packAccelerationOfPointFixedInBodyFrame(footTwist, swingAnkle, initialAcceleration);
      initialAcceleration.changeFrame(trajectoryGeneratorFrame);

      initialAcceleration.setToZero(trajectoryGeneratorFrame);    // TODO

      footTwist.changeFrame(swingFootFrame);
      FrameVector initialVelocity = new FrameVector(trajectoryGeneratorFrame);
      footTwist.packLinearPart(initialVelocity);
      initialVelocity.changeFrame(trajectoryGeneratorFrame);

      Footstep desiredFootstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(supportSide);
      FramePoint finalDesiredStepLocation = desiredFootstep.getFootstepPositionInFrame(trajectoryGeneratorFrame);
      finalDesiredStepLocation.setZ(finalDesiredStepLocation.getZ());

      FrameVector finalDesiredVelocity = new FrameVector(trajectoryGeneratorFrame);

      CartesianTrajectoryGenerator cartesianTrajectoryGenerator = footCartesianTrajectoryGenerators.get(swingSide);
      cartesianTrajectoryGenerator.initialize(initialPosition, initialVelocity, initialAcceleration, finalDesiredStepLocation, finalDesiredVelocity);

      desiredSwingFootOrientation.set(desiredFootstep.getFootstepOrientationInFrame(worldFrame));

      desiredICP.set(capturePoint);    // TODO: is this what we want?
      FramePoint2d finalDesiredICP = getSingleSupportFinalDesiredICPForWalking(desiredFootstep, swingSide);
      icpTrajectoryGenerator.initialize(desiredICP.getFramePoint2dCopy(), finalDesiredICP, footCartesianTrajectoryGenerators.get(swingSide).getFinalTime(),
                                        omega0, amountToBeInsideSingleSupport.getDoubleValue());

      limbNullspaceMultipliers.get(swingSide).get(LimbName.LEG).set(0.0);
      trajectoryInitialized.get(swingSide).set(true);

      // TODO: orientation stuff

      stateMachine.doAction();    // computes trajectory and stores results in YoFrameVectors and Points.
   }

   public boolean trajectoryInitialized(RobotSide robotSide)
   {
      return trajectoryInitialized.get(robotSide).getBooleanValue();
   }

   private void evaluateCoMTrajectory()
   {
      centerOfMassHeightTrajectoryGenerator.compute();
      comTrajectoryCounter++;

      if (comTrajectoryCounter >= 500)
      {
         FramePoint desiredCoM = new FramePoint(referenceFrames.getCenterOfMassFrame());
         desiredCoM.changeFrame(worldFrame);
         desiredCoM.setY(0.0);
         desiredCoM.setZ(centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeight());
         comTrajectoryBagOfBalls.setBallLoop(desiredCoM);
         comTrajectoryCounter = 0;
      }
   }

   public void doControl()
   {
      
      stateMachine.checkTransitionConditionsThoroughly();
      stateMachine.doAction();
      desiredCoMHeightAcceleration.set(computeDesiredCoMHeightAcceleration(desiredICPVelocity.getFrameVector2dCopy()));
      doChestControl();
   }

   private void setSwingControlGains(RobotSide swingSide)
   {
      RigidBodySpatialAccelerationControlModule footSpatialAccelerationControlModule = footSpatialAccelerationControlModules.get(swingSide);
      footSpatialAccelerationControlModule.setPositionProportionalGains(100.0, 100.0, 100.0);
      footSpatialAccelerationControlModule.setPositionDerivativeGains(20.0, 20.0, 20.0);

      footSpatialAccelerationControlModule.setOrientationProportionalGains(500.0, 500.0, 500.0);
      footSpatialAccelerationControlModule.setOrientationDerivativeGains(80.0, 80.0, 80.0);
   }

   private void setStanceControlGains(RobotSide supportSide)
   {
      RigidBodySpatialAccelerationControlModule footSpatialAccelerationControlModule = footSpatialAccelerationControlModules.get(supportSide);

      // these need to be zero right now, because position trajectory doesn't match orientation trajectory. projection onto motion subspace will fix inconsistent acceleration.
      footSpatialAccelerationControlModule.setPositionProportionalGains(0.0, 0.0, 0.0);
      footSpatialAccelerationControlModule.setPositionDerivativeGains(0.0, 0.0, 0.0);
      footSpatialAccelerationControlModule.setOrientationProportionalGains(500.0, 500.0, 500.0);
      footSpatialAccelerationControlModule.setOrientationDerivativeGains(75.0, 75.0, 75.0);
   }

   private void doChestControl()
   {
      ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      FrameOrientation desiredOrientation = new FrameOrientation(pelvisFrame);
      FrameVector desiredAngularVelocity = new FrameVector(pelvisFrame);
      FrameVector desiredAngularAcceleration = new FrameVector(pelvisFrame);

      FrameVector outputToPack = new FrameVector(fullRobotModel.getChest().getBodyFixedFrame());
      chestOrientationControlModule.controlSpine(outputToPack, desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      chestAngularAccelerationWithRespectToPelvis.set(outputToPack);
   }

   private double computeDesiredCoMHeightAcceleration(FrameVector2d desiredICPVelocity)
   {
      double zDesired = centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeight();
      double dzdxDesired = centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeightSlope();
      double d2zdx2Desired = centerOfMassHeightTrajectoryGenerator.getDesiredCenterOfMassHeightSecondDerivative();

      ReferenceFrame stairDirectionFrame = worldFrame;

      desiredICPVelocity.changeFrame(stairDirectionFrame);

      FramePoint com = new FramePoint(referenceFrames.getCenterOfMassFrame());
      com.changeFrame(stairDirectionFrame);

      // TODO: computing Jacobian a bunch of times...
      FrameVector comd = new FrameVector(stairDirectionFrame);
      centerOfMassJacobian.compute();
      centerOfMassJacobian.packCenterOfMassVelocity(comd);
      comd.changeFrame(stairDirectionFrame);

      // TODO: use current omega0 instead of previous
      double xd = comd.getX();
      double icpdx = desiredICPVelocity.getX();
      double xdd = omega0 * (icpdx - xd);    // MathTools.square(omega0.getDoubleValue()) * (com.getX() - copX);

      double zdDesired = dzdxDesired * xd;
      double zddFeedForward = d2zdx2Desired * MathTools.square(xd) + dzdxDesired * xdd;

      double zCurrent = com.getZ();
      double zdCurrent = comd.getZ();

      double zddDesired = centerOfMassHeightController.compute(zCurrent, zDesired, zdCurrent, zdDesired) + zddFeedForward;
      double epsilon = 1e-12;
      zddDesired = MathTools.clipToMinMax(zddDesired, -gravity + epsilon, Double.POSITIVE_INFINITY);

      return zddDesired;
   }
}
