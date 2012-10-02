package us.ihmc.commonWalkingControlModules.momentumBasedController;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FootPolygonEnum;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.SimpleWorldDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.commonWalkingControlModules.trajectories.CartesianTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.FifthOrderWaypointCartesianTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.FlatThenPolynomialCoMHeightTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionAction;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;

public class WalkingHighLevelHumanoidController extends AbstractHighLevelHumanoidController
{
   private static enum WalkingState {LEFT_SUPPORT, RIGHT_SUPPORT, TRANSFER_TO_LEFT_SUPPORT, TRANSFER_TO_RIGHT_SUPPORT, DOUBLE_SUPPORT}

   private final StateMachine stateMachine;

   private final SideDependentList<WalkingState> singleSupportStateEnums =
      new SideDependentList<WalkingHighLevelHumanoidController.WalkingState>(WalkingState.LEFT_SUPPORT, WalkingState.RIGHT_SUPPORT);

   private final SideDependentList<WalkingState> transferStateEnums =
      new SideDependentList<WalkingHighLevelHumanoidController.WalkingState>(WalkingState.TRANSFER_TO_LEFT_SUPPORT, WalkingState.TRANSFER_TO_RIGHT_SUPPORT);

   private final double doubleSupportTime = 0.2;    // 0.6;    // 0.3
   private final double stepTime = 0.45;    // 0.5; // 0.55;    // 0.55;
   private final double waypointHeight = -0.13;    // 0.05; // 0.15;

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

   private final FlatThenPolynomialCoMHeightTrajectoryGenerator flatThenPolynomialCoMHeightTrajectoryGenerator;    // TODO: kind of ugly, need this because one of the methods we use here is not in the CenterOfMassHeightTrajectoryGenerator interface

   public WalkingHighLevelHumanoidController(FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames, TwistCalculator twistCalculator,
           SideDependentList<BipedFootInterface> bipedFeet, BipedSupportPolygons bipedSupportPolygons, SideDependentList<FootSwitchInterface> footSwitches,
           ProcessedSensorsInterface processedSensors, DoubleYoVariable t, double controlDT, DesiredHeadingControlModule desiredHeadingControlModule,
           YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(fullRobotModel, referenceFrames, processedSensors, twistCalculator, bipedFeet, bipedSupportPolygons, controlDT, desiredHeadingControlModule,
            footSwitches, registry);
      SimpleWorldDesiredFootstepCalculator simpleDesiredFootstepCalculator = new SimpleWorldDesiredFootstepCalculator(bipedFeet, referenceFrames,
                                                                                desiredHeadingControlModule, registry);

//    SimpleDesiredFootstepCalculator simpleDesiredFootstepCalculator = new SimpleDesiredFootstepCalculator(referenceFrames.getAnkleZUpReferenceFrames(),
//    desiredHeadingControlModule, registry); // TODO: pass in
//    BoxDesiredFootstepCalculator boxDesiredFootstepCalculator = new BoxDesiredFootstepCalculator(referenceFrames.getAnkleZUpReferenceFrames(),
//          desiredHeadingControlModule, registry);
//    boxDesiredFootstepCalculator.setupParametersForR2();
//    this.desiredFootstepCalculator = boxDesiredFootstepCalculator;
      simpleDesiredFootstepCalculator.setupParametersForR2InverseDynamics();
      this.desiredFootstepCalculator = simpleDesiredFootstepCalculator;


      // this.centerOfMassHeightTrajectoryGenerator = new OrbitalEnergyCubicTrajectoryGenerator(processedSensors, referenceFrames,
      // desiredHeadingControlModule.getDesiredHeadingFrame(), footHeight, parentRegistry);

      // this.centerOfMassHeightTrajectoryGenerator = new LinearFootstepCalculatorBasedCoMHeightTrajectoryGenerator(processedSensors, desiredFootstepCalculator,
      // referenceFrames, desiredHeadingControlModule.getDesiredHeadingFrame(), registry);

      // this.centerOfMassHeightTrajectoryGenerator = new ConstantCenterOfMassHeightTrajectoryGenerator(registry);
      this.flatThenPolynomialCoMHeightTrajectoryGenerator = new FlatThenPolynomialCoMHeightTrajectoryGenerator("", processedSensors,
              simpleDesiredFootstepCalculator, desiredHeadingControlModule.getDesiredHeadingFrame(), bipedFeet, referenceFrames, registry);
      this.centerOfMassHeightTrajectoryGenerator = flatThenPolynomialCoMHeightTrajectoryGenerator;

      String namePrefix = "walking";
      this.stateMachine = new StateMachine(namePrefix + "State", namePrefix + "SwitchTime", WalkingState.class, t, registry);
      upcomingSupportLeg.set(RobotSide.LEFT);


      singleSupportICPGlideScaleFactor.set(0.9);
      FramePoint2d finalDesiredICPForDoubleSupportStance = getDoubleSupportFinalDesiredICPForDoubleSupportStance();
      finalDesiredICPForDoubleSupportStance.changeFrame(desiredICP.getReferenceFrame());

      for (RobotSide robotSide : RobotSide.values())
      {
         footCartesianTrajectoryGenerators.put(robotSide,
                 new FifthOrderWaypointCartesianTrajectoryGenerator(robotSide.getCamelCaseNameForStartOfExpression(), worldFrame, stepTime, waypointHeight,
                    registry));
         trajectoryInitialized.put(robotSide, new BooleanYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "TrajectoryInitialized", registry));
      }

      footTrajectoryBagOfBalls = new BagOfBalls(100, 0.01, "footBagOfBalls", YoAppearance.Black(), registry, dynamicGraphicObjectsListRegistry);
      comTrajectoryBagOfBalls = new BagOfBalls(500, 0.01, "comBagOfBalls", YoAppearance.Red(), registry, dynamicGraphicObjectsListRegistry);

      setUpStateMachine();

      walk.set(false);
      liftUpHeels.set(true);
      trailingFootPitch.set(0.05);
      leadingFootPitch.set(0.8);
      minOrbitalEnergyForSingleSupport.set(0.007);
      amountToBeInsideSingleSupport.set(0.0);
      amountToBeInsideDoubleSupport.set(0.05);
      desiredPelvisPitch.set(0.6);

      for (RobotSide robotSide : RobotSide.values())
      {
         controlModes.get(robotSide).get(LimbName.LEG).set(ControlMode.CARTESIAN_SPACE);
         controlModes.get(robotSide).get(LimbName.ARM).set(ControlMode.JOINT_SPACE);
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
               setDesiredFootPosVelAccForSupportSide(robotSide, footPitch);
            }
            else
            {
               bipedFoot.setFootPolygonInUse(FootPolygonEnum.FLAT);
               setDesiredFootPosVelAccForSupportSide(robotSide, 0.0);
            }
         }

//       if ((transferToSide != null) && icpTrajectoryGenerator.isDone() && icpTrajectoryHasBeenInitialized.getBooleanValue())
//       {
//          transferICPTrajectoryDone.set(true);
//       }

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

      private int counter = 0;

      public SingleSupportState(RobotSide robotSide)
      {
         super(singleSupportStateEnums.get(robotSide));
         this.swingSide = robotSide.getOppositeSide();
         supportAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide());
         this.positionToPack = new FramePoint(supportAnkleZUpFrame);
         this.velocityToPack = new FrameVector(supportAnkleZUpFrame);
         this.accelerationToPack = new FrameVector(supportAnkleZUpFrame);
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

         positionToPack.changeFrame(desiredEndEffectorPositions.get(swingSide).get(LimbName.LEG).getReferenceFrame());
         velocityToPack.changeFrame(desiredEndEffectorVelocities.get(swingSide).get(LimbName.LEG).getReferenceFrame());
         accelerationToPack.changeFrame(desiredEndEffectorAccelerations.get(swingSide).get(LimbName.LEG).getReferenceFrame());

         desiredEndEffectorPositions.get(swingSide).get(LimbName.LEG).set(positionToPack);
         desiredEndEffectorVelocities.get(swingSide).get(LimbName.LEG).set(velocityToPack);
         desiredEndEffectorAccelerations.get(swingSide).get(LimbName.LEG).set(accelerationToPack);

//       desiredFootOrientations.get(swingSide).setYawPitchRoll(0.0, 0.0, 0.0);
//       desiredFootAngularVelocities.get(swingSide).set(0.0, 0.0, 0.0);
//       desiredFootAngularAccelerations.get(swingSide).set(0.0, 0.0, 0.0);

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

         desiredEndEffectorOrientations.get(swingSide).get(LimbName.LEG).setYawPitchRoll(0.0, 0.0, 0.0);

         setDesiredFootPosVelAccForSupportSide(supportSide, trailingFootPitch.getDoubleValue());    // FIXME: make it a smooth trajectory
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
         double orbitalEnergy = flatThenPolynomialCoMHeightTrajectoryGenerator.computeOrbitalEnergyIfInitializedNow(getUpcomingSupportLeg());

//       return transferICPTrajectoryDone.getBooleanValue() && orbitalEnergy > minOrbitalEnergy;
         return orbitalEnergy > minOrbitalEnergyForSingleSupport.getDoubleValue();
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
      double trailingFootToLeadingFootFactor = 0.25;
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

      footstep.changeFrame(desiredHeadingControlModule.getDesiredHeadingFrame());

      Transform3D footstepTransform = new Transform3D();
      footstep.getFootstepPose().getTransform3D(footstepTransform);

      FramePoint2d centroid2d = footPolygonInSoleFrame.getCentroidCopy();
      FramePoint centroid = centroid2d.toFramePoint();
      centroid.changeFrame(referenceFrames.getFootFrame(robotSide));
      centroid.changeFrameUsingTransform(desiredHeadingControlModule.getDesiredHeadingFrame(), footstepTransform);
      FramePoint2d ret = centroid.toFramePoint2d();

      double extraX = 0.0;    // 0.02
      double extraY = robotSide.negateIfLeftSide(0.04);
      FrameVector2d offset = new FrameVector2d(desiredHeadingControlModule.getDesiredHeadingFrame(), extraX, extraY);
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

   private void setDesiredFootPosVelAccForSupportSide(RobotSide supportSide, double footPitch)
   {
      FramePoint desiredFootPosition = new FramePoint(referenceFrames.getFootFrame(supportSide));
      desiredFootPosition.changeFrame(desiredEndEffectorPositions.get(supportSide).get(LimbName.LEG).getReferenceFrame());
      desiredEndEffectorPositions.get(supportSide).get(LimbName.LEG).set(desiredFootPosition);
      desiredEndEffectorVelocities.get(supportSide).get(LimbName.LEG).set(0.0, 0.0, 0.0);
      desiredEndEffectorAccelerations.get(supportSide).get(LimbName.LEG).set(0.0, 0.0, 0.0);

      desiredEndEffectorOrientations.get(supportSide).get(LimbName.LEG).setYawPitchRoll(0.0, footPitch, 0.0);
      desiredEndEffectorAngularVelocities.get(supportSide).get(LimbName.LEG).set(0.0, 0.0, 0.0);
      desiredEndEffectorAngularAccelerations.get(supportSide).get(LimbName.LEG).set(0.0, 0.0, 0.0);
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

      desiredEndEffectorOrientations.get(swingSide).get(LimbName.LEG).set(
          desiredFootstep.getFootstepOrientationInFrame(desiredEndEffectorOrientations.get(swingSide).get(LimbName.LEG).getReferenceFrame()));

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
         FramePoint desiredCoM = processedSensors.getCenterOfMassPositionInFrame(worldFrame);
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
   }
}
