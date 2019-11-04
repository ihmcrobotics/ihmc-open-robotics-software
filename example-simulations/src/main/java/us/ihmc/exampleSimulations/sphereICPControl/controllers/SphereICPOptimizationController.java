package us.ihmc.exampleSimulations.sphereICPControl.controllers;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.YoICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationController;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationControllerInterface;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.SmoothCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class SphereICPOptimizationController implements GenericSphereController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private enum SupportState
   {
      STANDING, DOUBLE, SINGLE
   }

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint2D planarForces = new YoFramePoint2D("planarICPForces", worldFrame, registry);
   private final YoFrameVector3D desiredForces = new YoFrameVector3D("desiredForces", worldFrame, registry);
   private final YoBoolean isInDoubleSupport = new YoBoolean("isInDoubleSupport", registry);
   private final ArrayList<Footstep> nextFootsteps = new ArrayList<>();

   private final BagOfBalls cmpTrack;
   private final BagOfBalls icpReferenceTrack;
   private final BagOfBalls icpTrack;
   private final BagOfBalls comTrack;

   private final SphereControlToolbox controlToolbox;
   private final BasicHeightController heightController;

   private final StateMachine<SupportState, State> stateMachine;

   private final SmoothCMPBasedICPPlanner icpPlanner;

   private final ReferenceFrame centerOfMassFrame;

   private final SideDependentList<YoPlaneContactState> contactStates;
   private final SideDependentList<FootSpoof> contactableFeet;
   private final SideDependentList<FramePose3D> footPosesAtTouchdown;

   private final YoFramePoint3D icp;
   private final YoFrameVector3D icpVelocity;
   private final YoFramePoint3D desiredICP;
   private final YoFrameVector3D desiredICPVelocity;

   private final YoFramePoint3D yoDesiredCMP;

   private final ICPOptimizationControllerInterface icpOptimizationController;
   private final YoICPControlGains icpGains;
   private final YoDouble omega0 = new YoDouble("omega0", registry);
   private final double totalMass;

   private final int numberOfBalls = 100;
   private final int simulatedTicksPerGraphicUpdate = 16;

   private final YoDouble yoTime;

   public SphereICPOptimizationController(SphereControlToolbox controlToolbox, YoVariableRegistry parentRegistry)
   {
      this.controlToolbox = controlToolbox;

      isInDoubleSupport.set(true);
      contactStates = controlToolbox.getContactStates();
      contactableFeet = controlToolbox.getContactableFeet();
      yoTime = controlToolbox.getYoTime();
      footPosesAtTouchdown = controlToolbox.getFootPosesAtTouchdown();
      centerOfMassFrame = controlToolbox.getCenterOfMassFrame();
      totalMass = TotalMassCalculator.computeSubTreeMass(controlToolbox.getFullRobotModel().getElevator());

      icp = controlToolbox.getICP();
      icpVelocity = controlToolbox.getICPVelocity();
      desiredICP = controlToolbox.getDesiredICP();
      desiredICPVelocity = controlToolbox.getDesiredICPVelocity();

      yoDesiredCMP = controlToolbox.getDesiredCMP();

      YoGraphicsListRegistry yoGraphicsListRegistry = controlToolbox.getYoGraphicsListRegistry();

      omega0.set(controlToolbox.getOmega0());
      heightController = new BasicHeightController(controlToolbox, registry);
      icpPlanner = new SmoothCMPBasedICPPlanner(controlToolbox.getFullRobotModel(), controlToolbox.getBipedSupportPolygons(), controlToolbox.getSoleZUpFrames(),
                                                controlToolbox.getContactableFeet(), null, null, registry, yoGraphicsListRegistry, 9.81,
                                                controlToolbox.getNewCapturePointPlannerParameters());
      icpPlanner.setDefaultPhaseTimes(1.0, 1.0);
      icpPlanner.setOmega0(omega0.getValue());

      icpGains = new YoICPControlGains("CoMController", registry);
      icpGains.setKpOrthogonalToMotion(3.0);
      icpGains.setKpParallelToMotion(2.0);

      icpOptimizationController = new ICPOptimizationController(null, controlToolbox.getICPOptimizationParameters(), controlToolbox.getSoleZUpFrames(),
                                                                controlToolbox.getBipedSupportPolygons(), controlToolbox.getICPControlPolygons(),
                                                                controlToolbox.getContactableFeet(), controlToolbox.getControlDT(), registry, yoGraphicsListRegistry);

      StateMachineFactory<SupportState, State> factory = new StateMachineFactory<>(SupportState.class);
      factory.setNamePrefix("supportstateMachine").setRegistry(registry).buildYoClock(yoTime);

      factory.addStateAndDoneTransition(SupportState.STANDING, new StandingState(), SupportState.DOUBLE);
      factory.addStateAndDoneTransition(SupportState.DOUBLE, new DoubleSupportState(), SupportState.SINGLE);
      factory.addState(SupportState.SINGLE, new SingleSupportState());

      factory.addTransition(SupportState.SINGLE, SupportState.DOUBLE, new TransitionToDoubleSupportCondition());
      factory.addTransition(SupportState.SINGLE, SupportState.STANDING, new TransitionToStandingCondition());

      stateMachine = factory.build(SupportState.STANDING);

      cmpTrack = new BagOfBalls(numberOfBalls, 0.01, "eCMP", YoAppearance.Red(), registry, yoGraphicsListRegistry);
      icpReferenceTrack = new BagOfBalls(numberOfBalls, 0.01, "ICPReference", YoAppearance.Yellow(), registry, yoGraphicsListRegistry);
      icpTrack = new BagOfBalls(numberOfBalls, 0.01, "ICP", YoAppearance.Blue(), registry, yoGraphicsListRegistry);
      comTrack = new BagOfBalls(numberOfBalls, 0.01, "CoM", YoAppearance.Black(), registry, yoGraphicsListRegistry);

      YoGraphicVector forceVisualizer = new YoGraphicVector("forceViz", yoDesiredCMP, desiredForces, 0.05, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("forceViz", forceVisualizer);

      parentRegistry.addChild(registry);
      new DefaultParameterReader().readParametersInRegistry(registry);
   }

   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FrameVector2D capturePointVelocity2d = new FrameVector2D();
   private final FramePoint3D desiredCapturePoint = new FramePoint3D();
   private final FramePoint3D finalDesiredCapturePoint = new FramePoint3D();
   private final FrameVector3D desiredCapturePointVelocity = new FrameVector3D();
   private final FramePoint2D desiredCapturePoint2d = new FramePoint2D();
   private final FramePoint2D finalDesiredCapturePoint2d = new FramePoint2D();
   private final FrameVector2D desiredCapturePointVelocity2d = new FrameVector2D();
   private final FramePoint2D perfectCMP = new FramePoint2D();

   private int counter = 0;

   @Override
   public void doControl()
   {
      stateMachine.doActionAndTransition();

      heightController.doControl();

      controlToolbox.update();

      capturePoint2d.set(icp);
      capturePointVelocity2d.set(icpVelocity);
      icpPlanner.compute(yoTime.getDoubleValue());
      icpPlanner.getDesiredCapturePointPosition(desiredCapturePoint);
      icpPlanner.getDesiredCapturePointVelocity(desiredCapturePointVelocity);
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredCapturePoint);
      desiredICP.set(desiredCapturePoint);
      desiredCapturePointVelocity.set(desiredCapturePointVelocity);

      desiredCapturePoint2d.set(desiredCapturePoint);
      desiredCapturePointVelocity2d.set(desiredCapturePointVelocity);
      finalDesiredCapturePoint2d.set(finalDesiredCapturePoint);

      FramePoint2D desiredCMP = new FramePoint2D(yoDesiredCMP);

      double fZ = heightController.getVerticalForce();
      FrameVector3D reactionForces = computeGroundReactionForce(desiredCMP, fZ);
      reactionForces.changeFrame(worldFrame);
      planarForces.set(reactionForces);

      if (counter++ % simulatedTicksPerGraphicUpdate == 0)
      {
         centerOfMass.setToZero(centerOfMassFrame);
         centerOfMass.changeFrame(worldFrame);

         icpReferenceTrack.setBallLoop(desiredICP);
         icpTrack.setBallLoop(icp);
         cmpTrack.setBallLoop(yoDesiredCMP);
         comTrack.setBallLoop(centerOfMass);
      }
   }

   @Override
   public Vector3D getForces()
   {
      desiredForces.setX(planarForces.getX());
      desiredForces.setY(planarForces.getY());
      desiredForces.setZ(heightController.getVerticalForce());

      return new Vector3D(desiredForces);
   }

   private final FramePoint3D cmp3d = new FramePoint3D();
   private final FrameVector3D groundReactionForce = new FrameVector3D();
   private final FramePoint3D centerOfMass = new FramePoint3D();

   private FrameVector3D computeGroundReactionForce(FramePoint2D cmp2d, double fZ)
   {
      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computePseudoCMP3d(cmp3d, centerOfMass, cmp2d, fZ, totalMass, omega0.getDoubleValue());

      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computeForce(groundReactionForce, centerOfMass, cmp3d, fZ);
      groundReactionForce.changeFrame(centerOfMassFrame);

      return groundReactionForce;
   }

   private void updateViz(boolean isInTransfer)
   {
      Footstep nextFootstep = nextFootsteps.get(0);
      Footstep nextNextFootstep = nextFootsteps.get(1);
      Footstep nextNextNextFootstep = nextFootsteps.get(2);

      controlToolbox.updateUpcomingFootstepsViz(nextFootstep, nextNextFootstep, nextNextNextFootstep);

      if (!isInTransfer)
      {
         RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();
         FootSpoof footSpoof = contactableFeet.get(supportSide.getOppositeSide());
         FramePose3D nextSupportPose = footPosesAtTouchdown.get(supportSide.getOppositeSide());
         nextSupportPose.setToZero(nextFootstep.getSoleReferenceFrame());
         nextSupportPose.changeFrame(ReferenceFrame.getWorldFrame());
         footSpoof.setSoleFrame(nextSupportPose);
      }
   }

   private class StandingState implements State
   {
      private final FramePoint2D desiredCMP = new FramePoint2D();

      @Override
      public void doAction(double timeInState)
      {
         icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCMP, capturePoint2d,
                                           capturePointVelocity2d, omega0.getDoubleValue());
         icpOptimizationController.getDesiredCMP(desiredCMP);
         yoDesiredCMP.set(desiredCMP, 0.0);
      }

      @Override
      public void onEntry()
      {
         nextFootsteps.clear();

         desiredCapturePoint.set(desiredICP);

         icpPlanner.clearPlan();
         icpPlanner.holdCurrentICP(desiredCapturePoint);
         icpPlanner.initializeForStanding(yoTime.getDoubleValue());

         icpPlanner.clearPlan();
         icpOptimizationController.initializeForStanding(yoTime.getDoubleValue());

         for (RobotSide robotSide : RobotSide.values)
            contactStates.get(robotSide).setFullyConstrained();
      }

      @Override
      public void onExit()
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return controlToolbox.hasFootsteps();
      }
   }

   private final FootstepTiming timing = new FootstepTiming();
   private final FootstepShiftFractions shiftFractions = new FootstepShiftFractions();
   private final SimpleAdjustableFootstep footstep = new SimpleAdjustableFootstep();

   private class SingleSupportState implements State
   {
      private final FramePoint2D desiredCMP = new FramePoint2D();

      @Override
      public void doAction(double timeInState)
      {
         icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCMP, capturePoint2d,
                                           capturePointVelocity2d, omega0.getDoubleValue());
         icpOptimizationController.getDesiredCMP(desiredCMP);
         yoDesiredCMP.set(desiredCMP, 0.0);

         if (icpOptimizationController.useStepAdjustment())
         {
            Footstep footstep = nextFootsteps.get(0);

            if (footstep != null)
            {
               footstep.setPose(icpOptimizationController.getFootstepSolution());
            }
         }

         updateViz(false);
      }

      @Override
      public void onEntry()
      {
         nextFootsteps.clear();

         icpPlanner.clearPlan();
         icpOptimizationController.clearPlan();

         Footstep nextFootstep = controlToolbox.getFootstep(0);
         Footstep nextNextFootstep = controlToolbox.peekAtFootstep(0);
         Footstep nextNextNextFootstep = controlToolbox.peekAtFootstep(1);

         nextFootsteps.add(nextFootstep);
         nextFootsteps.add(nextNextFootstep);
         nextFootsteps.add(nextNextNextFootstep);

         timing.setTimings(controlToolbox.getDoubleSupportDuration(), controlToolbox.getSingleSupportDuration());
         icpPlanner.addFootstepToPlan(nextFootstep, timing, shiftFractions);
         icpPlanner.addFootstepToPlan(nextNextFootstep, timing, shiftFractions);
         icpPlanner.addFootstepToPlan(nextNextNextFootstep, timing, shiftFractions);

         footstep.set(nextFootstep);
         icpOptimizationController.addFootstepToPlan(footstep, timing.getSwingTime(), timing.getTransferTime());
         footstep.set(nextNextFootstep);
         icpOptimizationController.addFootstepToPlan(footstep, timing.getSwingTime(), timing.getTransferTime());
         footstep.set(nextNextNextFootstep);
         icpOptimizationController.addFootstepToPlan(footstep, timing.getSwingTime(), timing.getTransferTime());

         RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();

         icpPlanner.setSupportLeg(supportSide);
         icpPlanner.initializeForSingleSupport(yoTime.getDoubleValue());

         icpOptimizationController.initializeForSingleSupport(yoTime.getDoubleValue(), supportSide, omega0.getDoubleValue());

         FootSpoof footSpoof = contactableFeet.get(supportSide.getOppositeSide());
         FramePose3D nextSupportPose = footPosesAtTouchdown.get(supportSide.getOppositeSide());
         nextSupportPose.setToZero(nextFootstep.getSoleReferenceFrame());
         nextSupportPose.changeFrame(ReferenceFrame.getWorldFrame());
         footSpoof.setSoleFrame(nextSupportPose);

         contactStates.get(supportSide.getOppositeSide()).clear();
         if (nextFootstep.getPredictedContactPoints() == null)
            contactStates.get(supportSide.getOppositeSide()).setContactFramePoints(footSpoof.getContactPoints2d());
         else
            contactStates.get(supportSide.getOppositeSide()).setContactPoints(nextFootstep.getPredictedContactPoints());

         updateViz(false);
      }

      @Override
      public void onExit()
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return false;
      }
   }

   private class DoubleSupportState implements State
   {
      private final FramePoint2D desiredCMP = new FramePoint2D();

      @Override
      public void doAction(double timeInState)
      {
         icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCMP, capturePoint2d,
                                           capturePointVelocity2d, omega0.getDoubleValue());
         icpOptimizationController.getDesiredCMP(desiredCMP);
         yoDesiredCMP.set(desiredCMP, 0.0);

         if (stateMachine.getPreviousStateKey() != SupportState.STANDING)
         {
            if (icpOptimizationController.useStepAdjustment())
            {
               Footstep footstep = nextFootsteps.get(0);

               if (footstep != null)
               {
                  footstep.setPose(icpOptimizationController.getFootstepSolution());
               }
            }
         }

         updateViz(true);
      }

      @Override
      public void onEntry()
      {
         nextFootsteps.clear();

         icpPlanner.clearPlan();
         icpOptimizationController.clearPlan();

         for (RobotSide robotSide : RobotSide.values)
            contactStates.get(robotSide).setFullyConstrained();
         controlToolbox.getBipedSupportPolygons().updateUsingContactStates(contactStates);

         Footstep nextFootstep = controlToolbox.peekAtFootstep(0);
         Footstep nextNextFootstep = controlToolbox.peekAtFootstep(1);
         Footstep nextNextNextFootstep = controlToolbox.peekAtFootstep(2);

         nextFootsteps.add(nextFootstep);
         nextFootsteps.add(nextNextFootstep);
         nextFootsteps.add(nextNextNextFootstep);

         controlToolbox.updateUpcomingFootstepsViz(nextFootstep, nextNextFootstep, nextNextNextFootstep);

         timing.setTimings(controlToolbox.getDoubleSupportDuration(), controlToolbox.getSingleSupportDuration());
         icpPlanner.addFootstepToPlan(nextFootstep, timing, shiftFractions);
         icpPlanner.addFootstepToPlan(nextNextFootstep, timing, shiftFractions);
         icpPlanner.addFootstepToPlan(nextNextNextFootstep, timing, shiftFractions);

         footstep.set(nextFootstep);
         icpOptimizationController.addFootstepToPlan(footstep, timing.getSwingTime(), timing.getTransferTime());
         footstep.set(nextNextFootstep);
         icpOptimizationController.addFootstepToPlan(footstep, timing.getSwingTime(), timing.getTransferTime());
         footstep.set(nextNextNextFootstep);
         icpOptimizationController.addFootstepToPlan(footstep, timing.getSwingTime(), timing.getTransferTime());

         RobotSide transferToSide = nextFootstep.getRobotSide().getOppositeSide();

         icpPlanner.setTransferToSide(transferToSide);
         icpPlanner.initializeForTransfer(yoTime.getDoubleValue());

         icpOptimizationController.initializeForTransfer(yoTime.getDoubleValue(), transferToSide);

         updateViz(true);
      }

      @Override
      public void onExit()
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return icpPlanner.isDone();
      }
   }

   private class TransitionToStandingCondition implements StateTransitionCondition
   {
      @Override
      public boolean testCondition(double timeInState)
      {
         if (icpPlanner.isDone())
            return !controlToolbox.hasFootsteps();

         return false;
      }
   }

   private class TransitionToDoubleSupportCondition implements StateTransitionCondition
   {
      @Override
      public boolean testCondition(double timeInState)
      {
         if (icpPlanner.isDone())
            return controlToolbox.hasFootsteps();

         return false;
      }
   }
}
