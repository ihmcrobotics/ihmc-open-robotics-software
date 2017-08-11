package us.ihmc.exampleSimulations.sphereICPControl.controllers;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ContinuousCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPAdjustmentOptimizationController;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationController;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.YoICPControlGains;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;

public class SphereICPOptimizationController implements GenericSphereController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private enum SupportState {STANDING, DOUBLE, SINGLE}

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint2d planarForces = new YoFramePoint2d("planarICPForces", worldFrame, registry);
   private final YoFrameVector desiredForces = new YoFrameVector("desiredForces", worldFrame, registry);
   private final YoBoolean isInDoubleSupport = new YoBoolean("isInDoubleSupport", registry);
   private final ArrayList<Footstep> nextFootsteps = new ArrayList<>();

   private final BagOfBalls cmpTrack;
   private final BagOfBalls icpReferenceTrack;
   private final BagOfBalls icpTrack;
   private final BagOfBalls comTrack;

   private final SphereControlToolbox controlToolbox;
   private final BasicHeightController heightController;

   private final StateMachine<SupportState> stateMachine;

   private final ContinuousCMPBasedICPPlanner icpPlanner;

   private final ReferenceFrame centerOfMassFrame;

   private final SideDependentList<YoPlaneContactState> contactStates;
   private final SideDependentList<FootSpoof> contactableFeet;
   private final SideDependentList<FramePose> footPosesAtTouchdown;

   private final YoFramePoint icp;
   private final YoFramePoint desiredICP;
   private final YoFrameVector desiredICPVelocity;

   private final YoFramePoint yoDesiredCMP;

   private final ICPOptimizationController icpOptimizationController;
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
      desiredICP = controlToolbox.getDesiredICP();
      desiredICPVelocity = controlToolbox.getDesiredICPVelocity();

      yoDesiredCMP = controlToolbox.getDesiredCMP();

      YoGraphicsListRegistry yoGraphicsListRegistry = controlToolbox.getYoGraphicsListRegistry();

      omega0.set(controlToolbox.getOmega0());
      heightController = new BasicHeightController(controlToolbox, registry);
      icpPlanner = new ContinuousCMPBasedICPPlanner(controlToolbox.getBipedSupportPolygons(), controlToolbox.getContactableFeet(),
            controlToolbox.getCapturePointPlannerParameters().getNumberOfFootstepsToConsider(), registry, yoGraphicsListRegistry);
      icpPlanner.setOmega0(omega0.getDoubleValue());
      icpPlanner.initializeParameters(controlToolbox.getCapturePointPlannerParameters());

      icpGains = new YoICPControlGains("CoMController", registry);
      icpGains.setKpOrthogonalToMotion(3.0);
      icpGains.setKpParallelToMotion(2.0);

      icpOptimizationController = new ICPAdjustmentOptimizationController(controlToolbox.getCapturePointPlannerParameters(), controlToolbox.getICPOptimizationParameters(),
                                                                          null, controlToolbox.getBipedSupportPolygons(),
                                                                          controlToolbox.getICPControlPolygons(), controlToolbox.getContactableFeet(), controlToolbox.getControlDT(), registry, yoGraphicsListRegistry);

      stateMachine = new StateMachine<>("supportStateMachine", "supportStateTime", SupportState.class, controlToolbox.getYoTime(), registry);
      StandingState standingState = new StandingState();
      DoubleSupportState doubleSupportState = new DoubleSupportState();
      SingleSupportState singleSupportState = new SingleSupportState();

      standingState.setDefaultNextState(doubleSupportState.getStateEnum());
      doubleSupportState.setDefaultNextState(singleSupportState.getStateEnum());

      singleSupportState.addStateTransition(new StateTransition<>(doubleSupportState.getStateEnum(), new TransitionToDoubleSupportCondition()));
      singleSupportState.addStateTransition(new StateTransition<>(standingState.getStateEnum(), new TransitionToStandingCondition()));

      stateMachine.addState(standingState);
      stateMachine.addState(doubleSupportState);
      stateMachine.addState(singleSupportState);
      stateMachine.setCurrentState(SupportState.STANDING);

      cmpTrack = new BagOfBalls(numberOfBalls, 0.01, "eCMP", YoAppearance.Red(), registry, yoGraphicsListRegistry);
      icpReferenceTrack = new BagOfBalls(numberOfBalls, 0.01, "ICPReference", YoAppearance.Yellow(), registry, yoGraphicsListRegistry);
      icpTrack = new BagOfBalls(numberOfBalls, 0.01, "ICP", YoAppearance.Blue(), registry, yoGraphicsListRegistry);
      comTrack = new BagOfBalls(numberOfBalls, 0.01, "CoM", YoAppearance.Black(), registry, yoGraphicsListRegistry);

      YoGraphicVector forceVisualizer = new YoGraphicVector("forceViz", yoDesiredCMP, desiredForces, 0.05, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("forceViz", forceVisualizer);

      parentRegistry.addChild(registry);
   }

   private final FramePoint2d capturePoint2d = new FramePoint2d();
   private final FramePoint desiredCapturePoint = new FramePoint();
   private final FramePoint finalDesiredCapturePoint = new FramePoint();
   private final FrameVector desiredCapturePointVelocity = new FrameVector();
   private final FramePoint2d desiredCapturePoint2d = new FramePoint2d();
   private final FramePoint2d finalDesiredCapturePoint2d = new FramePoint2d();
   private final FrameVector2d desiredCapturePointVelocity2d = new FrameVector2d();
   private final FramePoint2d perfectCMP = new FramePoint2d();

   private int counter = 0;
   public void doControl()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      heightController.doControl();

      controlToolbox.update();

      icp.getFrameTuple2d(capturePoint2d);
      icpPlanner.compute(yoTime.getDoubleValue());
      icpPlanner.getDesiredCapturePointPosition(desiredCapturePoint);
      icpPlanner.getDesiredCapturePointVelocity(desiredCapturePointVelocity);
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredCapturePoint);
      desiredICP.set(desiredCapturePoint);
      desiredCapturePointVelocity.set(desiredCapturePointVelocity);

      desiredCapturePoint2d.setByProjectionOntoXYPlane(desiredCapturePoint);
      desiredCapturePointVelocity2d.setByProjectionOntoXYPlane(desiredCapturePointVelocity);
      finalDesiredCapturePoint2d.setByProjectionOntoXYPlane(finalDesiredCapturePoint);

      FramePoint2d desiredCMP = yoDesiredCMP.getFramePoint2dCopy();

      double fZ = heightController.getVerticalForce();
      FrameVector reactionForces = computeGroundReactionForce(desiredCMP, fZ);
      reactionForces.changeFrame(worldFrame);
      planarForces.setByProjectionOntoXYPlane(reactionForces);

      if (counter++ % simulatedTicksPerGraphicUpdate == 0)
      {
         icpReferenceTrack.setBallLoop(desiredICP.getFramePointCopy());
         icpTrack.setBallLoop(icp.getFramePointCopy());
         cmpTrack.setBallLoop(yoDesiredCMP.getFramePointCopy());
         comTrack.setBallLoop(centerOfMass);
      }
   }

   public Vector3D getForces()
   {
      desiredForces.setX(planarForces.getX());
      desiredForces.setY(planarForces.getY());
      desiredForces.setZ(heightController.getVerticalForce());

      return desiredForces.getVector3dCopy();
   }

   private final FramePoint cmp3d = new FramePoint();
   private final FrameVector groundReactionForce = new FrameVector();
   private final FramePoint centerOfMass = new FramePoint();
   private FrameVector computeGroundReactionForce(FramePoint2d cmp2d, double fZ)
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
         FramePose nextSupportPose = footPosesAtTouchdown.get(supportSide.getOppositeSide());
         nextSupportPose.setToZero(nextFootstep.getSoleReferenceFrame());
         nextSupportPose.changeFrame(ReferenceFrame.getWorldFrame());
         footSpoof.setSoleFrame(nextSupportPose);
      }
   }

   private class StandingState extends State<SupportState>
   {
      private final FramePoint2d desiredCMP = new FramePoint2d();

      public StandingState()
      {
         super(SupportState.STANDING);
      }

      @Override public void doAction()
      {
         if (controlToolbox.hasFootsteps())
            this.transitionToDefaultNextState();

         icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCMP, capturePoint2d, omega0.getDoubleValue());
         icpOptimizationController.getDesiredCMP(desiredCMP);
         yoDesiredCMP.setXY(desiredCMP);
      }

      @Override public void doTransitionIntoAction()
      {
         nextFootsteps.clear();

         desiredICP.getFrameTuple(desiredCapturePoint);

         icpPlanner.clearPlan();
         icpPlanner.holdCurrentICP(desiredCapturePoint);
         icpPlanner.initializeForStanding(yoTime.getDoubleValue());

         icpPlanner.clearPlan();
         icpOptimizationController.initializeForStanding(yoTime.getDoubleValue());

         for (RobotSide robotSide : RobotSide.values)
            contactStates.get(robotSide).setFullyConstrained();
      }

      @Override public void doTransitionOutOfAction()
      {

      }
   }

   private final FootstepTiming timing = new FootstepTiming();
   private class SingleSupportState extends State<SupportState>
   {
      private final FramePose footstepPose = new FramePose();
      private final FramePoint2d footstepPositionSolution = new FramePoint2d();
      private final FramePoint2d desiredCMP = new FramePoint2d();

      public SingleSupportState()
      {
         super(SupportState.SINGLE);
      }

      @Override public void doAction()
      {
         icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCMP, capturePoint2d, omega0.getDoubleValue());
         icpOptimizationController.getDesiredCMP(desiredCMP);
         yoDesiredCMP.setXY(desiredCMP);

         for (int i = 0; i < icpOptimizationController.getNumberOfFootstepsToConsider(); i++)
         {
            Footstep footstep = nextFootsteps.get(i);

            if (footstep != null)
            {
               footstep.getPose(footstepPose);
               icpOptimizationController.getFootstepSolution(i, footstepPositionSolution);
               footstepPose.setXYFromPosition2d(footstepPositionSolution);
               footstep.setPose(footstepPose);
            }
         }

         updateViz(false);
      }

      @Override public void doTransitionIntoAction()
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

         for (int i = 3; i < icpOptimizationController.getNumberOfFootstepsToConsider(); i++)
            nextFootsteps.add(controlToolbox.peekAtFootstep(i - 1));

         timing.setTimings(controlToolbox.getDoubleSupportDuration(), controlToolbox.getSingleSupportDuration());
         icpPlanner.addFootstepToPlan(nextFootstep, timing);
         icpPlanner.addFootstepToPlan(nextNextFootstep, timing);
         icpPlanner.addFootstepToPlan(nextNextNextFootstep, timing);

         icpOptimizationController.addFootstepToPlan(nextFootstep, timing);
         icpOptimizationController.addFootstepToPlan(nextNextFootstep, timing);
         icpOptimizationController.addFootstepToPlan(nextNextNextFootstep, timing);

         RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();

         icpPlanner.setSupportLeg(supportSide);
         icpPlanner.initializeForSingleSupport(yoTime.getDoubleValue());

         icpOptimizationController.initializeForSingleSupport(yoTime.getDoubleValue(), supportSide, omega0.getDoubleValue());

         FootSpoof footSpoof = contactableFeet.get(supportSide.getOppositeSide());
         FramePose nextSupportPose = footPosesAtTouchdown.get(supportSide.getOppositeSide());
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

      @Override public void doTransitionOutOfAction()
      {
      }
   }

   private class DoubleSupportState extends State<SupportState>
   {
      private final FramePose footstepPose = new FramePose();
      private final FramePoint2d footstepPositionSolution = new FramePoint2d();
      private final FramePoint2d desiredCMP = new FramePoint2d();

      public DoubleSupportState()
      {
         super(SupportState.DOUBLE);
      }

      @Override public void doAction()
      {
         if (icpPlanner.isDone())
            transitionToDefaultNextState();

         icpOptimizationController.compute(yoTime.getDoubleValue(), desiredCapturePoint2d, desiredCapturePointVelocity2d, perfectCMP, capturePoint2d, omega0.getDoubleValue());
         icpOptimizationController.getDesiredCMP(desiredCMP);
         yoDesiredCMP.setXY(desiredCMP);

         if (getPreviousState().getStateEnum() != SupportState.STANDING)
         {
            for (int i = 0; i < icpOptimizationController.getNumberOfFootstepsToConsider(); i++)
            {
               Footstep footstep = nextFootsteps.get(i);

               if (footstep != null)
               {
                  footstep.getPose(footstepPose);
                  icpOptimizationController.getFootstepSolution(i, footstepPositionSolution);
                  footstepPose.setXYFromPosition2d(footstepPositionSolution);
                  footstep.setPose(footstepPose);
               }
            }
         }

         updateViz(true);
      }

      @Override public void doTransitionIntoAction()
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

         for (int i = 3; i < icpOptimizationController.getNumberOfFootstepsToConsider(); i++)
            nextFootsteps.add(controlToolbox.peekAtFootstep(i - 1));

         controlToolbox.updateUpcomingFootstepsViz(nextFootstep, nextNextFootstep, nextNextNextFootstep);

         timing.setTimings(controlToolbox.getDoubleSupportDuration(), controlToolbox.getSingleSupportDuration());
         icpPlanner.addFootstepToPlan(nextFootstep, timing);
         icpPlanner.addFootstepToPlan(nextNextFootstep, timing);
         icpPlanner.addFootstepToPlan(nextNextNextFootstep, timing);

         icpOptimizationController.addFootstepToPlan(nextFootstep, timing);
         icpOptimizationController.addFootstepToPlan(nextNextFootstep, timing);
         icpOptimizationController.addFootstepToPlan(nextNextNextFootstep, timing);

         RobotSide transferToSide = nextFootstep.getRobotSide().getOppositeSide();

         icpPlanner.setTransferToSide(transferToSide);
         icpPlanner.initializeForTransfer(yoTime.getDoubleValue());

         icpOptimizationController.initializeForTransfer(yoTime.getDoubleValue(), transferToSide, omega0.getDoubleValue());

         updateViz(true);
      }

      @Override public void doTransitionOutOfAction()
      {

      }
   }

   private class TransitionToStandingCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         if (icpPlanner.isDone())
            return !controlToolbox.hasFootsteps();

         return false;
      }
   }

   private class TransitionToDoubleSupportCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         if (icpPlanner.isDone())
            return controlToolbox.hasFootsteps();

         return false;
      }
   }
}
