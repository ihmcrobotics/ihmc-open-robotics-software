package us.ihmc.exampleSimulations.sphereICPControl.controllers;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.YoICPControlGains;
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
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class SphereNewICPController implements GenericSphereController
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

   private final BagOfBalls cmpTrack;
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
   private final YoFramePoint3D desiredICP;
   private final YoFrameVector3D desiredICPVelocity;

   private final YoFramePoint3D yoDesiredCMP;

   private final ICPProportionalController icpController;
   private final YoICPControlGains icpGains;
   private final double omega0;
   private final double totalMass;

   private final int numberOfBalls = 100;
   private final int simulatedTicksPerGraphicUpdate = 16;

   private final YoDouble yoTime;

   public SphereNewICPController(SphereControlToolbox controlToolbox, YoVariableRegistry parentRegistry)
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

      omega0 = controlToolbox.getOmega0();
      heightController = new BasicHeightController(controlToolbox, registry);
      icpPlanner = new SmoothCMPBasedICPPlanner(controlToolbox.getFullRobotModel(), controlToolbox.getBipedSupportPolygons(), controlToolbox.getSoleZUpFrames(),
                                                controlToolbox.getContactableFeet(), null, null, registry, yoGraphicsListRegistry, 9.81,
                                                controlToolbox.getNewCapturePointPlannerParameters());
      icpPlanner.setDefaultPhaseTimes(1.0, 1.0);
      icpPlanner.setOmega0(omega0);

      icpGains = new YoICPControlGains("CoMController", registry);
      icpGains.setKpOrthogonalToMotion(3.0);
      icpGains.setKpParallelToMotion(2.0);

      icpController = new ICPProportionalController(icpGains, controlToolbox.getControlDT(), registry);

      StateMachineFactory<SupportState, State> factory = new StateMachineFactory<>(SupportState.class);
      factory.setNamePrefix("supportStateMachine").setRegistry(registry).buildYoClock(controlToolbox.getYoTime());

      factory.addStateAndDoneTransition(SupportState.STANDING, new StandingState(), SupportState.DOUBLE);
      factory.addStateAndDoneTransition(SupportState.DOUBLE, new DoubleSupportState(), SupportState.SINGLE);
      factory.addState(SupportState.SINGLE, new SingleSupportState());

      factory.addTransition(SupportState.DOUBLE, SupportState.SINGLE, new TransitionToSingleSupportCondition());
      factory.addTransition(SupportState.SINGLE, SupportState.DOUBLE, new TransitionToDoubleSupportCondition());
      factory.addTransition(SupportState.SINGLE, SupportState.STANDING, new TransitionToStandingCondition());

      stateMachine = factory.build(SupportState.STANDING);

      cmpTrack = new BagOfBalls(numberOfBalls, 0.01, "eCMP", YoAppearance.Purple(), registry, yoGraphicsListRegistry);
      icpTrack = new BagOfBalls(numberOfBalls, 0.01, "ICP", YoAppearance.Yellow(), registry, yoGraphicsListRegistry);
      comTrack = new BagOfBalls(numberOfBalls, 0.01, "CoM", YoAppearance.Black(), registry, yoGraphicsListRegistry);

      YoGraphicVector forceVisualizer = new YoGraphicVector("forceViz", yoDesiredCMP, desiredForces, 0.05, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("forceViz", forceVisualizer);

      parentRegistry.addChild(registry);
   }

   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FramePoint3D desiredCapturePoint = new FramePoint3D();
   private final FramePoint3D finalDesiredCapturePoint = new FramePoint3D();
   private final FrameVector3D desiredCapturePointVelocity = new FrameVector3D();
   private final FramePoint2D desiredCapturePoint2d = new FramePoint2D();
   private final FramePoint2D finalDesiredCapturePoint2d = new FramePoint2D();
   private final FrameVector2D desiredCapturePointVelocity2d = new FrameVector2D();
   private final FramePoint2D perfectCMP = new FramePoint2D();
   private final FramePoint2D previousPerfectCMP = new FramePoint2D();

   private int counter = 0;

   @Override
   public void doControl()
   {
      stateMachine.doActionAndTransition();

      heightController.doControl();

      controlToolbox.update();

      capturePoint2d.set(icp);
      icpPlanner.compute(yoTime.getDoubleValue());
      icpPlanner.getDesiredCapturePointPosition(desiredCapturePoint);
      icpPlanner.getDesiredCapturePointVelocity(desiredCapturePointVelocity);
      icpPlanner.getFinalDesiredCapturePointPosition(finalDesiredCapturePoint);
      desiredICP.set(desiredCapturePoint);
      desiredCapturePointVelocity.set(desiredCapturePointVelocity);

      desiredCapturePoint2d.set(desiredCapturePoint);
      desiredCapturePointVelocity2d.set(desiredCapturePointVelocity);
      finalDesiredCapturePoint2d.set(finalDesiredCapturePoint);

      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredCapturePoint2d, desiredCapturePointVelocity2d, omega0, perfectCMP);
      FramePoint2D desiredCMP = icpController.doProportionalControl(previousPerfectCMP, capturePoint2d, desiredCapturePoint2d, finalDesiredCapturePoint2d,
                                                                    desiredCapturePointVelocity2d, perfectCMP, omega0);
      previousPerfectCMP.setIncludingFrame(perfectCMP);

      double fZ = heightController.getVerticalForce();
      FrameVector3D reactionForces = computeGroundReactionForce(desiredCMP, fZ);
      reactionForces.changeFrame(worldFrame);
      planarForces.set(reactionForces);

      desiredCMP.changeFrame(worldFrame);
      yoDesiredCMP.set(desiredCMP, 0.0);

      if (counter++ % simulatedTicksPerGraphicUpdate == 0)
      {
         centerOfMass.setToZero(centerOfMassFrame);
         centerOfMass.changeFrame(worldFrame);

         icpTrack.setBallLoop(desiredICP);
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
      WrenchDistributorTools.computePseudoCMP3d(cmp3d, centerOfMass, cmp2d, fZ, totalMass, omega0);

      centerOfMass.setToZero(centerOfMassFrame);
      WrenchDistributorTools.computeForce(groundReactionForce, centerOfMass, cmp3d, fZ);
      groundReactionForce.changeFrame(centerOfMassFrame);

      return groundReactionForce;
   }

   private class StandingState implements State
   {
      @Override
      public boolean isDone(double timeInState)
      {
         return controlToolbox.hasFootsteps();
      }

      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         desiredCapturePoint.set(desiredICP);

         icpPlanner.clearPlan();
         icpPlanner.holdCurrentICP(desiredCapturePoint);
         icpPlanner.initializeForStanding(yoTime.getDoubleValue());

         for (RobotSide robotSide : RobotSide.values)
            contactStates.get(robotSide).setFullyConstrained();

         isInDoubleSupport.set(true);
      }

      @Override
      public void onExit()
      {

      }
   }

   private final FootstepTiming timing = new FootstepTiming();

   private class SingleSupportState implements State
   {
      @Override
      public boolean isDone(double timeInState)
      {
         return icpPlanner.isDone();
      }

      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         icpPlanner.clearPlan();

         Footstep nextFootstep = controlToolbox.getFootstep(0);
         Footstep nextNextFootstep = controlToolbox.peekAtFootstep(0);
         Footstep nextNextNextFootstep = controlToolbox.peekAtFootstep(1);

         controlToolbox.updateUpcomingFootstepsViz(nextFootstep, nextNextFootstep, nextNextNextFootstep);

         timing.setTimings(controlToolbox.getSingleSupportDuration(), controlToolbox.getDoubleSupportDuration());
         icpPlanner.addFootstepToPlan(nextFootstep, timing);
         icpPlanner.addFootstepToPlan(nextNextFootstep, timing);
         icpPlanner.addFootstepToPlan(nextNextNextFootstep, timing);

         RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();
         icpPlanner.setSupportLeg(supportSide);
         icpPlanner.initializeForSingleSupport(yoTime.getDoubleValue());

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

         isInDoubleSupport.set(false);
      }

      @Override
      public void onExit()
      {

      }
   }

   private class DoubleSupportState implements State
   {
      @Override
      public boolean isDone(double timeInState)
      {
         return icpPlanner.isDone();
      }

      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onEntry()
      {
         icpPlanner.clearPlan();

         for (RobotSide robotSide : RobotSide.values)
            contactStates.get(robotSide).setFullyConstrained();
         controlToolbox.getBipedSupportPolygons().updateUsingContactStates(contactStates);

         Footstep nextFootstep = controlToolbox.peekAtFootstep(0);
         Footstep nextNextFootstep = controlToolbox.peekAtFootstep(1);
         Footstep nextNextNextFootstep = controlToolbox.peekAtFootstep(2);

         controlToolbox.updateUpcomingFootstepsViz(nextFootstep, nextNextFootstep, nextNextNextFootstep);

         timing.setTimings(controlToolbox.getSingleSupportDuration(), controlToolbox.getDoubleSupportDuration());
         icpPlanner.addFootstepToPlan(nextFootstep, timing);
         icpPlanner.addFootstepToPlan(nextNextFootstep, timing);
         icpPlanner.addFootstepToPlan(nextNextNextFootstep, timing);

         RobotSide transferToSide = nextFootstep.getRobotSide().getOppositeSide();

         icpPlanner.setTransferToSide(transferToSide);
         icpPlanner.initializeForTransfer(yoTime.getDoubleValue());

         isInDoubleSupport.set(true);
      }

      @Override
      public void onExit()
      {

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

   private class TransitionToSingleSupportCondition implements StateTransitionCondition
   {
      @Override
      public boolean testCondition(double timeInState)
      {
         return icpPlanner.isDone();
      }
   }
}
