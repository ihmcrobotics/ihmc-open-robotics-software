package us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.MPC;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.BPWPLanarWalkingRobot;
import us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.BPWPlanarWalkingRobotEstimates;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MPCTouchdownCalculator
{
   private final YoRegistry registry;

   private final YoDouble capturePointTimeConstantX;
   private final YoDouble capturePointTimeConstantY;
   private final YoDouble capturePointPole;

   private final AutomaticFootstepMPCMatrixCalculator mpcFootstepCalculator;

   private final YoFramePoint2D desiredMPCTouchdownPositionRelativeToCoM2D;
   private final YoFramePoint3D desiredMPCTouchdownPositionRelativeToCoM;
   private final YoFramePoint3D desiredMPCTouchdownPositionRelativeToStanceFoot;

   private final RobotSide swingSide;
   private final BPWPlanarWalkingRobotEstimates estimates;
   private final BPWPlanarWalkerParameters parameters;

   YoFrameVector2D predictedALIPVelocityAtTouchdown;


   public MPCTouchdownCalculator(BPWPLanarWalkingRobot controllerRobot, RobotSide swingSide,
                                 YoDouble desiredWalkingSpeed, YoDouble desiredSideWalkingSpeed,
                                 BPWPlanarWalkingRobotEstimates estimates, BPWPlanarWalkerParameters parameters,
                                 YoGraphicsListRegistry graphicsListRegistry, YoRegistry parentRegistry)
   {
      this.swingSide = swingSide;
      this.estimates = estimates;
      this.parameters = parameters;

      registry = new YoRegistry(swingSide.getLowerCaseName() + getClass().getSimpleName());

      FrameVector3D angularMomentumForMPCStuff = new FrameVector3D();
      angularMomentumForMPCStuff.setIncludingFrame(estimates.getCentroidalAngularMomentum());
      angularMomentumForMPCStuff.changeFrame(estimates.getCenterOfMassControlZUPFrame());

      YoVector2D desiredVelocityProvider = new YoVector2D(desiredWalkingSpeed, desiredSideWalkingSpeed);

      mpcFootstepCalculator = new AutomaticFootstepMPCMatrixCalculator(swingSide.getOppositeSide(), angularMomentumForMPCStuff, estimates,
                                                                       desiredVelocityProvider, parameters, parentRegistry, graphicsListRegistry);

      desiredMPCTouchdownPositionRelativeToCoM2D = new YoFramePoint2D("desiredMPCTouchdownPositionRelativeToCoM2D", estimates.getCenterOfMassControlZUPFrame(), registry);
      desiredMPCTouchdownPositionRelativeToCoM = new YoFramePoint3D("desiredMPCTouchdownPositionRelativeToCoM", estimates.getCenterOfMassControlZUPFrame(), registry);
      desiredMPCTouchdownPositionRelativeToStanceFoot = new YoFramePoint3D("desiredMPCTouchdownPositionRelativeToStanceFoot", controllerRobot.getFootFrame(swingSide.getOppositeSide()), registry);

      capturePointTimeConstantX = new YoDouble("capturePointTimeConstantX", registry);
      capturePointTimeConstantY = new YoDouble("capturePointTimeConstantY", registry);
      capturePointPole = new YoDouble("capturePointPole", registry);
      capturePointPole.set(0.375);

      predictedALIPVelocityAtTouchdown = new YoFrameVector2D("predictedALIPVelocityAtTouchdown" + swingSide, ReferenceFrame.getWorldFrame(), registry);

      //TODO we need more
      capturePointPole.addListener(value -> computeCapturePointTimeConstant());

      parentRegistry.addChild(registry);
   }

   public void update(double timeInState, YoFramePoint3D pendulumBase3D, boolean leavingDoubleSupport)
   {
      double timeToReachGoal = parameters.getSwingDuration().getDoubleValue() - timeInState;

      //TODO Garbage
      FramePoint2D pendulumBase = new FramePoint2D(estimates.getCenterOfMassControlZUPFrame());
      pendulumBase.setMatchingFrame(pendulumBase3D.getReferenceFrame(), pendulumBase3D.getX(), pendulumBase3D.getY());


      computePredictedALIPVelocityAtTouchdown(timeToReachGoal, pendulumBase, predictedALIPVelocityAtTouchdown);

      mpcFootstepCalculator.computeTwoStepMPCTouchdown(estimates, timeToReachGoal,
                                                       pendulumBase, swingSide.getOppositeSide(),
                                                       capturePointTimeConstantX, capturePointTimeConstantY,
                                                       predictedALIPVelocityAtTouchdown, desiredMPCTouchdownPositionRelativeToCoM2D,
                                                       leavingDoubleSupport);

      desiredMPCTouchdownPositionRelativeToCoM.set(desiredMPCTouchdownPositionRelativeToCoM2D, 0.0);
      desiredMPCTouchdownPositionRelativeToStanceFoot.setMatchingFrame(desiredMPCTouchdownPositionRelativeToCoM);
   }

   private void computeCapturePointTimeConstant()
   {
      YoDouble omegaX = parameters.getOmegaX();
      YoDouble omegaY = parameters.getOmegaY();
      YoDouble swingDuration = parameters.getSwingDuration();

      double omegaTX = omegaX.getDoubleValue() * swingDuration.getValue();
      double omegaTY = omegaY.getDoubleValue() * swingDuration.getValue();

      capturePointTimeConstantX.set((Math.cosh(omegaTX) - capturePointPole.getValue()) / (omegaX.getDoubleValue() * Math.sinh(omegaTX)));
      capturePointTimeConstantY.set((Math.cosh(omegaTY) - capturePointPole.getValue()) / (omegaY.getDoubleValue() * Math.sinh(omegaTY)));
   }

   private void computePredictedALIPVelocityAtTouchdown(double timeToReachGoal, FramePoint2DReadOnly pendulumBase, YoFrameVector2D predictedALIPVelocityAtTouchdown)
   {
      YoDouble omegaX = parameters.getOmegaX();
      YoDouble omegaY = parameters.getOmegaY();
      YoFrameVector3D angularMomentum = estimates.getCentroidalAngularMomentum();

      //TODO Garbage
      FrameVector2D tempVector = new FrameVector2D();

      tempVector.setIncludingFrame(estimates.getCenterOfMassVelocity());
      tempVector.changeFrameAndProjectToXYPlane(estimates.getCenterOfMassControlZUPFrame());

      if (pendulumBase != null && Double.isFinite(timeToReachGoal))
      {
         double omegaTX = omegaX.getDoubleValue() * timeToReachGoal;
         double omegaTY = omegaY.getDoubleValue() * timeToReachGoal;
         double coshX = Math.cosh(omegaTX);
         double coshY = Math.cosh(omegaTY);
         double heightX = estimates.getGravity() / (omegaX.getDoubleValue() * omegaX.getDoubleValue());
         double heightY = estimates.getGravity() / (omegaY.getDoubleValue() * omegaY.getDoubleValue());
         double mhX = estimates.getTotalMass() * heightX;
         double mhY = estimates.getTotalMass() * heightY;

         predictedALIPVelocityAtTouchdown.set(coshX * tempVector.getX(), coshY * tempVector.getY());
         predictedALIPVelocityAtTouchdown.add(coshX / mhX * angularMomentum.getY(), -coshY / mhY * angularMomentum.getX());
         predictedALIPVelocityAtTouchdown.add(-omegaX.getDoubleValue() * Math.sinh(omegaTX) * pendulumBase.getX(),
                                              -omegaY.getDoubleValue() * Math.sinh(omegaTY) * pendulumBase.getY());
      }
      else
      {
         double heightX = estimates.getGravity() / (omegaX.getDoubleValue() * omegaX.getDoubleValue());
         double heightY = estimates.getGravity() / (omegaY.getDoubleValue() * omegaY.getDoubleValue());
         double mhX = estimates.getTotalMass() * heightX;
         double mhY = estimates.getTotalMass() * heightY;

         predictedALIPVelocityAtTouchdown.set(tempVector);
         predictedALIPVelocityAtTouchdown.addX(1.0 / mhX * angularMomentum.getY());
         predictedALIPVelocityAtTouchdown.addY(-1.0 / mhY * angularMomentum.getX());
      }
   }

   public double getDesiredTouchdownPositionX()
   {
      return desiredMPCTouchdownPositionRelativeToCoM.getX();
   }

   public double getDesiredTouchdownPositionY()
   {
      return desiredMPCTouchdownPositionRelativeToCoM.getY();
   }
}
