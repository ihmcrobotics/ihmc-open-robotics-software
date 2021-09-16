package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingCoPTrajectoryGeneratorState;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.trajectories.FixedFramePolynomialEstimator3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.PositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.robotics.time.TimeIntervalProvider;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLongTime;

public class ThreePotatoAngularMomentumCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean debug = false;
   private static final boolean visualize = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble potatoMass = new YoDouble("PotatoMass", registry);
   private final DoubleProvider potatoMassFraction;
   private final double totalMass;

   private final BooleanProvider useHeightScaledAngularMomentum = new BooleanParameter("useHeightScaledAngularMomentum", registry, true);
   private final DoubleProvider idealAngularMomentumSampleDt = new DoubleParameter("idealAngularMomentumSampleDt", registry, 0.05);
   private final IntegerProvider maxAngularMomentumSamplesPerSegment = new IntegerParameter("maxAngularMomentumSamplesPerSegment", registry, 8);
   private final IntegerProvider minAngularMomentumSamplesPerSegment = new IntegerParameter("minAngularMomentumSamplesPerSegment", registry, 5);

   private final YoFrameVector3D desiredAngularMomentum = new YoFrameVector3D("desiredAngularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D desiredAngularMomentumRate = new YoFrameVector3D("desiredAngularMomentumRate", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector3D desiredScaledAngularMomentum = new YoFrameVector3D("desiredScaledAngularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D desiredScaledAngularMomentumRate = new YoFrameVector3D("desiredScaledAngularMomentumRate", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector3D predictedAngularMomentum = new YoFrameVector3D("predictedAngularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D predictedAngularMomentumRate = new YoFrameVector3D("predictedAngularMomentumRate", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector3D predictedLinearMomentum = new YoFrameVector3D("predictedLinearMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D predictedLinearMomentumRate = new YoFrameVector3D("predictedLinearMomentumRate", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector3D actualModelAngularMomentum = new YoFrameVector3D("actualModelAngularMomentum", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint3D predictedCoMPosition = new YoFramePoint3D("predictedCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D predictedCoMVelocity = new YoFrameVector3D("predictedCoMVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D predictedLeftFootPosition = new YoFramePoint3D("predictedLeftFootPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D predictedLeftFootVelocity = new YoFrameVector3D("predictedLeftFootVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D predictedRightFootPosition = new YoFramePoint3D("predictedRightFootPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D predictedRightFootVelocity = new YoFrameVector3D("predictedRightFootVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final YoDouble comTrajectoryCurrentTime = new YoDouble("firstPotatoCurrentTrajectoryTime", registry);
   private final YoInteger comTrajectoryCurrentSegment = new YoInteger("firstPotatoCurrentSegmentIndex", registry);

   private MultipleSegmentPositionTrajectoryGenerator<?> predictedCoMTrajectory;

   private final FrameVector3D totalAngularMomentum = new FrameVector3D();
   private final FrameVector3D totalTorque = new FrameVector3D();
   private final FrameVector3D totalLinearMomentum = new FrameVector3D();
   private final FrameVector3D totalForce = new FrameVector3D();
   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final FrameVector3D linearMomentum = new FrameVector3D();
   private final FrameVector3D torque = new FrameVector3D();
   private final FrameVector3D force = new FrameVector3D();

   private final FrameVector3D relativePotatoPosition = new FrameVector3D();
   private final FrameVector3D relativePotatoVelocity = new FrameVector3D();
   private final FrameVector3D relativePotatoAcceleration = new FrameVector3D();

   private final FixedFramePolynomialEstimator3D angularMomentumEstimator = new FixedFramePolynomialEstimator3D(worldFrame);
   private final FixedFramePolynomialEstimator3D scaledAngularMomentumEstimator = new FixedFramePolynomialEstimator3D(worldFrame);
   private final MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomialEstimator3D> angularMomentumTrajectory;
   private final MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomialEstimator3D> heightScaledAngularMomentumTrajectory;

   private final FootTrajectoryPredictor footTrajectoryPredictor = new FootTrajectoryPredictor(registry);

   private final BagOfBalls comTrajectoryVis;
   private final BagOfBalls secondPotatoVis;
   private final BagOfBalls thirdPotatoVis;

   private final CenterOfMassJacobian centerOfMassJacobian;
   private final SideDependentList<MovingReferenceFrame> soleFrames;

   private final ExecutionTimer angularMomentumEstimationTimer = new ExecutionTimer("angularMomentumEstimation", registry);
   private final ExecutionTimer angularMomentumPredictionTimer = new ExecutionTimer("angularMomentumPrediction", registry);

   private final double gravityZ;

   public ThreePotatoAngularMomentumCalculator(double totalMass,
                                               DoubleProvider potatoMassFraction,
                                               double gravityZ,
                                               CenterOfMassJacobian centerOfMassJacobian,
                                               SideDependentList<MovingReferenceFrame> soleFrames,
                                               YoRegistry parentRegistry,
                                               YoGraphicsListRegistry graphicsListRegistry)
   {
      this.gravityZ = Math.abs(gravityZ);
      this.centerOfMassJacobian = centerOfMassJacobian;
      this.soleFrames = soleFrames;
      this.totalMass = totalMass;
      this.potatoMassFraction = potatoMassFraction;

      angularMomentumTrajectory = new MultipleSegmentPositionTrajectoryGenerator<>("angularMomentum",
                                                                                   50,
                                                                                   worldFrame,
                                                                                   () -> new FixedFramePolynomialEstimator3D(worldFrame),
                                                                                   registry);
      heightScaledAngularMomentumTrajectory = new MultipleSegmentPositionTrajectoryGenerator<>("heightScaledAngularMomentum",
                                                                                               50,
                                                                                               worldFrame,
                                                                                               () -> new FixedFramePolynomialEstimator3D(worldFrame),
                                                                                               registry);

      if (visualize)
      {
         double size = 0.01;
         comTrajectoryVis = new BagOfBalls(100, size, "comTrajectoryVis", YoAppearance.Black(), registry, graphicsListRegistry);
         secondPotatoVis = new BagOfBalls(100, size, "secondPotatoVis", YoAppearance.Blue(), registry, graphicsListRegistry);
         thirdPotatoVis = new BagOfBalls(100, size, "thirdPotatoVis", YoAppearance.Red(), registry, graphicsListRegistry);
      }
      else
      {
         comTrajectoryVis = null;
         secondPotatoVis = null;
         thirdPotatoVis = null;
      }

      parentRegistry.addChild(registry);
   }

   public void setSwingTrajectory(RobotSide swingSide, MultipleWaypointsPoseTrajectoryGenerator swingTrajectory)
   {
      footTrajectoryPredictor.setSwingTrajectory(swingSide, swingTrajectory);
   }

   public void clearSwingTrajectory()
   {
      footTrajectoryPredictor.clearSwingTrajectory();
   }

   public void predictFootTrajectories(CoPTrajectoryGeneratorState state)
   {
      footTrajectoryPredictor.compute(state);
   }

   public void predictFootTrajectories(JumpingCoPTrajectoryGeneratorState state)
   {
      footTrajectoryPredictor.compute(state);
   }

   public void reset()
   {
      predictedCoMTrajectory = null;
      angularMomentumEstimator.reset();
      angularMomentumEstimator.reshape(1);
      totalAngularMomentum.setToZero();
      angularMomentumEstimator.addObjectivePosition(0.0, totalAngularMomentum);
      angularMomentumEstimator.initialize();

      angularMomentumTrajectory.clear();
      heightScaledAngularMomentumTrajectory.clear();

      angularMomentumTrajectory.appendSegment(angularMomentumEstimator);
      heightScaledAngularMomentumTrajectory.appendSegment(angularMomentumEstimator);

      angularMomentumTrajectory.initialize();
      heightScaledAngularMomentumTrajectory.initialize();
   }

   public void computeAngularMomentum(double time)
   {
      angularMomentumPredictionTimer.startMeasurement();

      angularMomentumTrajectory.compute(time);
      heightScaledAngularMomentumTrajectory.compute(time);

      desiredAngularMomentum.set(angularMomentumTrajectory.getPosition());
      desiredAngularMomentumRate.set(angularMomentumTrajectory.getVelocity());

      desiredScaledAngularMomentum.set(heightScaledAngularMomentumTrajectory.getPosition());
      desiredScaledAngularMomentumRate.set(heightScaledAngularMomentumTrajectory.getVelocity());

      totalAngularMomentum.setToZero();
      if (centerOfMassJacobian != null && soleFrames != null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            FramePoint3DReadOnly comPosition = centerOfMassJacobian.getCenterOfMass();
            FrameVector3DReadOnly comVelocity = centerOfMassJacobian.getCenterOfMassVelocity();

            potatoPosition.setToZero(soleFrames.get(robotSide));
            potatoPosition.changeFrame(ReferenceFrame.getWorldFrame());
            potatoVelocity.setIncludingFrame(soleFrames.get(robotSide).getTwistOfFrame().getLinearPart());
            potatoVelocity.changeFrame(ReferenceFrame.getWorldFrame());

            computeAngularMomentumAtInstant(comPosition, comVelocity, potatoPosition, potatoVelocity, potatoMass.getDoubleValue(), angularMomentum);
            totalAngularMomentum.add(angularMomentum);
         }
         actualModelAngularMomentum.set(totalAngularMomentum);
      }


      MultipleWaypointsPositionTrajectoryGenerator predictedLeftFootTrajectory = footTrajectoryPredictor.getPredictedLeftFootTrajectories();
      MultipleWaypointsPositionTrajectoryGenerator predictedRightFootTrajectory = footTrajectoryPredictor.getPredictedRightFootTrajectories();

      totalAngularMomentum.setToZero();
      totalTorque.setToZero();
      totalLinearMomentum.setToZero();
      totalForce.setToZero();

      if (predictedCoMTrajectory == null)
      {
         angularMomentumPredictionTimer.stopMeasurement();
         return;
      }

      if (time > predictedCoMTrajectory.getEndTime() || time > predictedLeftFootTrajectory.getLastWaypointTime()
          || time > predictedRightFootTrajectory.getLastWaypointTime())
      {
         angularMomentumPredictionTimer.stopMeasurement();
         return;
      }

      predictedCoMTrajectory.compute(time);
      predictedLeftFootTrajectory.compute(time);
      predictedRightFootTrajectory.compute(time);

      computeAngularMomentumAtInstant(predictedCoMTrajectory,
                                      predictedLeftFootTrajectory,
                                      potatoMass.getDoubleValue(),
                                      angularMomentum,
                                      torque,
                                      linearMomentum,
                                      force);
      totalAngularMomentum.add(angularMomentum);
      totalTorque.add(torque);
      totalLinearMomentum.add(linearMomentum);
      totalForce.add(force);

      computeAngularMomentumAtInstant(predictedCoMTrajectory,
                                      predictedRightFootTrajectory,
                                      potatoMass.getDoubleValue(),
                                      angularMomentum,
                                      torque,
                                      linearMomentum,
                                      force);
      totalAngularMomentum.add(angularMomentum);
      totalTorque.add(torque);
      totalLinearMomentum.add(linearMomentum);
      totalForce.add(force);

      predictedCoMPosition.set(predictedCoMTrajectory.getPosition());
      predictedCoMVelocity.set(predictedCoMTrajectory.getVelocity());
      predictedLeftFootPosition.set(predictedLeftFootTrajectory.getPosition());
      predictedLeftFootVelocity.set(predictedLeftFootTrajectory.getVelocity());
      predictedRightFootPosition.set(predictedRightFootTrajectory.getPosition());
      predictedRightFootVelocity.set(predictedRightFootTrajectory.getVelocity());

      comTrajectoryCurrentSegment.set(predictedCoMTrajectory.getCurrentSegmentIndex());
      comTrajectoryCurrentTime.set(predictedCoMTrajectory.getCurrentSegmentTrajectoryTime());

      predictedAngularMomentum.set(totalAngularMomentum);
      predictedAngularMomentumRate.set(totalTorque);

      predictedLinearMomentum.set(totalLinearMomentum);
      predictedLinearMomentumRate.set(totalForce);

      angularMomentumPredictionTimer.stopMeasurement();
   }

   private final FramePoint3D potatoPosition = new FramePoint3D();
   private final FrameVector3D potatoVelocity = new FrameVector3D();


   public void computeAngularMomentumTrajectories(List<? extends TimeIntervalProvider> timeIntervals,
                                                  MultipleSegmentPositionTrajectoryGenerator<?> comTrajectories)
   {
      angularMomentumEstimationTimer.startMeasurement();

      potatoMass.set(potatoMassFraction.getValue() * totalMass);

      this.predictedCoMTrajectory = comTrajectories;
      MultipleWaypointsPositionTrajectoryGenerator predictedLeftFootTrajectory = footTrajectoryPredictor.getPredictedLeftFootTrajectories();
      MultipleWaypointsPositionTrajectoryGenerator predictedRightFootTrajectory = footTrajectoryPredictor.getPredictedRightFootTrajectories();

      angularMomentumTrajectory.clear();
      heightScaledAngularMomentumTrajectory.clear();

      for (int i = 0; i < timeIntervals.size(); i++)
      {
         TimeIntervalReadOnly timeInterval = timeIntervals.get(i).getTimeInterval();
         angularMomentumEstimator.reset();
         angularMomentumEstimator.reshape(5);
         scaledAngularMomentumEstimator.reset();
         scaledAngularMomentumEstimator.reshape(5);

         double duration = Math.min(timeInterval.getDuration(), 10.0);

         angularMomentumEstimator.getTimeInterval().setInterval(timeInterval.getStartTime(), timeInterval.getStartTime() + duration);
         scaledAngularMomentumEstimator.getTimeInterval().setInterval(timeInterval.getStartTime(), timeInterval.getStartTime() + duration);

         double minDt = duration / maxAngularMomentumSamplesPerSegment.getValue();
         double maxDt = duration / minAngularMomentumSamplesPerSegment.getValue();
         double segmentDt = duration / idealAngularMomentumSampleDt.getValue();
         segmentDt = MathTools.clamp(segmentDt, minDt, maxDt);

         for (double timeInInterval = 0.0; timeInInterval <= duration; timeInInterval += segmentDt)
         {
            double time = timeInInterval + timeInterval.getStartTime();

            if (time > predictedLeftFootTrajectory.getLastWaypointTime() && time > predictedRightFootTrajectory.getLastWaypointTime()
                || time > comTrajectories.getEndTime())
            {
               break;
            }

            comTrajectories.compute(time);
            totalAngularMomentum.setToZero();
            totalTorque.setToZero();

            if (time <= predictedLeftFootTrajectory.getLastWaypointTime())
            {
               predictedLeftFootTrajectory.compute(time);
               computeAngularMomentumAtInstant(comTrajectories,
                                               predictedLeftFootTrajectory,
                                               potatoMass.getDoubleValue(),
                                               angularMomentum,
                                               torque,
                                               linearMomentum,
                                               force);
               totalAngularMomentum.add(angularMomentum);
               totalTorque.add(torque);
               totalLinearMomentum.add(linearMomentum);
               totalForce.add(force);
            }

            if (time <= predictedRightFootTrajectory.getLastWaypointTime())
            {
               predictedRightFootTrajectory.compute(time);
               computeAngularMomentumAtInstant(comTrajectories,
                                               predictedRightFootTrajectory,
                                               potatoMass.getDoubleValue(),
                                               angularMomentum,
                                               torque,
                                               linearMomentum,
                                               force);
               totalAngularMomentum.add(angularMomentum);
               totalTorque.add(torque);
               totalLinearMomentum.add(linearMomentum);
               totalForce.add(force);
            }

            if (debug && totalAngularMomentum.containsNaN() || Double.isInfinite(totalAngularMomentum.length()))
               throw new RuntimeException("Error.");
            angularMomentumEstimator.addObjectivePosition(timeInInterval, totalAngularMomentum);

            if (!MathTools.isLessThanOrEqualToWithPrecision(comTrajectories.getAcceleration().getZ(), gravityZ, 1e-3))
               totalAngularMomentum.scale(gravityZ / (gravityZ + comTrajectories.getAcceleration().getZ()));

            scaledAngularMomentumEstimator.addObjectivePosition(timeInInterval, totalAngularMomentum);
            if (debug && totalAngularMomentum.containsNaN() || Double.isInfinite(totalAngularMomentum.length()))
               throw new RuntimeException("Error.");
         }

         angularMomentumEstimator.initialize();
         scaledAngularMomentumEstimator.initialize();

         angularMomentumTrajectory.appendSegment(angularMomentumEstimator);
         heightScaledAngularMomentumTrajectory.appendSegment(scaledAngularMomentumEstimator);
      }

      angularMomentumTrajectory.initialize();
      heightScaledAngularMomentumTrajectory.initialize();

      visualize(comTrajectories, predictedLeftFootTrajectory, predictedRightFootTrajectory);

      angularMomentumEstimationTimer.stopMeasurement();
   }

   private void visualize(MultipleSegmentPositionTrajectoryGenerator<?> comTrajectories,
                          MultipleWaypointsPositionTrajectoryGenerator secondPotatoTrajectories,
                          MultipleWaypointsPositionTrajectoryGenerator thirdPotatoTrajectories)
   {
      if (!visualize)
         return;

      double duration = EuclidCoreTools.min(comTrajectories.getEndTime(), secondPotatoTrajectories.getLastWaypointTime(), thirdPotatoTrajectories.getLastWaypointTime(), sufficientlyLongTime);

      comTrajectoryVis.reset();
      secondPotatoVis.reset();
      thirdPotatoVis.reset();

      for (double time = 0.0; time <= duration; time += idealAngularMomentumSampleDt.getValue())
      {
         comTrajectories.compute(time);
         secondPotatoTrajectories.compute(time);
         thirdPotatoTrajectories.compute(time);

         comTrajectoryVis.setBall(comTrajectories.getPosition());
         secondPotatoVis.setBall(secondPotatoTrajectories.getPosition());
         thirdPotatoVis.setBall(thirdPotatoTrajectories.getPosition());
      }
   }


   public boolean useHeightScaledAngularMomentum()
   {
      return useHeightScaledAngularMomentum.getValue();
   }


   public FrameVector3DReadOnly getDesiredAngularMomentum()
   {
      return desiredAngularMomentum;
   }

   public FrameVector3DReadOnly getDesiredHeightScaledAngularMomentum()
   {
      return desiredScaledAngularMomentum;
   }

   public FrameVector3DReadOnly getDesiredAngularMomentumRate()
   {
      return desiredAngularMomentumRate;
   }

   public FrameVector3DReadOnly getDesiredHeightScaledAngularMomentumRate()
   {
      return desiredScaledAngularMomentumRate;
   }

   public FrameVector3DReadOnly getDesiredDistributedLinearMomentum()
   {
      return predictedLinearMomentum;
   }

   public FrameVector3DReadOnly getDesiredDistributedLinearMomentumRate()
   {
      return predictedLinearMomentumRate;
   }

   public MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomialEstimator3D> getAngularMomentumTrajectories()
   {
      return angularMomentumTrajectory;
   }

   public MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomialEstimator3D> getHeightScaledAngularMomentumTrajectories()
   {
      return heightScaledAngularMomentumTrajectory;
   }

   private void computeAngularMomentumAtInstant(PositionTrajectoryGenerator comTrajectory,
                                                PositionTrajectoryGenerator potatoTrajectory,
                                                double potatoMass,
                                                Vector3DBasics angularMomentumToPack,
                                                Vector3DBasics torqueToPack,
                                                Vector3DBasics linearMomentumToPack,
                                                Vector3DBasics forceToPack)
   {
      computeAngularMomentumAtInstant(comTrajectory, potatoTrajectory, potatoMass, angularMomentumToPack);

      relativePotatoPosition.sub(potatoTrajectory.getPosition(), comTrajectory.getPosition());
      relativePotatoAcceleration.sub(potatoTrajectory.getAcceleration(), comTrajectory.getAcceleration());

      torqueToPack.cross(relativePotatoPosition, relativePotatoAcceleration);
      torqueToPack.scale(potatoMass);

      linearMomentumToPack.setAndScale(potatoMass, potatoTrajectory.getVelocity());
      forceToPack.setAndScale(potatoMass, potatoTrajectory.getAcceleration());

      if (debug && torqueToPack.containsNaN() || Double.isInfinite(totalAngularMomentum.length()))
         throw new RuntimeException("Error.");
   }

   private void computeAngularMomentumAtInstant(PositionTrajectoryGenerator comTrajectory,
                                                PositionTrajectoryGenerator potatoTrajectory,
                                                double potatoMass,
                                                Vector3DBasics angularMomentumToPack)
   {
      computeAngularMomentumAtInstant(comTrajectory.getPosition(),
                                      comTrajectory.getVelocity(),
                                      potatoTrajectory.getPosition(),
                                      potatoTrajectory.getVelocity(),
                                      potatoMass,
                                      angularMomentumToPack);
   }

   private void computeAngularMomentumAtInstant(Point3DReadOnly centerOfMassPosition,
                                                Vector3DReadOnly centerOfMassVelocity,
                                                Point3DReadOnly potatoPosition,
                                                Vector3DReadOnly potatoVelocity,
                                                double potatoMass,
                                                Vector3DBasics angularMomentumToPack)
   {
      relativePotatoPosition.sub(potatoPosition, centerOfMassPosition);
      relativePotatoVelocity.sub(potatoVelocity, centerOfMassVelocity);

      angularMomentumToPack.cross(relativePotatoPosition, relativePotatoVelocity);
      angularMomentumToPack.scale(potatoMass);
   }
}
