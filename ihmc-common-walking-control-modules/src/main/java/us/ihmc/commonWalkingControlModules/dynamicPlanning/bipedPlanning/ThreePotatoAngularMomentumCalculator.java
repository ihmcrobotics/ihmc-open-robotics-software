package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import javafx.geometry.Pos;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.trajectories.PolynomialEstimator3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.FramePolynomial3DBasics;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.robotics.math.trajectories.interfaces.PositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeIntervalProvider;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class ThreePotatoAngularMomentumCalculator
{
   private static final double estimationDt = 0.01;
   private static final double sufficientlyLong = 10.0;

   private static final boolean visualize = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble potatoMass = new YoDouble("PotatoMass", registry);
   private final YoFrameVector3D predictedFitAngularMomentum = new YoFrameVector3D("predictedFitAngularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D predictedFitAngularMomentumRate = new YoFrameVector3D("predictedFitAngularMomentumRate",
                                                                                       ReferenceFrame.getWorldFrame(),
                                                                                       registry);
   private final YoFrameVector3D predictedAngularMomentum = new YoFrameVector3D("predictedAngularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D predictedAngularMomentumRate = new YoFrameVector3D("predictedAngularMomentumRate", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D predictedSecondPotatoPosition = new YoFramePoint3D("predictedSecondPotatoPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D predictedSecondPotatoVelocity = new YoFrameVector3D("predictedSecondPotatoVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D predictedThirdPotatoPosition = new YoFramePoint3D("predictedThirdPotatoPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D predictedThirdPotatoVelocity = new YoFrameVector3D("predictedThirdPotatoVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector3D actualPotatoModelMomentum = new YoFrameVector3D("actualPotatoModelMomentum", ReferenceFrame.getWorldFrame(), registry);

   private MultipleSegmentPositionTrajectoryGenerator<?> predictedCoMTrajectory;
   private MultipleSegmentPositionTrajectoryGenerator<?> predictedSecondPotatoTrajectory;
   private MultipleSegmentPositionTrajectoryGenerator<?> predictedThirdPotatoTrajectory;

   private final FrameVector3D totalAngularMomentum = new FrameVector3D();
   private final FrameVector3D totalTorque = new FrameVector3D();
   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final FrameVector3D torque = new FrameVector3D();

   private final FrameVector3D relativePotatoPosition = new FrameVector3D();
   private final FrameVector3D relativePotatoVelocity = new FrameVector3D();
   private final FrameVector3D relativePotatoAcceleration = new FrameVector3D();

   private final RecyclingArrayList<PolynomialEstimator3D> angularMomentumEstimators = new RecyclingArrayList<>(PolynomialEstimator3D::new);
   private final MultipleSegmentPositionTrajectoryGenerator<FramePolynomial3DBasics> angularMomentumTrajectory;

   private final FootTrajectoryPredictor footTrajectoryPredictor = new FootTrajectoryPredictor(registry);

   private final BagOfBalls comTrajectoryVis;
   private final BagOfBalls secondPotatoVis;
   private final BagOfBalls thirdPotatoVis;

   private final CenterOfMassJacobian centerOfMassJacobian;
   private final SideDependentList<MovingReferenceFrame> soleFrames;

   public ThreePotatoAngularMomentumCalculator(double potatoMass,
                                               YoRegistry parentRegistry,
                                               CenterOfMassJacobian centerOfMassJacobian,
                                               SideDependentList<MovingReferenceFrame> soleFrames,
                                               YoGraphicsListRegistry graphicsListRegistry)
   {
      this.centerOfMassJacobian = centerOfMassJacobian;
      this.soleFrames = soleFrames;
      this.potatoMass.set(potatoMass);

     angularMomentumTrajectory = new MultipleSegmentPositionTrajectoryGenerator<>("angularMomentum", 5, ReferenceFrame.getWorldFrame(), registry);

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

   public void setSwingTrajectory(MultipleWaypointsPoseTrajectoryGenerator swingTrajectory)
   {
      footTrajectoryPredictor.setSwingTrajectory(swingTrajectory);
   }

   public void predictFootTrajectories(CoPTrajectoryGeneratorState state)
   {
      footTrajectoryPredictor.compute(state);
   }

   public void computeAngularMomentum(double time)
   {
      angularMomentumTrajectory.compute(time);

      predictedFitAngularMomentum.set(angularMomentumTrajectory.getPosition());
      predictedFitAngularMomentumRate.set(angularMomentumTrajectory.getVelocity());

      totalAngularMomentum.setToZero();
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
      actualPotatoModelMomentum.set(totalAngularMomentum);

      totalAngularMomentum.setToZero();
      totalTorque.setToZero();
      if (time > predictedCoMTrajectory.getEndTime() || time > predictedSecondPotatoTrajectory.getEndTime() || time > predictedThirdPotatoTrajectory.getEndTime())
         return;


      predictedCoMTrajectory.compute(time);
      predictedSecondPotatoTrajectory.compute(time);
      predictedThirdPotatoTrajectory.compute(time);

      computeAngularMomentumAtInstant(predictedCoMTrajectory, predictedSecondPotatoTrajectory, potatoMass.getDoubleValue(), angularMomentum, torque);
      totalAngularMomentum.add(angularMomentum);
      totalTorque.add(torque);
      computeAngularMomentumAtInstant(predictedCoMTrajectory, predictedThirdPotatoTrajectory, potatoMass.getDoubleValue(), angularMomentum, torque);
      totalAngularMomentum.add(angularMomentum);
      totalTorque.add(torque);

      predictedSecondPotatoPosition.set(predictedSecondPotatoTrajectory.getPosition());
      predictedSecondPotatoVelocity.set(predictedSecondPotatoTrajectory.getVelocity());
      predictedThirdPotatoPosition.set(predictedThirdPotatoTrajectory.getPosition());
      predictedThirdPotatoVelocity.set(predictedThirdPotatoTrajectory.getVelocity());

      predictedAngularMomentum.set(totalAngularMomentum);
      predictedAngularMomentumRate.set(totalTorque);
   }

   private final FramePoint3D potatoPosition = new FramePoint3D();
   private final FrameVector3D potatoVelocity = new FrameVector3D();

   public void computeAngularMomentumTrajectories(List<? extends TimeIntervalProvider> timeIntervals, MultipleSegmentPositionTrajectoryGenerator<?> comTrajectories)
   {
      computeAngularMomentumTrajectories(timeIntervals,
                                         comTrajectories,
                                         footTrajectoryPredictor.getPredictedLeftFootTrajectories(),
                                         footTrajectoryPredictor.getPredictedRightFootTrajectories());
   }

   public void computeAngularMomentumTrajectories(List<? extends TimeIntervalProvider> timeIntervals,
                                                  MultipleSegmentPositionTrajectoryGenerator<?> comTrajectories,
                                                  MultipleSegmentPositionTrajectoryGenerator<?> secondPotatoTrajectories,
                                                  MultipleSegmentPositionTrajectoryGenerator<?> thirdPotatoTrajectories)
   {
      this.predictedCoMTrajectory = comTrajectories;
      this.predictedSecondPotatoTrajectory = secondPotatoTrajectories;
      this.predictedThirdPotatoTrajectory = thirdPotatoTrajectories;

      angularMomentumEstimators.clear();

      for (int i = 0; i < timeIntervals.size(); i++)
      {
         TimeIntervalReadOnly timeInterval = timeIntervals.get(i).getTimeInterval();
         PolynomialEstimator3D angularMomentumTrajectory = angularMomentumEstimators.add();
         angularMomentumTrajectory.reset();
         angularMomentumTrajectory.reshape(5);

         double duration = Math.min(timeInterval.getDuration(), sufficientlyLong);

         angularMomentumTrajectory.getTimeInterval().setInterval(0.0, duration);

         double segmentDt = Math.min(duration / 5, estimationDt);
         double globalStartTime = timeInterval.getStartTime();

         for (double time = 0.0; time <= duration; time += segmentDt)
         {
            double globalTime = time + globalStartTime;

            if (globalTime > secondPotatoTrajectories.getEndTime() && globalTime > thirdPotatoTrajectories.getEndTime() || globalStartTime > comTrajectories.getEndTime())
               break;

            comTrajectories.compute(time);
            totalAngularMomentum.setToZero();
            totalTorque.setToZero();

            if (globalTime > secondPotatoTrajectories.getEndTime())
            {
               secondPotatoTrajectories.compute(globalTime);
               computeAngularMomentumAtInstant(comTrajectories, secondPotatoTrajectories, potatoMass.getDoubleValue(), angularMomentum, torque);
               totalAngularMomentum.add(angularMomentum);

               totalTorque.add(torque);
            }

            if (globalTime > thirdPotatoTrajectories.getEndTime())
            {
               thirdPotatoTrajectories.compute(globalTime);
               computeAngularMomentumAtInstant(comTrajectories, thirdPotatoTrajectories, potatoMass.getDoubleValue(), angularMomentum, torque);
               totalAngularMomentum.add(angularMomentum);

               totalTorque.add(torque);
            }

            angularMomentumTrajectory.addObjectivePosition(time, totalAngularMomentum);
         }

         angularMomentumTrajectory.initialize();
      }

      visualize(comTrajectories, secondPotatoTrajectories, thirdPotatoTrajectories);
   }

   private void visualize(MultipleSegmentPositionTrajectoryGenerator<?> comTrajectories,
                          MultipleSegmentPositionTrajectoryGenerator<?> secondPotatoTrajectories,
                          MultipleSegmentPositionTrajectoryGenerator<?> thirdPotatoTrajectories)
   {
      if (!visualize)
         return;

      double duration = Math.min(comTrajectories.getEndTime(), Math.min(secondPotatoTrajectories.getEndTime(), Math.min(thirdPotatoTrajectories.getEndTime(), sufficientlyLong)));

      comTrajectoryVis.reset();
      secondPotatoVis.reset();
      thirdPotatoVis.reset();

      for (double time = 0.0; time <= duration; time += estimationDt)
      {
         comTrajectories.compute(time);
         secondPotatoTrajectories.compute(time);
         thirdPotatoTrajectories.compute(time);

         comTrajectoryVis.setBall(comTrajectories.getPosition());
         secondPotatoVis.setBall(secondPotatoTrajectories.getPosition());
         thirdPotatoVis.setBall(thirdPotatoTrajectories.getPosition());
      }
   }

   public MultipleSegmentPositionTrajectoryGenerator<?> getAngularMomentumTrajectories()
   {
      return angularMomentumTrajectory;
   }

   private void computeAngularMomentumAtInstant(PositionTrajectoryGenerator comTrajectory,
                                                PositionTrajectoryGenerator potatoTrajectory,
                                                double potatoMass,
                                                Vector3DBasics angularMomentumToPack,
                                                Vector3DBasics torqueToPack)
   {
      computeAngularMomentumAtInstant(comTrajectory, potatoTrajectory, potatoMass, angularMomentumToPack);

      relativePotatoPosition.sub(potatoTrajectory.getPosition(), comTrajectory.getPosition());
      relativePotatoAcceleration.sub(potatoTrajectory.getAcceleration(), comTrajectory.getAcceleration());

      torqueToPack.cross(relativePotatoPosition, relativePotatoAcceleration);
      torqueToPack.scale(potatoMass);
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

   public int getSegmentNumber(double time, List<? extends TimeIntervalReadOnly> segments)
   {
      double startTime = 0.0;
      for (int i = 0; i < segments.size(); i++)
      {
         if (segments.get(i).intervalContains(time - startTime))
            return i;

         startTime += segments.get(i).getDuration();
      }

      return -1;
   }

   public double getTimeInSegment(int segmentNumber, double time, List<? extends TimeIntervalReadOnly> segments)
   {
      for (int i = 0; i < segmentNumber; i++)
         time -= segments.get(i).getDuration();

      return time;
   }
}
