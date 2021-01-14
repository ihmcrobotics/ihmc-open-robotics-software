package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

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
import us.ihmc.robotics.math.trajectories.MultipleWaypointsBlendedPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.PolynomialEstimator3D;
import us.ihmc.robotics.math.trajectories.Trajectory3DReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
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

   private List<? extends Trajectory3DReadOnly> predictedCoMTrajectory;
   private List<? extends Trajectory3DReadOnly> predictedSecondPotatoTrajectory;
   private List<? extends Trajectory3DReadOnly> predictedThirdPotatoTrajectory;

   private final FrameVector3D totalAngularMomentum = new FrameVector3D();
   private final FrameVector3D totalTorque = new FrameVector3D();
   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final FrameVector3D torque = new FrameVector3D();

   private final FrameVector3D relativePotatoPosition = new FrameVector3D();
   private final FrameVector3D relativePotatoVelocity = new FrameVector3D();
   private final FrameVector3D relativePotatoAcceleration = new FrameVector3D();

   private final RecyclingArrayList<PolynomialEstimator3D> angularMomentumEstimators = new RecyclingArrayList<>(PolynomialEstimator3D::new);

   private final FootTrajectoryPredictor footTrajectoryPredictor = new FootTrajectoryPredictor();

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
      int segment = getSegmentNumber(time, angularMomentumEstimators);
      double localTime = getTimeInSegment(segment, time, angularMomentumEstimators);
      Trajectory3DReadOnly angularMomentumTrajectory = angularMomentumEstimators.get(segment);
      angularMomentumTrajectory.compute(localTime);

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
      int activeCoMIdx = getSegmentNumber(time, predictedCoMTrajectory);
      int activeSecondPotatoIdx = getSegmentNumber(time, predictedSecondPotatoTrajectory);
      int activeThirdPotatoIdx = getSegmentNumber(time, predictedThirdPotatoTrajectory);
      if (activeCoMIdx < 0 || (activeSecondPotatoIdx < 0 && activeThirdPotatoIdx < 0))
         return;

      double comTime = getTimeInSegment(activeCoMIdx, time, predictedCoMTrajectory);
      double secondPotatoTime = getTimeInSegment(activeSecondPotatoIdx, time, predictedSecondPotatoTrajectory);
      double thirdPotatoTime = getTimeInSegment(activeThirdPotatoIdx, time, predictedThirdPotatoTrajectory);

      predictedCoMTrajectory.get(activeCoMIdx).compute(comTime);
      predictedSecondPotatoTrajectory.get(activeSecondPotatoIdx).compute(secondPotatoTime);
      predictedThirdPotatoTrajectory.get(activeThirdPotatoIdx).compute(thirdPotatoTime);

      computeAngularMomentumAtInstant(predictedCoMTrajectory.get(activeCoMIdx), predictedSecondPotatoTrajectory.get(activeSecondPotatoIdx), potatoMass.getDoubleValue(), angularMomentum, torque);
      totalAngularMomentum.add(angularMomentum);
      totalTorque.add(torque);
      computeAngularMomentumAtInstant(predictedCoMTrajectory.get(activeCoMIdx), predictedThirdPotatoTrajectory.get(activeThirdPotatoIdx), potatoMass.getDoubleValue(), angularMomentum, torque);
      totalAngularMomentum.add(angularMomentum);
      totalTorque.add(torque);

      predictedSecondPotatoPosition.set(predictedSecondPotatoTrajectory.get(activeSecondPotatoIdx).getPosition());
      predictedSecondPotatoVelocity.set(predictedSecondPotatoTrajectory.get(activeSecondPotatoIdx).getVelocity());
      predictedThirdPotatoPosition.set(predictedThirdPotatoTrajectory.get(activeThirdPotatoIdx).getPosition());
      predictedThirdPotatoVelocity.set(predictedThirdPotatoTrajectory.get(activeThirdPotatoIdx).getVelocity());

      predictedAngularMomentum.set(totalAngularMomentum);
      predictedAngularMomentumRate.set(totalTorque);
   }

   private final FramePoint3D potatoPosition = new FramePoint3D();
   private final FrameVector3D potatoVelocity = new FrameVector3D();

   public void computeAngularMomentumTrajectories(List<? extends TimeIntervalProvider> timeIntervals, List<? extends Trajectory3DReadOnly> comTrajectories)
   {
      computeAngularMomentumTrajectories(timeIntervals,
                                         comTrajectories,
                                         footTrajectoryPredictor.getPredictedLeftFootTrajectories(),
                                         footTrajectoryPredictor.getPredictedRightFootTrajectories());
   }

   public void computeAngularMomentumTrajectories(List<? extends TimeIntervalProvider> timeIntervals,
                                                  List<? extends Trajectory3DReadOnly> comTrajectories,
                                                  List<? extends Trajectory3DReadOnly> secondPotatoTrajectories,
                                                  List<? extends Trajectory3DReadOnly> thirdPotatoTrajectories)
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

         angularMomentumTrajectory.setInterval(0.0, duration);

         double segmentDt = Math.min(duration / 5, estimationDt);
         double globalStartTime = timeInterval.getStartTime();

         for (double time = 0.0; time <= duration; time += segmentDt)
         {
            double globalTime = time + globalStartTime;
            int activeCoMIdx = getSegmentNumber(globalTime, comTrajectories);
            int activeSecondPotatoIdx = getSegmentNumber(globalTime, secondPotatoTrajectories);
            int activeThirdPotatoIdx = getSegmentNumber(globalTime, thirdPotatoTrajectories);

            if (activeSecondPotatoIdx < 0 && activeThirdPotatoIdx < 0 || activeCoMIdx < 0)
               break;

            Trajectory3DReadOnly comTrajectory = comTrajectories.get(activeCoMIdx);
            comTrajectory.compute(time);
            totalAngularMomentum.setToZero();
            totalTorque.setToZero();

            if (activeSecondPotatoIdx > -1)
            {
               Trajectory3DReadOnly secondPotatoTrajectory = secondPotatoTrajectories.get(activeSecondPotatoIdx);
               double timeInSegment = getTimeInSegment(activeSecondPotatoIdx, globalTime, secondPotatoTrajectories);
               secondPotatoTrajectory.compute(timeInSegment);

               computeAngularMomentumAtInstant(comTrajectory, secondPotatoTrajectory, potatoMass.getDoubleValue(), angularMomentum, torque);
               totalAngularMomentum.add(angularMomentum);

               totalTorque.add(torque);
            }

            if (activeThirdPotatoIdx > -1)
            {
               Trajectory3DReadOnly thirdPotatoTrajectory = thirdPotatoTrajectories.get(activeThirdPotatoIdx);
               double timeInSegment = getTimeInSegment(activeThirdPotatoIdx, globalTime, thirdPotatoTrajectories);
               thirdPotatoTrajectory.compute(timeInSegment);

               computeAngularMomentumAtInstant(comTrajectory, thirdPotatoTrajectory, potatoMass.getDoubleValue(), angularMomentum, torque);
               totalAngularMomentum.add(angularMomentum);

               totalTorque.add(torque);
            }

            angularMomentumTrajectory.addObjectivePosition(time, totalAngularMomentum);
         }

         angularMomentumTrajectory.solve();
      }

      visualize(comTrajectories, secondPotatoTrajectories, thirdPotatoTrajectories);
   }

   private void visualize(List<? extends Trajectory3DReadOnly> comTrajectories,
                          List<? extends Trajectory3DReadOnly> secondPotatoTrajectories,
                          List<? extends Trajectory3DReadOnly> thirdPotatoTrajectories)
   {
      if (!visualize)
         return;

      double totalCoMDuration = 0.0;
      double total2Duration = 0.0;
      double total3Duration = 0.0;
      for (int i = 0; i < comTrajectories.size(); i++)
         totalCoMDuration += comTrajectories.get(i).getDuration();

      for (int i = 0; i < secondPotatoTrajectories.size(); i++)
         total2Duration += secondPotatoTrajectories.get(i).getDuration();

      for (int i = 0; i < thirdPotatoTrajectories.size(); i++)
         total3Duration += thirdPotatoTrajectories.get(i).getDuration();

      double duration = Math.min(totalCoMDuration, Math.min(total2Duration, Math.min(total3Duration, sufficientlyLong)));

      comTrajectoryVis.reset();
      secondPotatoVis.reset();
      thirdPotatoVis.reset();

      for (double time = 0.0; time <= duration; time += estimationDt)
      {
         int segment = getSegmentNumber(time, comTrajectories);
         double localtime = getTimeInSegment(segment, time, comTrajectories);
         comTrajectories.get(segment).compute(localtime);
         comTrajectoryVis.setBall(comTrajectories.get(segment).getPosition());
      }

      for (double time = 0.0; time <= duration; time += estimationDt)
      {
         int segment = getSegmentNumber(time, secondPotatoTrajectories);
         double localtime = getTimeInSegment(segment, time, secondPotatoTrajectories);
         secondPotatoTrajectories.get(segment).compute(localtime);
         secondPotatoVis.setBall(secondPotatoTrajectories.get(segment).getPosition());
      }

      for (double time = 0.0; time <= duration; time += estimationDt)
      {
         int segment = getSegmentNumber(time, thirdPotatoTrajectories);
         double localtime = getTimeInSegment(segment, time, thirdPotatoTrajectories);
         thirdPotatoTrajectories.get(segment).compute(localtime);
         thirdPotatoVis.setBall(thirdPotatoTrajectories.get(segment).getPosition());
      }
   }

   public List<? extends Trajectory3DReadOnly> getAngularMomentumTrajectories()
   {
      return angularMomentumEstimators;
   }

   private void computeAngularMomentumAtInstant(Trajectory3DReadOnly comTrajectory,
                                                Trajectory3DReadOnly potatoTrajectory,
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

   private void computeAngularMomentumAtInstant(Trajectory3DReadOnly comTrajectory,
                                                Trajectory3DReadOnly potatoTrajectory,
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
