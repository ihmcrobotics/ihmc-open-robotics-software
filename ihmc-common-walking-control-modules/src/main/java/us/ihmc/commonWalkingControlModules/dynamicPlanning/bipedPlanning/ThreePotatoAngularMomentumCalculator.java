package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.trajectories.PolynomialEstimator3D;
import us.ihmc.robotics.math.trajectories.Trajectory3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class ThreePotatoAngularMomentumCalculator
{
   private static final double estimationDt = 0.05;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble potatoMass = new YoDouble("PotatoMass", registry);
   private final YoFrameVector3D predictedAngularMomentum = new YoFrameVector3D("predictedAngularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D predictedAngularMomentumRate = new YoFrameVector3D("predictedAngularMomentumRate", ReferenceFrame.getWorldFrame(), registry);

   private final FrameVector3D totalAngularMomentum = new FrameVector3D();
   private final FrameVector3D angularMomentum = new FrameVector3D();

   private final FrameVector3D relativePotatoPosition = new FrameVector3D();
   private final FrameVector3D relativePotatoVelocity = new FrameVector3D();

   private final RecyclingArrayList<PolynomialEstimator3D> angularMomentumEstimators = new RecyclingArrayList<>(PolynomialEstimator3D::new);

   private final FootTrajectoryPredictor footTrajectoryPredictor = new FootTrajectoryPredictor();

   public ThreePotatoAngularMomentumCalculator(double potatoMass, YoRegistry parentRegistry)
   {
      this.potatoMass.set(potatoMass);

      parentRegistry.addChild(registry);
   }

   public void predictFootTrajectories(CoPTrajectoryGeneratorState state)
   {
      footTrajectoryPredictor.compute(state);
   }

   public void computeAngularMomentum(double time)
   {
      int segment = getSegmentNumber(time, angularMomentumEstimators);
      Trajectory3DReadOnly angularMomentumTrajectory = angularMomentumEstimators.get(segment);
      angularMomentumTrajectory.compute(time);

      predictedAngularMomentum.set(angularMomentumTrajectory.getPosition());
      predictedAngularMomentumRate.set(angularMomentumTrajectory.getVelocity());
   }

   public void computeAngularMomentumTrajectories(List<? extends Trajectory3DReadOnly> comTrajectories)
   {
      computeAngularMomentumTrajectories(comTrajectories,
                                         footTrajectoryPredictor.getPredictedLeftFootTrajectories(),
                                         footTrajectoryPredictor.getPredictedRightFootTrajectories());
   }

   public void computeAngularMomentumTrajectories(List<? extends Trajectory3DReadOnly> comTrajectories,
                                                  List<? extends Trajectory3DReadOnly> secondPotatoTrajectories,
                                                  List<? extends Trajectory3DReadOnly> thirdPotatoTrajectories)
   {
      angularMomentumEstimators.clear();

      double globalStartTime = 0.0;

      for (int activeCoMIdx = 0; activeCoMIdx < comTrajectories.size(); activeCoMIdx++)
      {
         PolynomialEstimator3D angularMomentumTrajectory = angularMomentumEstimators.add();
         angularMomentumTrajectory.reset();
         angularMomentumTrajectory.reshape(5);

         Trajectory3DReadOnly comTrajectory = comTrajectories.get(activeCoMIdx);
         double duration = Math.min(comTrajectory.getDuration(), 10.0);

         angularMomentumTrajectory.setInterval(0.0, duration);

         double segmentDt = Math.min(duration / 5, estimationDt);

         for (double time = 0.0; time <= duration; time += segmentDt)
         {
            double globalTime = time + globalStartTime;
            int activeSecondPotatoIdx = getSegmentNumber(globalTime, secondPotatoTrajectories);
            int activeThirdPotatoIdx = getSegmentNumber(globalTime, thirdPotatoTrajectories);

            if (activeSecondPotatoIdx < 0 && activeThirdPotatoIdx < 0)
               break;

            comTrajectory.compute(time);
            totalAngularMomentum.setToZero();

            if (activeSecondPotatoIdx > -1)
            {
               Trajectory3DReadOnly secondPotatoTrajectory = secondPotatoTrajectories.get(activeSecondPotatoIdx);
               double timeInSegment = getTimeInSegment(activeSecondPotatoIdx, globalTime, secondPotatoTrajectories);
               secondPotatoTrajectory.compute(timeInSegment);

               computeAngularMomentumAtInstant(comTrajectory, secondPotatoTrajectory, potatoMass.getDoubleValue(), angularMomentum);
               totalAngularMomentum.add(angularMomentum);
            }

            if (activeThirdPotatoIdx > -1)
            {
               Trajectory3DReadOnly thirdPotatoTrajectory = thirdPotatoTrajectories.get(activeThirdPotatoIdx);
               double timeInSegment = getTimeInSegment(activeThirdPotatoIdx, globalTime, thirdPotatoTrajectories);
               thirdPotatoTrajectory.compute(timeInSegment);

               computeAngularMomentumAtInstant(comTrajectory, thirdPotatoTrajectory, potatoMass.getDoubleValue(), angularMomentum);
               totalAngularMomentum.add(angularMomentum);
            }

            angularMomentumEstimators.get(activeCoMIdx).addObjectivePosition(time, totalAngularMomentum);
         }


         globalStartTime += duration;
         angularMomentumTrajectory.solve();
      }
   }

   public List<? extends Trajectory3DReadOnly> getAngularMomentumTrajectories()
   {
      return angularMomentumEstimators;
   }

   private void computeAngularMomentumAtInstant(Trajectory3DReadOnly comTrajectory,
                                                Trajectory3DReadOnly potatoTrajectory,
                                                double potatoMass,
                                                Vector3DBasics angularMomentumToPack)
   {
      relativePotatoPosition.sub(potatoTrajectory.getPosition(), comTrajectory.getPosition());
      relativePotatoVelocity.sub(potatoTrajectory.getVelocity(), comTrajectory.getVelocity());

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
