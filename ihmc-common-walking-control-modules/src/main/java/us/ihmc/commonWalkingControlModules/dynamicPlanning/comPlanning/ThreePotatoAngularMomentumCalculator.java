package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.math.trajectories.PolynomialEstimator3D;
import us.ihmc.robotics.math.trajectories.Trajectory3DReadOnly;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class ThreePotatoAngularMomentumCalculator
{
   private static final double estimationDt = 0.05;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble potatoMass = new YoDouble("PotatoMass", registry);

   private final FrameVector3D totalAngularMomentum = new FrameVector3D();
   private final FrameVector3D angularMomentum = new FrameVector3D();

   private final FrameVector3D relativePotatoPosition = new FrameVector3D();
   private final FrameVector3D relativePotatoVelocity = new FrameVector3D();

   private final RecyclingArrayList<PolynomialEstimator3D> angularMomentumEstimators = new RecyclingArrayList<>(PolynomialEstimator3D::new);

   public ThreePotatoAngularMomentumCalculator(double potatoMass, YoRegistry parentRegistry)
   {
      this.potatoMass.set(potatoMass);

      parentRegistry.addChild(registry);
   }

   public void compute(List<Trajectory3DReadOnly> comTrajectories,
                       List<Trajectory3DReadOnly> secondPotatoTrajectories,
                       List<Trajectory3DReadOnly> thirdPotatoTrajectories)
   {
      double time = 0.0;
      int activeCoMIdx = getSegmentContainingTime(time, comTrajectories);
      int activeSecondPotatoIdx = getSegmentContainingTime(time, secondPotatoTrajectories);
      int activeThirdPotatoIdx = getSegmentContainingTime(time, thirdPotatoTrajectories);

      angularMomentumEstimators.clear();
      for (int i = 0; i < comTrajectories.size(); i++)
      {
         PolynomialEstimator3D estimator = angularMomentumEstimators.add();
         estimator.reset();
         estimator.setInterval(comTrajectories.get(i).getStartTime(), comTrajectories.get(i).getEndTime());
      }

      // FIXME don't grow the angular momentum estimate if you don't need to
      while (activeCoMIdx > 0)
      {
         Trajectory3DReadOnly comTrajectory = comTrajectories.get(activeCoMIdx);
         comTrajectory.compute(time);

         totalAngularMomentum.setToZero();

         if (activeSecondPotatoIdx > 0)
         {
            Trajectory3DReadOnly secondPotatoTrajectory = secondPotatoTrajectories.get(activeSecondPotatoIdx);
            secondPotatoTrajectory.compute(time);

            computeAngularMomentumAtInstant(comTrajectory, secondPotatoTrajectory, potatoMass.getDoubleValue(), angularMomentum);
            totalAngularMomentum.add(angularMomentum);
         }

         if (activeThirdPotatoIdx > 0)
         {
            Trajectory3DReadOnly thirdPotatoTrajectory = thirdPotatoTrajectories.get(activeThirdPotatoIdx);
            thirdPotatoTrajectory.compute(time);

            computeAngularMomentumAtInstant(comTrajectory, thirdPotatoTrajectory, potatoMass.getDoubleValue(), angularMomentum);
            totalAngularMomentum.add(angularMomentum);
         }

         angularMomentumEstimators.get(activeCoMIdx).addObjectivePosition(time, totalAngularMomentum);

         time += estimationDt;
         activeCoMIdx = getSegmentContainingTime(time, activeCoMIdx, comTrajectories);
         activeSecondPotatoIdx = getSegmentContainingTime(time, activeSecondPotatoIdx, secondPotatoTrajectories);
         activeThirdPotatoIdx = getSegmentContainingTime(time, activeThirdPotatoIdx, thirdPotatoTrajectories);
      }

      for (int i = 0; i < angularMomentumEstimators.size(); i++)
         angularMomentumEstimators.get(i).solve();
   }

   public List<? extends Trajectory3DReadOnly> getAngularMomentumTrajectories()
   {
      return angularMomentumEstimators;
   }

   private void computeAngularMomentumAtInstant(Trajectory3DReadOnly comTrajectory, Trajectory3DReadOnly potatoTrajectory, double potatoMass, Vector3DBasics angularMomentumToPack)
   {
      relativePotatoPosition.sub(potatoTrajectory.getPosition(), comTrajectory.getPosition());
      relativePotatoVelocity.sub(potatoTrajectory.getVelocity(), comTrajectory.getVelocity());

      angularMomentumToPack.cross(relativePotatoPosition, relativePotatoVelocity);
      angularMomentumToPack.scale(potatoMass);
   }

   private static int getSegmentContainingTime(double time, List<? extends TimeIntervalReadOnly> segments)
   {
      return getSegmentContainingTime(time, 0, segments);
   }

   public static int getSegmentContainingTime(double time, int previousCheckedSegment, List<? extends TimeIntervalReadOnly> segments)
   {
      for (int i = 0; i < segments.size(); i++)
      {
         int wrappedIdx = wrapIndex(previousCheckedSegment + i, segments);
         if (segments.get(wrappedIdx).intervalContains(time))
            return wrappedIdx;
      }

      return -1;
   }

   public static int wrapIndex(int idx, List<?> list)
   {
      if (idx > list.size() - 1)
         return idx - list.size();

      return idx;
   }
}
