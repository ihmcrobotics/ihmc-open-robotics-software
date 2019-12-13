package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerInterface;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

import java.util.ArrayList;
import java.util.List;

public class SimpleDCMPlan implements CoMTrajectoryPlannerInterface
{
   private final FramePoint3D desiredDCMPosition = new FramePoint3D();
   private final FrameVector3D desiredDCMVelocity = new FrameVector3D();
   private final FramePoint3D desiredVRPPosition = new FramePoint3D();
   private final FramePoint3D desiredECMPPosition = new FramePoint3D();
   private final List<Trajectory3D> vrpPlan = new ArrayList<>();
   private final List<Trajectory3D> dcmPlan = new ArrayList<>();

   private final FramePoint3D start =  new FramePoint3D();
   private final FramePoint3D end =  new FramePoint3D();

   private double nominalHeight = Double.NaN;

   @Override
   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      this.nominalHeight = nominalCoMHeight;
   }

   @Override
   public double getNominalCoMHeight()
   {
      return nominalHeight;
   }

   @Override
   public void compute(double globalTime)
   {
      int segmentIndex = getSegmentIndex(globalTime);
      compute(segmentIndex, globalTime);
   }

   @Override
   public void compute(int segmentId, double globalTime)
   {
      compute(segmentId, globalTime, null, null, null, desiredDCMPosition, desiredDCMVelocity, desiredVRPPosition, desiredECMPPosition);
   }

   @Override
   public void compute(int segmentId, double globalTime, FixedFramePoint3DBasics comPositionToPack, FixedFrameVector3DBasics comVelocityToPack,
                FixedFrameVector3DBasics comAccelerationToPack, FixedFramePoint3DBasics dcmPositionToPack, FixedFrameVector3DBasics dcmVelocityToPack,
                FixedFramePoint3DBasics vrpPositionToPack, FixedFramePoint3DBasics ecmpPositionToPack)
   {
      Trajectory3D vrpTrajectory = vrpPlan.get(segmentId);
      Trajectory3D dcmTrajectory = dcmPlan.get(segmentId);
      vrpTrajectory.compute(globalTime);
      dcmTrajectory.compute(globalTime);

      if (comPositionToPack != null)
         comPositionToPack.setToNaN();
      if (comVelocityToPack != null)
         comVelocityToPack.setToNaN();
      if (comAccelerationToPack != null)
         comAccelerationToPack.setToNaN();

      dcmPositionToPack.set(dcmTrajectory.getPosition());
      dcmVelocityToPack.set(dcmTrajectory.getVelocity());
      vrpPositionToPack.set(vrpTrajectory.getPosition());
      ecmpPositionToPack.set(vrpPositionToPack);
      ecmpPositionToPack.subZ(nominalHeight);
   }

   @Override
   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
   }

   @Override
   public void solveForTrajectory(List<? extends ContactStateProvider> contactStates)
   {
      for (int i = 0; i < contactStates.size(); i++)
      {
         ContactStateProvider stateProvider = contactStates.get(i);
         double startTime = stateProvider.getTimeInterval().getStartTime();
         double endTime = stateProvider.getTimeInterval().getEndTime();
         start.set(stateProvider.getCopStartPosition());
         end.set(stateProvider.getCopEndPosition());
         start.addZ(nominalHeight);
         end.addZ(nominalHeight);

         Trajectory3D vrpTrajectory = new Trajectory3D(2);
         vrpTrajectory.setLinear(startTime, endTime, start, end);
         vrpPlan.add(vrpTrajectory);
      }

      recursivelyComputeDCMPlan();
   }

   @Override
   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return desiredDCMPosition;
   }

   @Override
   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return desiredDCMVelocity;
   }

   @Override
   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return null;
   }

   @Override
   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return null;
   }

   @Override
   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return null;
   }

   @Override
   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return desiredVRPPosition;
   }

   @Override
   public FramePoint3DReadOnly getDesiredECMPPosition()
   {
      return desiredECMPPosition;
   }

   public List<Trajectory3D> getVRPTrajectories()
   {
      return vrpPlan;
   }

   private void recursivelyComputeDCMPlan()
   {
   }

   private int getSegmentIndex(double time)
   {
      for (int i = 0; i < vrpPlan.size(); i++)
      {
         if (time >= vrpPlan.get(i).getInitialTime())
            return i;
      }

      throw new RuntimeException("Unable to find segment.");
   }

}