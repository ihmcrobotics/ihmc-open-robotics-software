package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerInterface;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.ArrayList;
import java.util.List;

public class SimpleDCMPlan implements CoMTrajectoryPlannerInterface
{
   private final FramePoint3D desiredDCMPosition = new FramePoint3D();
   private final FrameVector3D desiredDCMVelocity = new FrameVector3D();
   private final FramePoint3D desiredVRPPosition = new FramePoint3D();
   private final FramePoint3D desiredECMPPosition = new FramePoint3D();
   private final List<Trajectory3D> vrpPlan = new ArrayList<>();
   private final List<FramePoint3DReadOnly> endDCMPositions = new ArrayList<>();

   private final FramePoint3D start =  new FramePoint3D();
   private final FramePoint3D end =  new FramePoint3D();

   private double nominalHeight = Double.NaN;

   private final double omega;

   public SimpleDCMPlan(double omega)
   {
      this.omega = omega;
   }

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

   private FramePoint3D startCoP = new FramePoint3D();
   private FramePoint3D endCoP = new FramePoint3D();
   @Override
   public void compute(int segmentId, double globalTime, FixedFramePoint3DBasics comPositionToPack, FixedFrameVector3DBasics comVelocityToPack,
                FixedFrameVector3DBasics comAccelerationToPack, FixedFramePoint3DBasics dcmPositionToPack, FixedFrameVector3DBasics dcmVelocityToPack,
                FixedFramePoint3DBasics vrpPositionToPack, FixedFramePoint3DBasics ecmpPositionToPack)
   {
      Trajectory3D vrpTrajectory = vrpPlan.get(segmentId);
      vrpTrajectory.compute(globalTime);

      vrpTrajectory.getStartPoint(startCoP);
      vrpTrajectory.getEndPoint(endCoP);
      FramePoint3DReadOnly endDCM = endDCMPositions.get(segmentId);

      dcmPositionToPack.set(integrateBackwards(globalTime, vrpTrajectory.getDuration(), startCoP, endCoP, endDCM, omega));
      dcmVelocityToPack.sub(dcmPositionToPack, vrpTrajectory.getPosition());
      dcmVelocityToPack.scale(omega);

      if (comPositionToPack != null)
         comPositionToPack.setToNaN();
      if (comVelocityToPack != null)
         comVelocityToPack.setToNaN();
      if (comAccelerationToPack != null)
         comAccelerationToPack.setToNaN();

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

      recursivelyComputeDCMPlan(contactStates);
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

   private final FramePoint3D endDCM = new FramePoint3D();
   private void recursivelyComputeDCMPlan(List<? extends ContactStateProvider> contactStates)
   {
      endDCM.set(contactStates.get(vrpPlan.size() - 1).getCopEndPosition());
      endDCM.addZ(nominalHeight);

      endDCMPositions.clear();
      for (int i = vrpPlan.size() - 1; i >= 0; i--)
      {
         endDCMPositions.add(new FramePoint3D(endDCM));
         ContactStateProvider contactStateProvider = contactStates.get(i);
         double startTime = contactStateProvider.getTimeInterval().getStartTime();
         double endTime = contactStateProvider.getTimeInterval().getEndTime();
         endCoP.set(contactStateProvider.getCopEndPosition());
         startCoP.set(contactStateProvider.getCopStartPosition());
         endCoP.addZ(nominalHeight);
         startCoP.addZ(nominalHeight);

         double duration = endTime - startTime;
         endDCM.set(integrateBackwards(duration, duration, startCoP, endCoP, endDCM, omega));
      }
   }

   private static FramePoint3DReadOnly integrateBackwards(double time, double duration, FramePoint3DReadOnly startCoPPosition,
                                                          FramePoint3DReadOnly endCoPPosition, FramePoint3DReadOnly endDCMPosition, double omega)
   {
      FramePoint3D startDCMPosition = new FramePoint3D();

      startDCMPosition.sub(endCoPPosition, startCoPPosition);
      startDCMPosition.scale(time / duration + 1.0 / (duration * omega));
      startDCMPosition.sub(startCoPPosition);
      startDCMPosition.add(endDCMPosition);
      startDCMPosition.scale(Math.exp(-omega * time));

      startDCMPosition.scaleAdd(-1.0 / (omega * duration), startCoPPosition, startDCMPosition);
      startDCMPosition.scaleAdd(1.0 / (omega * duration), endCoPPosition, startDCMPosition);

      startDCMPosition.add(startCoPPosition);

      return startDCMPosition;
   }

   private static FramePoint3DReadOnly integrateForwards(double time, double duration, FramePoint3DReadOnly startCoPPosition,
                                                         FramePoint3DReadOnly endCoPPosition, FramePoint3DReadOnly startDCMPosition, double omega)
   {
      FramePoint3D endDCMPosition = new FramePoint3D();

      endDCMPosition.sub(startCoPPosition, endCoPPosition);
      endDCMPosition.scale(1.0 / (omega * duration));
      endDCMPosition.add(startDCMPosition);
      endDCMPosition.sub(startCoPPosition);
      endDCMPosition.scale(Math.exp(omega * time));

      double psi = time / duration + 1.0 / (omega * duration);
      endDCMPosition.scaleAdd(psi, endCoPPosition, endDCMPosition);
      endDCMPosition.scaleAdd(-psi, startCoPPosition, endDCMPosition);
      endDCMPosition.add(startCoPPosition);

      return endDCMPosition;
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