package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SimpleCoMTrajectoryPlanner implements CoMTrajectoryProvider
{
   private double nominalCoMHeight;

   private final FramePoint3D initialCoMPosition = new FramePoint3D();

   private final RecyclingArrayList<FramePoint3D> dcmCornerPointPool = new RecyclingArrayList<>(FramePoint3D::new);
   final List<FramePoint3D> dcmCornerPoints = new ArrayList<>();
   private final RecyclingArrayList<Trajectory3D> vrpTrajectoryPool = new RecyclingArrayList<>(() -> new Trajectory3D(4));
   private final RecyclingArrayList<LineSegment3D> vrpWaypointPools = new RecyclingArrayList<>(LineSegment3D::new);
   private final List<Trajectory3D> vrpTrajectories = new ArrayList<>();
   private final List<LineSegment3D> vrpWaypoints = new ArrayList<>();

   final RecyclingArrayList<FramePoint3D> comCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);

   private final DoubleProvider omega0;

   private final FramePoint3D desiredDCMPosition = new FramePoint3D();
   private final FrameVector3D desiredDCMVelocity = new FrameVector3D();

   private final FramePoint3D desiredCoMPosition = new FramePoint3D();
   private final FrameVector3D desiredCoMVelocity = new FrameVector3D();
   private final FrameVector3D desiredCoMAcceleration = new FrameVector3D();

   private final FramePoint3D desiredVRPPosition = new FramePoint3D();
   private final FramePoint3D desiredECMPPosition = new FramePoint3D();

   private CornerPointViewer viewer = null;

   public SimpleCoMTrajectoryPlanner(DoubleProvider omega0)
   {
      this.omega0 = omega0;
   }

   public void setCornerPointViewer(CornerPointViewer viewer)
   {
      this.viewer = viewer;
   }

   @Override
   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      this.nominalCoMHeight = nominalCoMHeight;
   }

   @Override
   public double getNominalCoMHeight()
   {
      return nominalCoMHeight;
   }

   @Override
   public void solveForTrajectory(List<? extends ContactStateProvider> contactSequence)
   {
      computeDCMCornerPoints(contactSequence);
      computeCoMCornerPoints();

      if (viewer != null)
      {
         viewer.updateDCMCornerPoints(dcmCornerPoints);
         viewer.updateCoMCornerPoints(comCornerPoints);
         viewer.updateVRPWaypoints(vrpWaypoints);
      }
   }

   private final FramePoint3D finalVRP = new FramePoint3D();
   private final FramePoint3D startVRP = new FramePoint3D();

   private void computeDCMCornerPoints(List<? extends ContactStateProvider> contactSequence)
   {
      dcmCornerPointPool.clear();
      dcmCornerPoints.clear();
      vrpTrajectoryPool.clear();
      vrpTrajectories.clear();

      FramePoint3DReadOnly finalCoP = contactSequence.get(contactSequence.size() - 1).getCopEndPosition();
      FramePoint3D finalDCM = dcmCornerPointPool.add();
      finalDCM.set(finalCoP);
      finalDCM.addZ(nominalCoMHeight);

      dcmCornerPoints.add(finalDCM);

      for (int i = contactSequence.size() - 1; i >= 0; i--)
      {
         ContactStateProvider contact = contactSequence.get(i);
         double duration = contact.getTimeInterval().getDuration();

         finalVRP.set(contact.getCopEndPosition());
         startVRP.set(contact.getCopStartPosition());
         finalVRP.addZ(nominalCoMHeight);
         startVRP.addZ(nominalCoMHeight);

         FramePoint3D nextCornerPoint = dcmCornerPointPool.add();
         dcmCornerPoints.add(nextCornerPoint);
         CenterOfMassDynamicsTools.computeDesiredDCMPositionBackwardTime(omega0.getValue(), duration, duration, finalDCM, startVRP, finalVRP, nextCornerPoint);

         LineSegment3D vrpSegment = vrpWaypointPools.add();
         Trajectory3D vrpTrajectory = vrpTrajectoryPool.add();
         vrpTrajectories.add(vrpTrajectory);
         vrpWaypoints.add(vrpSegment);
         vrpTrajectory.setLinear(0, duration, startVRP, finalVRP);
         vrpSegment.set(startVRP, finalVRP);

         finalDCM = nextCornerPoint;
      }

      Collections.reverse(dcmCornerPoints);
      Collections.reverse(vrpTrajectories);
      Collections.reverse(vrpWaypoints);
   }

   private void computeCoMCornerPoints()
   {
      comCornerPoints.clear();
      comCornerPoints.add().set(initialCoMPosition);

      for (int i = 0; i < dcmCornerPoints.size() - 1; i++)
      {
         FramePoint3D initialCoMPosition = comCornerPoints.get(i);
         FramePoint3D initialDCMPosition = dcmCornerPoints.get(i);

         double duration = vrpTrajectories.get(i).getDuration();
         startVRP.set(vrpWaypoints.get(i).getFirstEndpoint());
         finalVRP.set(vrpWaypoints.get(i).getSecondEndpoint());

         FramePoint3D finalCoMPosition = comCornerPoints.add();

         CenterOfMassDynamicsTools
               .computeDesiredCoMPositionForwardTime(omega0.getValue(), duration, duration, initialCoMPosition, initialDCMPosition, startVRP, finalVRP,
                                                     finalCoMPosition);
      }
   }

   @Override
   public void compute(int segmentId, double timeInPhase)
   {
      compute(segmentId, timeInPhase, desiredCoMPosition, desiredCoMVelocity, desiredCoMAcceleration, desiredDCMPosition, desiredDCMVelocity,
              desiredVRPPosition, desiredECMPPosition);
   }

   @Override
   public void compute(int segmentId, double timeInPhase, FixedFramePoint3DBasics comPositionToPack, FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack, FixedFramePoint3DBasics dcmPositionToPack, FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack, FixedFramePoint3DBasics ecmpPositionToPack)
   {
      Trajectory3D vrpTrajectory = vrpTrajectories.get(segmentId);
      LineSegment3DReadOnly vrpSegment = vrpWaypoints.get(segmentId);
      startVRP.set(vrpSegment.getFirstEndpoint());
      finalVRP.set(vrpSegment.getSecondEndpoint());

      double omega = omega0.getValue();

      double duration = vrpTrajectory.getDuration();

      vrpTrajectory.compute(timeInPhase);

      FramePoint3DReadOnly initialDCM = dcmCornerPoints.get(segmentId);
      FramePoint3DReadOnly initialCoM = comCornerPoints.get(segmentId);

      CenterOfMassDynamicsTools.computeDesiredDCMPositionForwardTime(omega, timeInPhase, duration, initialDCM, startVRP, finalVRP, dcmPositionToPack);
      vrpPositionToPack.set(vrpTrajectory.getPosition());
      ecmpPositionToPack.set(vrpPositionToPack);
      ecmpPositionToPack.subZ(nominalCoMHeight);

      CapturePointTools.computeCapturePointVelocity(dcmPositionToPack, vrpPositionToPack, omega, dcmVelocityToPack);

      CenterOfMassDynamicsTools
            .computeDesiredCoMPositionForwardTime(omega, timeInPhase, duration, initialCoM, initialDCM, startVRP, finalVRP, comPositionToPack);
      CapturePointTools.computeCenterOfMassVelocity(comPositionToPack, dcmPositionToPack, omega, comVelocityToPack);
      CapturePointTools.computeCenterOfMassAcceleration(comVelocityToPack, dcmVelocityToPack, omega, comAccelerationToPack);
   }

   @Override
   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      initialCoMPosition.set(centerOfMassPosition);
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
      return desiredCoMPosition;
   }

   @Override
   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return desiredCoMVelocity;
   }

   @Override
   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return desiredCoMAcceleration;
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

   @Override
   public List<Trajectory3D> getVRPTrajectories()
   {
      return vrpTrajectories;
   }
}
