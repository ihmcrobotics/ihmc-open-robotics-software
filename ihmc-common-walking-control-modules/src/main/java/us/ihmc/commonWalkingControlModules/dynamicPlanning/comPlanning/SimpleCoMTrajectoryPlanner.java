package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

import java.util.Collections;
import java.util.List;

public class SimpleCoMTrajectoryPlanner implements CoMTrajectoryPlannerInterface
{
   private double nominalCoMHeight;

   private final FramePoint3D initialCoMPosition = new FramePoint3D();

   private final RecyclingArrayList<FramePoint3D> dcmCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> comCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);

   private final double omega0;

   private final FramePoint3D desiredDCMPosition = new FramePoint3D();
   private final FrameVector3D desiredDCMVelocity = new FrameVector3D();

   private final FramePoint3D desiredCoMPosition = new FramePoint3D();
   private final FrameVector3D desiredCoMVelocity = new FrameVector3D();
   private final FrameVector3D desiredCoMAcceleration = new FrameVector3D();

   private final FramePoint3D desiredVRPPosition = new FramePoint3D();
   private final FramePoint3D desiredECMPPosition = new FramePoint3D();

   public SimpleCoMTrajectoryPlanner(double omega0)
   {
      this.omega0 = omega0;
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
      computeCoMCornerPoints(contactSequence);
   }

   private final FramePoint3D finalVRP = new FramePoint3D();
   private final FramePoint3D startVRP = new FramePoint3D();

   private void computeDCMCornerPoints(List<? extends ContactStateProvider> contactSequence)
   {
      dcmCornerPoints.clear();

      FramePoint3DReadOnly finalCoP = contactSequence.get(contactSequence.size() - 1).getCopEndPosition();
      FramePoint3D finalDCM = dcmCornerPoints.add();
      finalDCM.set(finalCoP);
      finalDCM.addZ(nominalCoMHeight);

      for (int i = contactSequence.size() - 1; i >= 0; i--)
      {
         ContactStateProvider contact = contactSequence.get(i);
         double duration = contact.getTimeInterval().getDuration();

         finalVRP.set(contact.getCopEndPosition());
         startVRP.set(contact.getCopStartPosition());
         finalVRP.addZ(nominalCoMHeight);
         startVRP.addZ(nominalCoMHeight);

         FramePoint3D nextCornerPoint = dcmCornerPoints.add();
         CenterOfMassDynamicsTools.computeDesiredDCMPositionBackwardTime(omega0, duration, duration, finalDCM, startVRP, finalVRP, nextCornerPoint);

         finalDCM = nextCornerPoint;
      }

      Collections.reverse(dcmCornerPoints);
   }

   private void computeCoMCornerPoints(List<? extends ContactStateProvider> contactSequence)
   {
      comCornerPoints.clear();
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
      // TODO
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
}
