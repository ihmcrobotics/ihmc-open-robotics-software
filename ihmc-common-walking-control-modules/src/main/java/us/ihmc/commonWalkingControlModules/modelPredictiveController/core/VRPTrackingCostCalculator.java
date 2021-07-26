package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrix;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.VRPTrackingCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is used to compute the cost functions for tracking a desired VRP function.
 * This tracking function can be computed as a convex quadratic cost term.
 *
 * TODO review class to make it more efficient for column-wise operations, such as what happens in the sparse matrices
 */
public class VRPTrackingCostCalculator
{
   private final LinearMPCIndexHandler indexHandler;
   private final double gravityZ;

   private final List<FrameVector3DReadOnly> allBasisVectors = new ArrayList<>();
   private final FrameVector3D vrpChange = new FrameVector3D();
   private final FrameVector3D c0Desired = new FrameVector3D();
   private final FrameVector3D c1Desired = new FrameVector3D();
   private final FrameVector3D c2Desired = new FrameVector3D();
   private final FrameVector3D c3Desired = new FrameVector3D();

   private final FramePoint3D desiredValuePosition = new FramePoint3D();
   private final FramePoint3D desiredValueVelocity = new FramePoint3D();

   public VRPTrackingCostCalculator(LinearMPCIndexHandler indexHandler, double gravityZ)
   {
      this.indexHandler = indexHandler;
      this.gravityZ = -Math.abs(gravityZ);
   }

   /**
    * Calculates the quadratic cost function for tracking a nominal VRP trajectory, which is specified by the {@link VRPTrackingCommand}
    *
    * @param costHessianToPack hessian of the quadratic cost function to pack
    * @param costGradientToPack gradient of the quadratic cost function to pack
    * @param objective objective object containing the desired VRP trajectory information
    * @return whether the computation was successful
    */
   public boolean calculateVRPTrackingObjective(DMatrix costHessianToPack, DMatrix costGradientToPack, VRPTrackingCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();
      int startCoMIdx = indexHandler.getComCoefficientStartIndex(segmentNumber, 0);
      int startRhoIdx = indexHandler.getRhoCoefficientStartIndex(segmentNumber);

      if (hasValidVelocityBounds(objective))
         return calculateCubicVRPTrackingObjectiveInternal(costHessianToPack, costGradientToPack, objective, startCoMIdx, startRhoIdx);
      else
         return calculateLinearVRPTrackingObjectiveInternal(costHessianToPack, costGradientToPack, objective, startCoMIdx, startRhoIdx);
   }

   public boolean calculateCompactVRPTrackingObjective(DMatrix costHessianToPack, DMatrix costGradientToPack, VRPTrackingCommand objective)
   {
      if (hasValidVelocityBounds(objective))
         return calculateCubicVRPTrackingObjectiveInternal(costHessianToPack, costGradientToPack, objective, 0, LinearMPCIndexHandler.comCoefficientsPerSegment);
      else
         return calculateLinearVRPTrackingObjectiveInternal(costHessianToPack, costGradientToPack, objective, 0, LinearMPCIndexHandler.comCoefficientsPerSegment);
   }

   private static boolean hasValidVelocityBounds(VRPTrackingCommand objective)
   {
      return !objective.getStartVRPVelocity().containsNaN() && !objective.getEndVRPVelocity().containsNaN();
   }

   private boolean calculateLinearVRPTrackingObjectiveInternal(DMatrix costHessianToPack,
                                                               DMatrix costGradientToPack,
                                                               VRPTrackingCommand objective,
                                                               int startCoMIdx,
                                                               int startRhoIdx)
   {
      double omega = objective.getOmega();
      double w2 = omega * omega;
      double w4 = w2 * w2;

      double tEnd = objective.getEndTime();
      double t2End = tEnd * tEnd;
      double t3End = tEnd * t2End;
      double t4End = tEnd * t3End;
      double t5End = tEnd * t4End;
      double t6End = tEnd * t5End;
      double t7End = tEnd * t6End;

      double c0c0End = t3End / 3.0;
      double c0c1End = 0.5 * t2End;

      double gc0End = t4End / 8.0 - 0.5 * t2End / w2;
      double gc1End = t3End / 6.0 - tEnd / w2;

      double tStart = objective.getStartTime();
      double t2Start = tStart * tStart;
      double t3Start = tStart * t2Start;
      double t4Start = tStart * t3Start;
      double t5Start = tStart * t4Start;
      double t6Start = tStart * t5Start;
      double t7Start = tStart * t6Start;

      double c0c0Start = t3Start / 3.0;
      double c0c1Start = 0.5 * t2Start;

      double gc0Start = t4Start / 8.0 - 0.5 * t2Start / w2;
      double gc1Start = t3Start / 6.0 - tStart / w2;

      double c0c0 = c0c0End - c0c0Start;
      double c0c1 = c0c1End - c0c1Start;

      double gc0 = gc0End - gc0Start;
      double gc1 = gc1End - gc1Start;

      double duration = tEnd - tStart;

      costHessianToPack.set(startCoMIdx, startCoMIdx, c0c0);
      costHessianToPack.set(startCoMIdx, startCoMIdx + 1, c0c1);
      costHessianToPack.set(startCoMIdx + 1, startCoMIdx, c0c1);
      costHessianToPack.set(startCoMIdx + 1, startCoMIdx + 1, duration);

      costHessianToPack.set(startCoMIdx + 2, startCoMIdx + 2, c0c0);
      costHessianToPack.set(startCoMIdx + 2, startCoMIdx + 3, c0c1);
      costHessianToPack.set(startCoMIdx + 3, startCoMIdx + 2, c0c1);
      costHessianToPack.set(startCoMIdx + 3, startCoMIdx + 3, duration);

      costHessianToPack.set(startCoMIdx + 4, startCoMIdx + 4, c0c0);
      costHessianToPack.set(startCoMIdx + 4, startCoMIdx + 5, c0c1);
      costHessianToPack.set(startCoMIdx + 5, startCoMIdx + 4, c0c1);
      costHessianToPack.set(startCoMIdx + 5, startCoMIdx + 5, duration);

      costGradientToPack.set(startCoMIdx + 4, 0, gc0 * gravityZ);
      costGradientToPack.set(startCoMIdx + 5, 0, gc1 * gravityZ);

      allBasisVectors.clear();
      for (int contactPlaneIdx = 0; contactPlaneIdx < objective.getNumberOfContacts(); contactPlaneIdx++)
      {
         MPCContactPlane contactPlane = objective.getContactPlaneHelper(contactPlaneIdx);
         for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);
            for (int i = 0; i < contactPoint.getRhoSize(); i++)
            {
               allBasisVectors.add(contactPoint.getBasisVector(i));
            }
         }
      }

      double a2a2End = t7End / 7.0 - 12.0 * t5End / (5.0 * w2) + 12.0 / w4 * t3End;
      double a2a3End = t6End / 6.0 - 2.0 * t4End / w2 + 6.0 / w4 * t2End;
      double a3a3End = t5End / 5.0 - 4.0 / 3.0 * t3End / w2 + 4.0 / w4 * tEnd;

      double a2c0End = t5End / 5.0 - 2.0 * t3End / w2;
      double a3c0End = t4End / 4.0 - t2End / w2;
      double a2c1End = t4End / 4.0 - 3.0 * t2End / w2;
      double a3c1End = t3End / 3.0 - 2.0 * tEnd / w2;

      double a2a2Start = t7Start / 7.0 - 12.0 * t5Start / (5.0 * w2) + 12.0 / w4 * t3Start;
      double a2a3Start = t6Start / 6.0 - 2.0 * t4Start / w2 + 6.0 / w4 * t2Start;
      double a3a3Start = t5Start / 5.0 - 4.0 / 3.0 * t3Start / w2 + 4.0 / w4 * tStart;

      double a2c0Start = t5Start / 5.0 - 2.0 * t3Start / w2;
      double a3c0Start = t4Start / 4.0 - t2Start / w2;
      double a2c1Start = t4Start / 4.0 - 3.0 * t2Start / w2;
      double a3c1Start = t3Start / 3.0 - 2.0 * tStart / w2;

      double a2a2 = a2a2End - a2a2Start;
      double a2a3 = a2a3End - a2a3Start;
      double a3a3 = a3a3End - a3a3Start;

      double a2c0 = a2c0End - a2c0Start;
      double a3c0 = a3c0End - a3c0Start;
      double a2c1 = a2c1End - a2c1Start;
      double a3c1 = a3c1End - a3c1Start;


      double a2Delta = (t4End - t4Start) / 5.0 - 2.0 * (t2End - t2Start) / w2;
      double a3Delta = (t3End - t3Start) / 4.0 - (tEnd - tStart) / w2;
      double a2 = a2c1;
      double a3 = a3c1;

      double ga2 = (t6End - t6Start) / 12.0 - (t4End - t4Start) / w2 + 3.0 * (t2End - t2Start) / w4;
      double ga3 = (t5End - t5Start) / 10 - 2.0 * (t3End - t3Start) / (3.0 * w2) + 2.0 / w4 * (tEnd - tStart);

      c3Desired.set(objective.getStartVRP());
      c2Desired.sub(objective.getEndVRP(), objective.getStartVRP());

      // TODO review to see if the set vs add methods are correct
      for (int ordinal = 0; ordinal < 3; ordinal++)
      {
         int offset = 2 * ordinal + startCoMIdx;
         double c0 = (t2End - t2Start) / 3.0 * c2Desired.getElement(ordinal) + (t2End - t2Start) / 2.0 * c3Desired.getElement(ordinal);
         double c1 = (tEnd - tStart) / 2.0 * c2Desired.getElement(ordinal) + (tEnd - tStart) * c3Desired.getElement(ordinal);

         MatrixMissingTools.unsafe_add(costGradientToPack, offset, 0, -c0);
         MatrixMissingTools.unsafe_add(costGradientToPack, offset + 1, 0, -c1);
      }

      for (int i = 0; i < allBasisVectors.size(); i++)
      {
         int idxI = 4 * i + startRhoIdx + 2;

         FrameVector3DReadOnly basisVector = allBasisVectors.get(i);

         MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxI, a2a2);
         MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxI + 1, a2a3);
         MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxI, a2a3);
         MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxI + 1, a3a3);

         for (int j = i + 1; j < allBasisVectors.size(); j++)
         {
            FrameVector3DReadOnly otherBasisVector = allBasisVectors.get(j);

            double basisDot = basisVector.dot(otherBasisVector);

            int idxJ = 4 * j + startRhoIdx + 2;

            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxJ, basisDot * a2a2);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxJ + 1, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxJ, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxJ + 1, basisDot * a3a3);

            // we know it's symmetric, and this way we can avoid iterating as much
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ, idxI, basisDot * a2a2);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ + 1, idxI, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ, idxI + 1, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ + 1, idxI + 1, basisDot * a3a3);
         }


         for (int ordinal = 0; ordinal < 3; ordinal++)
         {
            int offset = startCoMIdx + 2 * ordinal;
            double value = basisVector.getElement(ordinal);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset, idxI, a2c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset, idxI + 1, a3c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset + 1, idxI, a2c1 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset + 1, idxI + 1, a3c1 * value);

            // symmetric...
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, offset, a2c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, offset,  a3c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, offset + 1, a2c1 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, offset + 1, a3c1 * value);
         }

         double basisDotDelta = c2Desired.dot(basisVector);
         double basisDotStart = c3Desired.dot(basisVector);
         double basisDotG = basisVector.getZ() * gravityZ;

         MatrixMissingTools.unsafe_add(costGradientToPack, idxI, 0, -basisDotDelta * a2Delta - basisDotStart * a2 + basisDotG * ga2);
         MatrixMissingTools.unsafe_add(costGradientToPack, idxI + 1, 0, -basisDotDelta * a3Delta - basisDotStart * a3 + basisDotG * ga3);
      }

      return true;
   }

   private boolean calculateCubicVRPTrackingObjectiveInternal(DMatrix costHessianToPack,
                                                               DMatrix costGradientToPack,
                                                               VRPTrackingCommand objective,
                                                               int startCoMIdx,
                                                               int startRhoIdx)
   {
      double omega = objective.getOmega();
      double w2 = omega * omega;
      double w4 = w2 * w2;

      double duration = objective.getEndTime() - objective.getStartTime();
      double t2 = duration * duration;
      double t3 = duration * t2;

      double tEnd = objective.getEndTime();
      double t2End = tEnd * tEnd;
      double t3End = tEnd * t2End;
      double t4End = tEnd * t3End;
      double t5End = tEnd * t4End;
      double t6End = tEnd * t5End;
      double t7End = tEnd * t6End;

      double c0c0End = t3End / 3.0;
      double c0c1End = 0.5 * t2End;

      double gc0End = t4End / 8.0 - 0.5 * t2End / w2;
      double gc1End = t3End / 6.0 - tEnd / w2;

      double tStart = objective.getStartTime();
      double t2Start = tStart * tStart;
      double t3Start = tStart * t2Start;
      double t4Start = tStart * t3Start;
      double t5Start = tStart * t4Start;
      double t6Start = tStart * t5Start;
      double t7Start = tStart * t6Start;

      double c0c0Start = t3Start / 3.0;
      double c0c1Start = 0.5 * t2Start;

      double gc0Start = t4Start / 8.0 - 0.5 * t2Start / w2;
      double gc1Start = t3Start / 6.0 - tStart / w2;

      double c0c0 = c0c0End - c0c0Start;
      double c0c1 = c0c1End - c0c1Start;

      double gc0 = gc0End - gc1Start;
      double gc1 = gc1End - gc1Start;

      costHessianToPack.set(startCoMIdx, startCoMIdx, c0c0);
      costHessianToPack.set(startCoMIdx, startCoMIdx + 1, c0c1);
      costHessianToPack.set(startCoMIdx + 1, startCoMIdx, c0c1);
      costHessianToPack.set(startCoMIdx + 1, startCoMIdx + 1, duration);

      costHessianToPack.set(startCoMIdx + 2, startCoMIdx + 2, c0c0);
      costHessianToPack.set(startCoMIdx + 2, startCoMIdx + 3, c0c1);
      costHessianToPack.set(startCoMIdx + 3, startCoMIdx + 2, c0c1);
      costHessianToPack.set(startCoMIdx + 3, startCoMIdx + 3, duration);

      costHessianToPack.set(startCoMIdx + 4, startCoMIdx + 4, c0c0);
      costHessianToPack.set(startCoMIdx + 4, startCoMIdx + 5, c0c1);
      costHessianToPack.set(startCoMIdx + 5, startCoMIdx + 4, c0c1);
      costHessianToPack.set(startCoMIdx + 5, startCoMIdx + 5, duration);

      costGradientToPack.set(startCoMIdx + 4, 0, gc0 * gravityZ);
      costGradientToPack.set(startCoMIdx + 5, 0, gc1 * gravityZ);

      allBasisVectors.clear();
      for (int contactPlaneIdx = 0; contactPlaneIdx < objective.getNumberOfContacts(); contactPlaneIdx++)
      {
         MPCContactPlane contactPlane = objective.getContactPlaneHelper(contactPlaneIdx);
         for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);
            for (int i = 0; i < contactPoint.getRhoSize(); i++)
            {
               allBasisVectors.add(contactPoint.getBasisVector(i));
            }
         }
      }

      double a2a2End = t7End / 7.0 - 12.0 * t5End / (5.0 * w2) + 12.0 / w4 * t3End;
      double a2a3End = t6End / 6.0 - 2.0 * t4End / w2 + 6.0 / w4 * t2End;
      double a3a3End = t5End / 5.0 - 4.0 / 3.0 * t3End / w2 + 4.0 / w4 * tEnd;

      double a2c0End = t5End / 5.0 - 2.0 * t3End / w2;
      double a3c0End = t4End / 4.0 - t2End / w2;
      double a2c1End = t4End / 4.0 - 3.0 * t2End / w2;
      double a3c1End = t3End / 3.0 - 2.0 * tEnd / w2;

      double a2a2Start = t7Start / 7.0 - 12.0 * t5Start / (5.0 * w2) + 12.0 / w4 * t3Start;
      double a2a3Start = t6Start / 6.0 - 2.0 * t4Start / w2 + 6.0 / w4 * t2Start;
      double a3a3Start = t5Start / 5.0 - 4.0 / 3.0 * t3Start / w2 + 4.0 / w4 * tStart;

      double a2c0Start = t5Start / 5.0 - 2.0 * t3Start / w2;
      double a3c0Start = t4Start / 4.0 - t2Start / w2;
      double a2c1Start = t4Start / 4.0 - 3.0 * t2Start / w2;
      double a3c1Start = t3Start / 3.0 - 2.0 * tStart / w2;

      double a2c0Desired = (t7End - t7Start) / 7.0 - 6.0 * (t5End - t5Start) / (5.0 * w2);
      double a3c0Desired = (t6End - t6Start) / 6.0 - (t4End - t4Start) / (2.0 * w2);
      double a2c1Desired = (t6End - t6Start) / 6.0 - 3.0 * (t4End - t4Start) / (2.0 * w2);
      double a3c1Desired = (t5End - t5Start) / 5.0 - 2.0 * (t3End - t3Start) / (3.0 * w2);
      double a2c2Desired = (t5End - t5Start) / 5.0 - 2.0 * (t3End - t3Start) / w2;
      double a3c2Desired = (t4End - t4Start) / 4.0 - (t2End - t2Start) / w2;
      double a2c3Desired = (t4End - t4Start) / 4.0 - 3.0 * (t2End - t2Start) / w2;
      double a3c3Desired = (t3End - t3Start) / 3.0 - 2.0 * (tEnd - tStart) / w2;

      double ga2 = (t6End - t6Start) / 12.0 - (t4End - t4Start) / w2 + 3.0 * (t2End - t2Start) / w4;
      double ga3 = (t5End - t5Start) / 10 - 2.0 * (t3End - t3Start) / (3.0 * w2) + 2.0 / w4 * (tEnd - tStart);

      double a2a2 = a2a2End - a2a2Start;
      double a2a3 = a2a3End - a2a3Start;
      double a3a3 = a3a3End - a3a3Start;

      double a2c0 = a2c0End - a2c0Start;
      double a3c0 = a3c0End - a3c0Start;
      double a2c1 = a2c1End - a2c1Start;
      double a3c1 = a3c1End - a3c1Start;

      vrpChange.sub(objective.getEndVRP(), objective.getStartVRP());

      c0Desired.add(objective.getEndVRPVelocity(), objective.getStartVRPVelocity());
      c0Desired.scale(1.0 / t2);
      c0Desired.scaleAdd(-2.0 / t3, vrpChange, c0Desired);

      c1Desired.add(objective.getEndVRPVelocity(), objective.getStartVRPVelocity());
      c1Desired.add(objective.getStartVRPVelocity());
      c1Desired.scale(-1.0 / duration);
      c1Desired.scaleAdd(3.0 / t2, vrpChange, c1Desired);

      c2Desired.set(objective.getStartVRPVelocity());
      c3Desired.set(objective.getStartVRP());

      desiredValuePosition.setAndScale(t5End / 5.0, c0Desired);
      desiredValuePosition.scaleAdd(t4End / 4.0, c1Desired, desiredValuePosition);
      desiredValuePosition.scaleAdd(t3End / 3.0, c2Desired, desiredValuePosition);
      desiredValuePosition.scaleAdd(t2End / 2.0, c3Desired, desiredValuePosition);

      desiredValueVelocity.setAndScale(t4End / 4.0, c0Desired);
      desiredValueVelocity.scaleAdd(t3End / 3.0, c1Desired, desiredValueVelocity);
      desiredValueVelocity.scaleAdd(t2End / 2.0, c2Desired, desiredValueVelocity);
      desiredValueVelocity.scaleAdd(tEnd, c3Desired, desiredValueVelocity);

      // TODO review to see if the set vs add methods are correct
      for (int ordinal = 0; ordinal < 3; ordinal++)
      {
         int offset = 2 * ordinal + startCoMIdx;

         MatrixMissingTools.unsafe_add(costGradientToPack, offset, 0, -desiredValuePosition.getElement(ordinal));
         MatrixMissingTools.unsafe_add(costGradientToPack, offset + 1, 0, -desiredValueVelocity.getElement(ordinal));
      }

      for (int i = 0; i < allBasisVectors.size(); i++)
      {
         int idxI = 4 * i + startRhoIdx + 2;

         FrameVector3DReadOnly basisVector = allBasisVectors.get(i);

         MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxI, a2a2);
         MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxI + 1, a2a3);
         MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxI, a2a3);
         MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxI + 1, a3a3);

         for (int j = i + 1; j < allBasisVectors.size(); j++)
         {
            FrameVector3DReadOnly otherBasisVector = allBasisVectors.get(j);

            double basisDot = basisVector.dot(otherBasisVector);

            int idxJ = 4 * j + startRhoIdx + 2;

            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxJ, basisDot * a2a2);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxJ + 1, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxJ, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxJ + 1, basisDot * a3a3);

            // we know it's symmetric, and this way we can avoid iterating as much
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ, idxI, basisDot * a2a2);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ + 1, idxI, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ, idxI + 1, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ + 1, idxI + 1, basisDot * a3a3);
         }


         for (int ordinal = 0; ordinal < 3; ordinal++)
         {
            int offset = startCoMIdx + 2 * ordinal;
            double value = basisVector.getElement(ordinal);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset, idxI, a2c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset, idxI + 1, a3c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset + 1, idxI, a2c1 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset + 1, idxI + 1, a3c1 * value);

            // symmetric...
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, offset, a2c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, offset,  a3c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, offset + 1, a2c1 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, offset + 1, a3c1 * value);
         }

         double basisDotC0 = c0Desired.dot(basisVector);
         double basisDotC1 = c1Desired.dot(basisVector);
         double basisDotC2 = c2Desired.dot(basisVector);
         double basisDotC3 = c3Desired.dot(basisVector);
         double basisDotG = basisVector.getZ() * gravityZ;

         MatrixMissingTools.unsafe_add(costGradientToPack, idxI, 0, -basisDotC0 * a2c0Desired - basisDotC1 * a2c1Desired - basisDotC2 * a2c2Desired - basisDotC3 * a2c3Desired + basisDotG * ga2);
         MatrixMissingTools.unsafe_add(costGradientToPack, idxI + 1, 0, -basisDotC0 * a3c0Desired - basisDotC1 * a3c1Desired
                                                                        -basisDotC2 * a3c2Desired - basisDotC3 * a3c3Desired + basisDotG * ga3);
      }

      return true;
   }
}
