package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerIndexHandler;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.MultipleCoMSegmentTrajectoryGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLongTime;

public class LinearTrajectoryHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final CoMTrajectoryPlanner positionInitializationCalculator;

   protected final List<ContactPlaneProvider> planningWindow = new ArrayList<>();

   private final LinearMPCIndexHandler indexHandler;
   private final double gravityZ;

   final DMatrixRMaj xCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj yCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj zCoefficientVector = new DMatrixRMaj(0, 1);

   private final FixedFramePoint3DBasics desiredCoMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMVelocity = new FrameVector3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMAcceleration = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredDCMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredDCMVelocity = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredVRPPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredVRPVelocity = new FrameVector3D(worldFrame);
   private final FixedFramePoint3DBasics desiredECMPPosition = new FramePoint3D(worldFrame);

   private final DMatrixRMaj coefficientArray = new DMatrixRMaj(0, 3);

   private final MultipleCoMSegmentTrajectoryGenerator comTrajectory;
   private final RecyclingArrayList<Polynomial3DBasics> vrpTrajectoryPool = new RecyclingArrayList<>(() -> new Polynomial3D(4));
   private double currentTimeInState;

   public LinearTrajectoryHandler(LinearMPCIndexHandler indexHandler, double gravityZ, double nominalCoMHeight, YoRegistry registry)
   {
      this.indexHandler = indexHandler;
      this.gravityZ = Math.abs(gravityZ);

      positionInitializationCalculator = new CoMTrajectoryPlanner(gravityZ, nominalCoMHeight, registry);
      comTrajectory = new MultipleCoMSegmentTrajectoryGenerator("desiredCoMTrajectory", registry);
   }

   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      positionInitializationCalculator.setNominalCoMHeight(nominalCoMHeight);
   }

   public void setInitialCenterOfMassPositionState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      positionInitializationCalculator.setInitialCenterOfMassState(centerOfMassPosition, centerOfMassVelocity);
   }

   public void solveForTrajectoryOutsidePreviewWindow(List<ContactPlaneProvider> fullContactSequence)
   {
      positionInitializationCalculator.solveForTrajectory(fullContactSequence);
   }

   public void extractSolutionForPreviewWindow(DMatrixRMaj solutionCoefficients,
                                               List<ContactPlaneProvider> planningWindow,
                                               List<? extends List<ContactPlaneHelper>> contactPlaneHelpers,
                                               double currentTimeInState)
   {
      this.currentTimeInState = currentTimeInState;
      int numberOfPhases = planningWindow.size();
      this.planningWindow.clear();
      for (int i = 0; i < numberOfPhases; i++)
         this.planningWindow.add(planningWindow.get(i));

      xCoefficientVector.reshape(6 * numberOfPhases, 1);
      yCoefficientVector.reshape(6 * numberOfPhases, 1);
      zCoefficientVector.reshape(6 * numberOfPhases, 1);
      xCoefficientVector.zero();
      yCoefficientVector.zero();
      zCoefficientVector.zero();

      for (int sequence = 0; sequence < contactPlaneHelpers.size(); sequence++)
      {
         int coeffStartIdx = indexHandler.getRhoCoefficientStartIndex(sequence);

         for (int contact = 0; contact < contactPlaneHelpers.get(sequence).size(); contact++)
         {
            ContactPlaneHelper contactPlaneHelper = contactPlaneHelpers.get(sequence).get(contact);
            contactPlaneHelper.computeContactForceCoefficientMatrix(solutionCoefficients, coeffStartIdx);
            coeffStartIdx += contactPlaneHelper.getCoefficientSize();
         }
      }

      for (int i = 0; i < numberOfPhases; i++)
      {
         int positionVectorStart = 6 * i;

         xCoefficientVector.set(positionVectorStart + 4, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 0), 0));
         yCoefficientVector.set(positionVectorStart + 4, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 1), 0));
         zCoefficientVector.set(positionVectorStart + 4, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 2), 0));

         xCoefficientVector.set(positionVectorStart + 5, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 0) + 1, 0));
         yCoefficientVector.set(positionVectorStart + 5, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 1) + 1, 0));
         zCoefficientVector.set(positionVectorStart + 5, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 2) + 1, 0));

         for (int contactIdx = 0; contactIdx < contactPlaneHelpers.get(i).size(); contactIdx++)
         {
            ContactPlaneHelper contactPlaneHelper = contactPlaneHelpers.get(i).get(contactIdx);
            DMatrixRMaj contactCoefficientMatrix = contactPlaneHelper.getContactWrenchCoefficientMatrix();

            xCoefficientVector.add(positionVectorStart, 0, contactCoefficientMatrix.get(0, 0));
            yCoefficientVector.add(positionVectorStart, 0, contactCoefficientMatrix.get(1, 0));
            zCoefficientVector.add(positionVectorStart, 0, contactCoefficientMatrix.get(2, 0));

            xCoefficientVector.add(positionVectorStart + 1, 0, contactCoefficientMatrix.get(0, 1));
            yCoefficientVector.add(positionVectorStart + 1, 0, contactCoefficientMatrix.get(1, 1));
            zCoefficientVector.add(positionVectorStart + 1, 0, contactCoefficientMatrix.get(2, 1));

            xCoefficientVector.add(positionVectorStart + 2, 0, contactCoefficientMatrix.get(0, 2));
            yCoefficientVector.add(positionVectorStart + 2, 0, contactCoefficientMatrix.get(1, 2));
            zCoefficientVector.add(positionVectorStart + 2, 0, contactCoefficientMatrix.get(2, 2));

            xCoefficientVector.add(positionVectorStart + 3, 0, contactCoefficientMatrix.get(0, 3));
            yCoefficientVector.add(positionVectorStart + 3, 0, contactCoefficientMatrix.get(1, 3));
            zCoefficientVector.add(positionVectorStart + 3, 0, contactCoefficientMatrix.get(2, 3));
         }
         zCoefficientVector.add(positionVectorStart + 3, 0, -0.5 * gravityZ);
      }


      coefficientArray.reshape(numRows, 3);

      MatrixTools.setMatrixBlock(coefficientArray, 0, 0, xCoefficientVector, 0, 0, numRows, 1, 1.0);
      MatrixTools.setMatrixBlock(coefficientArray, 0, 1, yCoefficientVector, 0, 0, numRows, 1, 1.0);
      MatrixTools.setMatrixBlock(coefficientArray, 0, 2, zCoefficientVector, 0, 0, numRows, 1, 1.0);

      clearTrajectory();

      int startRow = 0;
      for (int i = 0; i < planningWindow.size(); i++)
      {
         TimeIntervalReadOnly timeInterval = planningWindow.get(i).getTimeInterval();

         comTrajectory.appendSegment(timeInterval, omega, coefficientArray, startRow);

         double duration = Math.min(timeInterval.getDuration(), sufficientlyLongTime);
         computeVRPBoundaryConditionsFromCoefficients(startRow,
                                                      coefficientArray,
                                                      omega,
                                                      duration,
                                                      vrpStartPosition,
                                                      vrpStartVelocity,
                                                      vrpEndPosition,
                                                      vrpEndVelocity);
         Polynomial3DBasics vrpTrajectory = vrpTrajectoryPool.add();
         vrpTrajectory.setCubic(0.0, duration, vrpStartPosition, vrpStartVelocity, vrpEndPosition, vrpEndVelocity);
         vrpTrajectory.getTimeInterval().setInterval(0.0, timeInterval.getDuration());
         this.vrpTrajectories.add(vrpTrajectory);

         startRow += CoMTrajectoryPlannerIndexHandler.polynomialCoefficientsPerSegment;
      }
      comTrajectory.initialize();
   }

   public void compute(double timeInPhase, double omega)
   {
      boolean success;

      if (isTimeInPlanningWindow(timeInPhase))
         success = computeInPlanningWindow(timeInPhase, omega);
      else
         success = false;

      if (!success)
         computeOutsideOfPlanningWindow(timeInPhase);
   }

   public void fastComputePosition(double timeInPhase, double omega, FixedFramePoint3DBasics desiredCoMPosition)
   {
      boolean success;

      if (isTimeInPlanningWindow(timeInPhase))
         success = fastComputePositionInPlanningWindow(timeInPhase, omega, desiredCoMPosition);
      else
         success = false;

      if (!success)
         fastComputePositionOutsideOfPlanningWindow(timeInPhase, desiredCoMPosition);
   }

   protected boolean isTimeInPlanningWindow(double time)
   {
      return time < comTrajectory.getEndTime();
   }

   private boolean computeInPlanningWindow(double timeInPhase, double omega)
   {
      int segmentNumber = getPreviewWindowSegmentNumber(timeInPhase);
      if (segmentNumber < 0)
         return false;

      double timeInSegment = getTimeInSegment(segmentNumber, timeInPhase);

      return computeInPlanningWindow(segmentNumber, timeInSegment, omega);
   }


   protected boolean computeInPlanningWindow(int segmentNumber, double timeInSegment, double omega)
   {
      comTrajectory.getSegment(segmentNumber).compute(timeInSegment,
                                                      desiredCoMPosition,
                                                      desiredCoMVelocity,
                                                      desiredCoMAcceleration,
                                                      desiredDCMPosition,
                                                      desiredDCMVelocity,
                                                      desiredVRPPosition,
                                                      desiredVRPVelocity);


      double nominalHeight = gravityZ / MathTools.square(omega);
      desiredECMPPosition.set(desiredVRPPosition);
      desiredECMPPosition.subZ(nominalHeight);

      return true;
   }

   private boolean fastComputePositionInPlanningWindow(double timeInPhase, double omega, FixedFramePoint3DBasics desiredCoMPositionToPack)
   {
      int segmentNumber = getPreviewWindowSegmentNumber(timeInPhase);
      if (segmentNumber < 0)
         return false;

      double timeInSegment = getTimeInSegment(segmentNumber, timeInPhase);

      comTrajectory.getSegment(segmentNumber).computeCoMPosition(timeInSegment, desiredCoMPositionToPack);
      return true;
   }

   public int getPreviewWindowSegmentNumber(double time)
   {
      for (int i = 0; i < planningWindow.size(); i++)
      {
         if (planningWindow.get(i).getTimeInterval().intervalContains(time))
            return i;
      }

      return -1;
   }



   public double getTimeInSegment(int segmentNumber, double timeInPhase)
   {
      timeInPhase -= currentTimeInState;

      for (int i = 0; i < segmentNumber; i++)
         timeInPhase -= planningWindow.get(i).getTimeInterval().getDuration();

      return timeInPhase;
   }

   private void fastComputePositionOutsideOfPlanningWindow(double time, FixedFramePoint3DBasics desiredCoMPositionToPack)
   {
      positionInitializationCalculator.fastComputeDesiredPosition(time, desiredCoMPositionToPack);
   }

   protected void computeOutsideOfPlanningWindow(double time)
   {
      positionInitializationCalculator.compute(time);

      desiredCoMPosition.set(positionInitializationCalculator.getDesiredCoMPosition());
      desiredCoMVelocity.set(positionInitializationCalculator.getDesiredCoMVelocity());
      desiredCoMAcceleration.set(positionInitializationCalculator.getDesiredCoMAcceleration());

      desiredVRPPosition.set(positionInitializationCalculator.getDesiredVRPPosition());
      desiredVRPVelocity.set(positionInitializationCalculator.getDesiredVRPVelocity());

      desiredDCMPosition.set(positionInitializationCalculator.getDesiredDCMPosition());
      desiredDCMVelocity.set(positionInitializationCalculator.getDesiredDCMVelocity());

      desiredECMPPosition.set(positionInitializationCalculator.getDesiredECMPPosition());
   }

   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return desiredCoMVelocity;
   }

   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return desiredCoMAcceleration;
   }

   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return desiredVRPPosition;
   }

   public FrameVector3DReadOnly getDesiredVRPVelocity()
   {
      return desiredVRPVelocity;
   }

   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return desiredDCMPosition;
   }

   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return desiredDCMVelocity;
   }

   public FramePoint3DReadOnly getDesiredECMPPosition()
   {
      return desiredECMPPosition;
   }

   private static void computeVRPBoundaryConditionsFromCoefficients(int startRow,
                                                                    DMatrixRMaj coefficientArray,
                                                                    double omega,
                                                                    double duration,
                                                                    FixedFramePoint3DBasics startPosition,
                                                                    FixedFrameVector3DBasics startVelocity,
                                                                    FixedFramePoint3DBasics endPosition,
                                                                    FixedFrameVector3DBasics endVelocity)
   {
      startPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      startVelocity.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      endPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      endVelocity.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      double omega2 = omega * omega;
      double t2 = duration * duration;
      double t3 = duration * t2;

      for (Axis3D axis : Axis3D.values)
      {
         int element = axis.ordinal();
         double startPositionElement = coefficientArray.get(startRow + 5, element) - 2.0 / omega2 * coefficientArray.get(startRow + 3, element);
         double startVelocityElement = coefficientArray.get(startRow + 4, element) - 6.0 / omega2 * coefficientArray.get(startRow + 2, element);
         startPosition.setElement(element, startPositionElement);
         startVelocity.setElement(element, startVelocityElement);

         double endPositionElement =
               coefficientArray.get(startRow + 2, element) * t3 + coefficientArray.get(startRow + 3, element) * t2 + startVelocityElement * duration
               + startPositionElement;
         double endVelocityElement =
               3.0 * coefficientArray.get(startRow + 2, element) * t2 + 2.0 * coefficientArray.get(startRow + 3, element) * duration + startVelocityElement;

         endPosition.setElement(element, endPositionElement);
         endVelocity.setElement(element, endVelocityElement);
      }
   }

}
