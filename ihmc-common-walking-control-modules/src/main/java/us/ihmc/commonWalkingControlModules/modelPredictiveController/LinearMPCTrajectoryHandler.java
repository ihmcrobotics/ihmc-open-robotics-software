package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerIndexHandler;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectorySegment;
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
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DReadOnly;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLongTime;

public class LinearMPCTrajectoryHandler
{
   private final CoMTrajectoryPlanner positionInitializationCalculator;

   protected final List<ContactPlaneProvider> planningWindow = new ArrayList<>();

   private final LinearMPCIndexHandler indexHandler;
   private final double gravityZ;

   final DMatrixRMaj xCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj yCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj zCoefficientVector = new DMatrixRMaj(0, 1);

   private final DMatrixRMaj coefficientArray = new DMatrixRMaj(0, 3);

   private final MultipleCoMSegmentTrajectoryGenerator comTrajectory;
   private final RecyclingArrayList<Polynomial3DBasics> vrpTrajectories = new RecyclingArrayList<>(() -> new Polynomial3D(4));

   private final CoMTrajectorySegment segmentToAppend = new CoMTrajectorySegment();

   public LinearMPCTrajectoryHandler(LinearMPCIndexHandler indexHandler, double gravityZ, double nominalCoMHeight, YoRegistry registry)
   {
      this.indexHandler = indexHandler;
      this.gravityZ = Math.abs(gravityZ);

      positionInitializationCalculator = new CoMTrajectoryPlanner(gravityZ, nominalCoMHeight, registry);
      comTrajectory = new MultipleCoMSegmentTrajectoryGenerator("desiredCoMTrajectory", registry);
   }

   public void clearTrajectory()
   {
      comTrajectory.clear();
      vrpTrajectories.clear();
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

      comTrajectory.clear();
      for (int i = 0; i < positionInitializationCalculator.getCoMTrajectory().getCurrentNumberOfSegments(); i++)
         comTrajectory.appendSegment(positionInitializationCalculator.getCoMTrajectory().getSegment(i));
      comTrajectory.initialize();

      vrpTrajectories.clear();
      for (int i = 0; i < positionInitializationCalculator.getVRPTrajectories().size(); i++)
         vrpTrajectories.add().set(positionInitializationCalculator.getVRPTrajectories().get(i));
   }

   private final FramePoint3D vrpStartPosition = new FramePoint3D();
   private final FrameVector3D vrpStartVelocity = new FrameVector3D();
   private final FramePoint3D vrpEndPosition = new FramePoint3D();
   private final FrameVector3D vrpEndVelocity = new FrameVector3D();

   public void extractSolutionForPreviewWindow(DMatrixRMaj solutionCoefficients,
                                               List<ContactPlaneProvider> planningWindow,
                                               List<? extends List<ContactPlaneHelper>> contactPlaneHelpers,
                                               double currentTimeInState,
                                               double omega)
   {
      int numberOfPhases = planningWindow.size();
      this.planningWindow.clear();
      for (int i = 0; i < numberOfPhases; i++)
         this.planningWindow.add(planningWindow.get(i));

      computeCoMSegmentCoefficients(solutionCoefficients,
                                    contactPlaneHelpers,
                                    xCoefficientVector,
                                    yCoefficientVector,
                                    zCoefficientVector);

      int numRows = xCoefficientVector.getNumRows();
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
         Polynomial3DBasics vrpTrajectory = vrpTrajectories.add();
         vrpTrajectory.setCubic(0.0, duration, vrpStartPosition, vrpStartVelocity, vrpEndPosition, vrpEndVelocity);
         vrpTrajectory.getTimeInterval().setInterval(0.0, timeInterval.getDuration());

         startRow += CoMTrajectoryPlannerIndexHandler.polynomialCoefficientsPerSegment;
      }

      MultipleCoMSegmentTrajectoryGenerator comTrajectoryOutsideWindow = positionInitializationCalculator.getCoMTrajectory();
      List<Polynomial3DReadOnly> vrpTrajectoryOutsideWindow = positionInitializationCalculator.getVRPTrajectories();
      while (comTrajectory.getEndTime() < comTrajectoryOutsideWindow.getEndTime())
      {
         int segmentIndexToAdd = getSegmentIndexContainingTime(comTrajectory.getEndTime() + 1e-5, positionInitializationCalculator.getCoMTrajectory());
         if (segmentIndexToAdd == -1)
            throw new RuntimeException("oops.");

         CoMTrajectorySegment segmentToAdd = comTrajectoryOutsideWindow.getSegment(segmentIndexToAdd);
         CoMTrajectorySegment lastSegmentOfPreview = comTrajectory.getSegment(comTrajectory.getCurrentNumberOfSegments() - 1);
         if (MathTools.epsilonEquals(lastSegmentOfPreview.getTimeInterval().getEndTime(), segmentToAdd.getTimeInterval().getStartTime(), 1e-3))
         {
            comTrajectory.appendSegment(segmentToAdd);
            vrpTrajectories.add().set(vrpTrajectoryOutsideWindow.get(segmentIndexToAdd));
         }
         else
         {
            double durationToRemove = lastSegmentOfPreview.getTimeInterval().getEndTime() - segmentToAdd.getTimeInterval().getStartTime();
            segmentToAppend.set(segmentToAdd);
            segmentToAppend.shiftStartOfSegment(durationToRemove);
            comTrajectory.appendSegment(segmentToAppend);

            // TODO make this cleaner
            double duration = Math.min(segmentToAppend.getTimeInterval().getDuration(), sufficientlyLongTime);
            computeVRPBoundaryConditionsFromCoefficients(segmentToAppend,
                                                         duration,
                                                         omega,
                                                         vrpStartPosition,
                                                         vrpStartVelocity,
                                                         vrpEndPosition,
                                                         vrpEndVelocity);
            Polynomial3DBasics vrpTrajectory = vrpTrajectories.add();
            vrpTrajectory.setCubic(0.0, duration, vrpStartPosition, vrpStartVelocity, vrpEndPosition, vrpEndVelocity);
            vrpTrajectory.getTimeInterval().setInterval(0.0, segmentToAppend.getTimeInterval().getDuration());
         }


         segmentIndexToAdd++;
         for (;segmentIndexToAdd < comTrajectoryOutsideWindow.getCurrentNumberOfSegments(); segmentIndexToAdd++)
         {
            comTrajectory.appendSegment(comTrajectoryOutsideWindow.getSegment(segmentIndexToAdd));
            vrpTrajectories.add().set(vrpTrajectoryOutsideWindow.get(segmentIndexToAdd));
         }
      }
      comTrajectory.initialize();
   }

   private int getSegmentIndexContainingTime(double time, MultipleCoMSegmentTrajectoryGenerator trajectory)
   {
      for (int i = 0; i < trajectory.getCurrentNumberOfSegments(); i++)
      {
         CoMTrajectorySegment segment = trajectory.getSegment(i);
         if (segment.getTimeInterval().intervalContains(time))
            return i;
      }

      return -1;
   }

   public void compute(double timeInPhase)
   {
      comTrajectory.compute(timeInPhase);
   }

   public List<? extends Polynomial3DReadOnly> getVrpTrajectories()
   {
      return vrpTrajectories;
   }

   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return comTrajectory.getPosition();
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return comTrajectory.getVelocity();
   }

   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return comTrajectory.getAcceleration();
   }

   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return comTrajectory.getSegment(comTrajectory.getCurrentSegmentIndex()).getVRPPosition();
   }

   public FrameVector3DReadOnly getDesiredVRPVelocity()
   {
      return comTrajectory.getSegment(comTrajectory.getCurrentSegmentIndex()).getVRPVelocity();
   }

   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return comTrajectory.getSegment(comTrajectory.getCurrentSegmentIndex()).getDCMPosition();
   }

   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return comTrajectory.getSegment(comTrajectory.getCurrentSegmentIndex()).getDCMVelocity();
   }

   private void computeCoMSegmentCoefficients(DMatrixRMaj solutionCoefficients,
                                              List<? extends List<ContactPlaneHelper>> contactPlaneHelpers,
                                              DMatrixRMaj xCoefficientVectorToPack,
                                              DMatrixRMaj yCoefficientVectorToPack,
                                              DMatrixRMaj zCoefficientVectorToPack)
   {
      int numberOfPhases = contactPlaneHelpers.size();

      xCoefficientVectorToPack.reshape(6 * numberOfPhases, 1);
      yCoefficientVectorToPack.reshape(6 * numberOfPhases, 1);
      zCoefficientVectorToPack.reshape(6 * numberOfPhases, 1);
      xCoefficientVectorToPack.zero();
      yCoefficientVectorToPack.zero();
      zCoefficientVectorToPack.zero();

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

         xCoefficientVectorToPack.set(positionVectorStart + 4, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 0), 0));
         yCoefficientVectorToPack.set(positionVectorStart + 4, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 1), 0));
         zCoefficientVectorToPack.set(positionVectorStart + 4, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 2), 0));

         xCoefficientVectorToPack.set(positionVectorStart + 5, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 0) + 1, 0));
         yCoefficientVectorToPack.set(positionVectorStart + 5, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 1) + 1, 0));
         zCoefficientVectorToPack.set(positionVectorStart + 5, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 2) + 1, 0));

         for (int contactIdx = 0; contactIdx < contactPlaneHelpers.get(i).size(); contactIdx++)
         {
            ContactPlaneHelper contactPlaneHelper = contactPlaneHelpers.get(i).get(contactIdx);
            DMatrixRMaj contactCoefficientMatrix = contactPlaneHelper.getContactWrenchCoefficientMatrix();

            xCoefficientVectorToPack.add(positionVectorStart, 0, contactCoefficientMatrix.get(0, 0));
            yCoefficientVectorToPack.add(positionVectorStart, 0, contactCoefficientMatrix.get(1, 0));
            zCoefficientVectorToPack.add(positionVectorStart, 0, contactCoefficientMatrix.get(2, 0));

            xCoefficientVectorToPack.add(positionVectorStart + 1, 0, contactCoefficientMatrix.get(0, 1));
            yCoefficientVectorToPack.add(positionVectorStart + 1, 0, contactCoefficientMatrix.get(1, 1));
            zCoefficientVectorToPack.add(positionVectorStart + 1, 0, contactCoefficientMatrix.get(2, 1));

            xCoefficientVectorToPack.add(positionVectorStart + 2, 0, contactCoefficientMatrix.get(0, 2));
            yCoefficientVectorToPack.add(positionVectorStart + 2, 0, contactCoefficientMatrix.get(1, 2));
            zCoefficientVectorToPack.add(positionVectorStart + 2, 0, contactCoefficientMatrix.get(2, 2));

            xCoefficientVectorToPack.add(positionVectorStart + 3, 0, contactCoefficientMatrix.get(0, 3));
            yCoefficientVectorToPack.add(positionVectorStart + 3, 0, contactCoefficientMatrix.get(1, 3));
            zCoefficientVectorToPack.add(positionVectorStart + 3, 0, contactCoefficientMatrix.get(2, 3));
         }
         zCoefficientVectorToPack.add(positionVectorStart + 3, 0, -0.5 * gravityZ);
      }
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

   private static void computeVRPBoundaryConditionsFromCoefficients(CoMTrajectorySegment segment,
                                                                    double duration,
                                                                    double omega,
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
         double startPositionElement = segment.getSixthCoefficient().getElement(element) - 2.0 / omega2 * segment.getFourthCoefficient().getElement(element);
         double startVelocityElement = segment.getFifthCoefficient().getElement(element) - 6.0 / omega2 * segment.getThirdCoefficient().getElement(element);
         startPosition.setElement(element, startPositionElement);
         startVelocity.setElement(element, startVelocityElement);

         double endPositionElement =
               segment.getThirdCoefficient().getElement(element) * t3 + segment.getFourthCoefficient().getElement(element) * t2 + startVelocityElement * duration
               + startPositionElement;
         double endVelocityElement =
               3.0 * segment.getThirdCoefficient().getElement(element) * t2 + 2.0 * segment.getFourthCoefficient().getElement(element) * duration
               + startVelocityElement;

         endPosition.setElement(element, endPositionElement);
         endVelocity.setElement(element, endVelocityElement);
      }
   }
}
