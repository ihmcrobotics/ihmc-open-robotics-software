package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerIndexHandler;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectorySegment;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.MultipleCoMSegmentTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
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
import us.ihmc.robotics.time.TimeIntervalProvider;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLongTime;

/**
 * This class is meant to handle the trajectory from the MPC module. It includes the trajectory for the full planning window, which is overwritten with the
 * solution for the planning window at the beginning.
 *
 * It assembles all this solution into a continuous multi-segment trajectory for the center of mass, and a list of polynomials for the VRP.
 */
public class LinearMPCTrajectoryHandler
{
   private final CoMTrajectoryPlanner positionInitializationCalculator;

   protected final RecyclingArrayList<ContactPlaneProvider> planningWindowForSolution = new RecyclingArrayList<>(ContactPlaneProvider::new);
   private final RecyclingArrayList<ContactPlaneProvider> fullContactSet = new RecyclingArrayList<>(ContactPlaneProvider::new);

   private final LinearMPCIndexHandler indexHandler;
   private final double gravityZ;

   private final DMatrixRMaj xCoefficientVector = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj yCoefficientVector = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj zCoefficientVector = new DMatrixRMaj(0, 1);

   private final DMatrixRMaj coefficientArray = new DMatrixRMaj(0, 3);

   private final MultipleCoMSegmentTrajectoryGenerator comTrajectory;
   private final RecyclingArrayList<Polynomial3DBasics> vrpTrajectories = new RecyclingArrayList<>(() -> new Polynomial3D(4));

   private final CoMTrajectorySegment comSegmentToAppend = new CoMTrajectorySegment();
   private final ContactSegmentHelper contactSegmentHelper = new ContactSegmentHelper();

   public LinearMPCTrajectoryHandler(LinearMPCIndexHandler indexHandler, double gravityZ, double nominalCoMHeight, YoRegistry registry)
   {
      this.indexHandler = indexHandler;
      this.gravityZ = Math.abs(gravityZ);

      positionInitializationCalculator = new CoMTrajectoryPlanner(gravityZ, nominalCoMHeight, registry);
      comTrajectory = new MultipleCoMSegmentTrajectoryGenerator("desiredCoMTrajectory", registry);
   }

   /**
    * Clears the CoM and VRP solution trajectories
    */
   public void clearTrajectory()
   {
      comTrajectory.clear();
      vrpTrajectories.clear();
      fullContactSet.clear();
   }

   /**
    * Sets the nominal CoM height for the trajectory that is generated outside the preview window
    * @param nominalCoMHeight nominal height
    */
   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      positionInitializationCalculator.setNominalCoMHeight(nominalCoMHeight);
   }

   /**
    * Sets the initial center of mass state, both the position and velocity, which ins needed for computing the CoM trajectory
    * outside the preview window.
    * @param centerOfMassPosition initial CoM position at the start of the calculation
    * @param centerOfMassVelocity initial CoM velocity at the start of the calculation
    */
   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      positionInitializationCalculator.setInitialCenterOfMassState(centerOfMassPosition, centerOfMassVelocity);
   }

   /**
    * Computes the full CoM trajectory for the trajectory starting from the CoM state set by
    * {@link #setInitialCenterOfMassState(FramePoint3DReadOnly, FrameVector3DReadOnly)} and ending with convergence to the final state specified in
    * {@param fullContactSequence}
    * @param fullContactSequence contact sequence to use to calculate the full CoM trajectory.
    */
   public void solveForTrajectoryOutsidePreviewWindow(List<ContactPlaneProvider> fullContactSequence)
   {
      positionInitializationCalculator.solveForTrajectory(fullContactSequence);

      removeInfoOutsidePreviewWindow();
      overwriteTrajectoryOutsidePreviewWindow(positionInitializationCalculator.getOmega());
      overwriteContactsOutsidePreviewWindow(fullContactSequence);
   }

   private final FramePoint3D vrpStartPosition = new FramePoint3D();
   private final FrameVector3D vrpStartVelocity = new FrameVector3D();
   private final FramePoint3D vrpEndPosition = new FramePoint3D();
   private final FrameVector3D vrpEndVelocity = new FrameVector3D();

   /**
    * Extracts the solution from the MPC module into the CoM and VRP trajectories for the preview window
    * @param solutionCoefficients full set of coefficients that go into calculate the motion function.
    * @param planningWindow nominal contact sequence for the preview window used to compute the solution coefficients
    * @param contactPlaneHelpers contact planes that contain the generalized contact vectors for the MPC planning window
    * @param omega time constant for the motion function
    */
   public void extractSolutionForPreviewWindow(DMatrixRMaj solutionCoefficients,
                                               List<ContactPlaneProvider> planningWindow,
                                               List<? extends List<MPCContactPlane>> contactPlaneHelpers,
                                               List<ContactPlaneProvider> fullContactSequence,
                                               double omega)
   {
      int numberOfPhases = planningWindow.size();
      this.planningWindowForSolution.clear();
      for (int i = 0; i < numberOfPhases; i++)
         this.planningWindowForSolution.add().set(planningWindow.get(i));

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

         fullContactSet.add().set(fullContactSequence.get(i));
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

      overwriteTrajectoryOutsidePreviewWindow(omega);
      overwriteContactsOutsidePreviewWindow(fullContactSequence);

      if (vrpTrajectories.size() != fullContactSet.size())
         throw new RuntimeException("Somehow these didn't match up.");
   }

   private void removeInfoOutsidePreviewWindow()
   {
      if (planningWindowForSolution.size() > 0 && fullContactSet.size() > 0)
      {
         while (fullContactSet.getLast().getTimeInterval().getEndTime() > planningWindowForSolution.getLast().getTimeInterval().getEndTime())
         {
            int lastIndx = fullContactSet.size() - 1;
            fullContactSet.remove(lastIndx);
            vrpTrajectories.remove(lastIndx);
            comTrajectory.removeSegment(lastIndx);
         }
      }
   }

   private void overwriteTrajectoryOutsidePreviewWindow(double omega)
   {
      MultipleCoMSegmentTrajectoryGenerator comTrajectoryOutsideWindow = positionInitializationCalculator.getCoMTrajectory();
      List<Polynomial3DReadOnly> vrpTrajectoryOutsideWindow = positionInitializationCalculator.getVRPTrajectories();

      boolean hasTrajectoryAlready = comTrajectory.getCurrentNumberOfSegments() > 0;
      double existingEndTime = hasTrajectoryAlready ? comTrajectory.getEndTime() : 0.0;
      if (existingEndTime >= comTrajectoryOutsideWindow.getEndTime())
         return;

      int segmentIndexToAdd = getSegmentIndexContainingTime(existingEndTime + 1e-5, positionInitializationCalculator.getCoMTrajectory().getSegments());
      if (segmentIndexToAdd == -1)
         return;

      CoMTrajectorySegment nextCoMSegment = comTrajectoryOutsideWindow.getSegment(segmentIndexToAdd);
      CoMTrajectorySegment lastCoMSegmentOfPreview = hasTrajectoryAlready ? comTrajectory.getSegment(comTrajectory.getCurrentNumberOfSegments() - 1) : null;

      // end of the preview starts perfectly with the next segment
      if (lastCoMSegmentOfPreview == null || TimeIntervalTools.areTimeIntervalsConsecutive(lastCoMSegmentOfPreview, nextCoMSegment))
      {
         comTrajectory.appendSegment(nextCoMSegment);
         vrpTrajectories.add().set(vrpTrajectoryOutsideWindow.get(segmentIndexToAdd));
      }
      else
      { // end of the preview is in one of the next segments, so we need to prune it.
         double durationToRemove = lastCoMSegmentOfPreview.getTimeInterval().getEndTime() - nextCoMSegment.getTimeInterval().getStartTime();
         comSegmentToAppend.set(nextCoMSegment);
         comSegmentToAppend.shiftStartOfSegment(durationToRemove);
         comTrajectory.appendSegment(comSegmentToAppend);

         // TODO make this cleaner
         double duration = Math.min(comSegmentToAppend.getTimeInterval().getDuration(), sufficientlyLongTime);
         computeVRPBoundaryConditionsFromCoefficients(comSegmentToAppend,
                                                      duration,
                                                      omega,
                                                      vrpStartPosition,
                                                      vrpStartVelocity,
                                                      vrpEndPosition,
                                                      vrpEndVelocity);
         Polynomial3DBasics vrpTrajectory = vrpTrajectories.add();
         vrpTrajectory.setCubic(0.0, duration, vrpStartPosition, vrpStartVelocity, vrpEndPosition, vrpEndVelocity);
         vrpTrajectory.getTimeInterval().setInterval(0.0, comSegmentToAppend.getTimeInterval().getDuration());
      }


      segmentIndexToAdd++;
      for (;segmentIndexToAdd < comTrajectoryOutsideWindow.getCurrentNumberOfSegments(); segmentIndexToAdd++)
      {
         comTrajectory.appendSegment(comTrajectoryOutsideWindow.getSegment(segmentIndexToAdd));
         vrpTrajectories.add().set(vrpTrajectoryOutsideWindow.get(segmentIndexToAdd));
      }

      comTrajectory.initialize();
   }

   private void overwriteContactsOutsidePreviewWindow(List<ContactPlaneProvider> contactsToUse)
   {
      boolean hasContactsAlready = fullContactSet.size() > 0;
      double existingEndTime = hasContactsAlready ? fullContactSet.getLast().getTimeInterval().getEndTime() : 0.0;

      if (hasContactsAlready && existingEndTime >= contactsToUse.get(contactsToUse.size() - 1).getTimeInterval().getEndTime())
         return;

      int segmentIndexToAdd = getSegmentIndexContainingTime(existingEndTime + 1e-5, contactsToUse);
      if (segmentIndexToAdd == -1)
         return;

      ContactPlaneProvider nextContact = contactsToUse.get(segmentIndexToAdd);
      ContactPlaneProvider lastContactOfPreview = hasContactsAlready ? fullContactSet.get(fullContactSet.size() - 1) : null;

      // end of the preview starts perfectly with the next segment
      if (lastContactOfPreview == null || TimeIntervalTools.areTimeIntervalsConsecutive(lastContactOfPreview, nextContact))
      {
         fullContactSet.add().set(nextContact);
      }
      else
      { // end of the preview is in one of the next segments, so we need to prune it.
         ContactPlaneProvider contactPlaneToAppend = fullContactSet.add();
         contactPlaneToAppend.set(nextContact);
         contactSegmentHelper.cropInitialSegmentLength(contactPlaneToAppend, lastContactOfPreview.getTimeInterval().getEndTime());
      }

      segmentIndexToAdd++;
      for (;segmentIndexToAdd < contactsToUse.size(); segmentIndexToAdd++)
         fullContactSet.add().set(contactsToUse.get(segmentIndexToAdd));
   }

   private static int getSegmentIndexContainingTime(double time, List<? extends TimeIntervalProvider> segments)
   {
      for (int i = 0; i < segments.size(); i++)
      {
         TimeIntervalProvider segment = segments.get(i);
         if (segment.getTimeInterval().intervalContains(time))
            return i;
      }

      return -1;
   }

   public void compute(double timeInPhase)
   {
      comTrajectory.compute(timeInPhase);
   }

   public void computeOutsidePreview(double timeInPhase)
   {
      positionInitializationCalculator.compute(timeInPhase);
   }

   public FramePoint3DReadOnly getDesiredCoMPositionOutsidePreview()
   {
      return positionInitializationCalculator.getDesiredCoMPosition();
   }

   public FrameVector3DReadOnly getDesiredCoMVelocityOutsidePreview()
   {
      return positionInitializationCalculator.getDesiredCoMVelocity();
   }

   public FramePoint3DReadOnly getDesiredDCMPositionOutsidePreview()
   {
      return positionInitializationCalculator.getDesiredDCMPosition();
   }

   public FramePoint3DReadOnly getDesiredVRPPositionOutsidePreview()
   {
      return positionInitializationCalculator.getDesiredVRPPosition();
   }

   public MultipleCoMSegmentTrajectoryGenerator getComTrajectory()
   {
      return comTrajectory;
   }

   public List<? extends Polynomial3DReadOnly> getVrpTrajectories()
   {
      return vrpTrajectories;
   }

   public List<ContactPlaneProvider> getFullPlanningSequence()
   {
      return fullContactSet;
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

   public DMatrixRMaj getXCoefficientVector()
   {
      return xCoefficientVector;
   }

   public DMatrixRMaj getYCoefficientVector()
   {
      return yCoefficientVector;
   }

   public DMatrixRMaj getZCoefficientVector()
   {
      return zCoefficientVector;
   }

   private void computeCoMSegmentCoefficients(DMatrixRMaj solutionCoefficients,
                                              List<? extends List<MPCContactPlane>> contactPlaneHelpers,
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
            MPCContactPlane contactPlaneHelper = contactPlaneHelpers.get(sequence).get(contact);
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
            MPCContactPlane contactPlaneHelper = contactPlaneHelpers.get(i).get(contactIdx);
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
