package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final CoMTrajectoryPlanner initializationCalculator;

   private final List<ContactPlaneProvider> planningWindow = new ArrayList<>();

   private final MPCIndexHandler indexHandler;
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

   private double currentTimeInState;

   public TrajectoryHandler(MPCIndexHandler indexHandler, double gravityZ, double nominalCoMHeight, YoRegistry registry)
   {
      this.indexHandler = indexHandler;
      this.gravityZ = gravityZ;

      initializationCalculator = new CoMTrajectoryPlanner(gravityZ, nominalCoMHeight, registry);
   }

   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      initializationCalculator.setNominalCoMHeight(nominalCoMHeight);
   }

   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      initializationCalculator.setInitialCenterOfMassState(centerOfMassPosition, centerOfMassVelocity);
   }

   public void solveForTrajectoryOutsidePreviewWindow(List<ContactPlaneProvider> fullContactSequence)
   {
      initializationCalculator.solveForTrajectory(fullContactSequence);
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
         int vectorStart = 6 * i;

         xCoefficientVector.set(vectorStart + 4, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 0), 0));
         yCoefficientVector.set(vectorStart + 4, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 1), 0));
         zCoefficientVector.set(vectorStart + 4, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 2), 0));

         xCoefficientVector.set(vectorStart + 5, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 0) + 1, 0));
         yCoefficientVector.set(vectorStart + 5, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 1) + 1, 0));
         zCoefficientVector.set(vectorStart + 5, 0, solutionCoefficients.get(indexHandler.getComCoefficientStartIndex(i, 2) + 1, 0));

         for (int contactIdx = 0; contactIdx < contactPlaneHelpers.get(i).size(); contactIdx++)
         {
            ContactPlaneHelper contactPlaneHelper = contactPlaneHelpers.get(i).get(contactIdx);
            DMatrixRMaj contactCoefficientMatrix = contactPlaneHelper.getContactWrenchCoefficientMatrix();

            xCoefficientVector.add(vectorStart, 0, contactCoefficientMatrix.get(0, 0));
            yCoefficientVector.add(vectorStart, 0, contactCoefficientMatrix.get(1, 0));
            zCoefficientVector.add(vectorStart, 0, contactCoefficientMatrix.get(2, 0));

            xCoefficientVector.add(vectorStart + 1, 0, contactCoefficientMatrix.get(0, 1));
            yCoefficientVector.add(vectorStart + 1, 0, contactCoefficientMatrix.get(1, 1));
            zCoefficientVector.add(vectorStart + 1, 0, contactCoefficientMatrix.get(2, 1));

            xCoefficientVector.add(vectorStart + 2, 0, contactCoefficientMatrix.get(0, 2));
            yCoefficientVector.add(vectorStart + 2, 0, contactCoefficientMatrix.get(1, 2));
            zCoefficientVector.add(vectorStart + 2, 0, contactCoefficientMatrix.get(2, 2));

            xCoefficientVector.add(vectorStart + 3, 0, contactCoefficientMatrix.get(0, 3));
            yCoefficientVector.add(vectorStart + 3, 0, contactCoefficientMatrix.get(1, 3));
            zCoefficientVector.add(vectorStart + 3, 0, contactCoefficientMatrix.get(2, 3));
         }

         zCoefficientVector.add(vectorStart + 3, 0, -0.5 * gravityZ);
      }
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

   private boolean isTimeInPlanningWindow(double time)
   {
      if (planningWindow.size() > 0)
         return time < planningWindow.get(planningWindow.size() - 1).getTimeInterval().getEndTime();

      return false;
   }

   private final FramePoint3D firstCoefficient = new FramePoint3D();
   private final FramePoint3D secondCoefficient = new FramePoint3D();
   private final FramePoint3D thirdCoefficient = new FramePoint3D();
   private final FramePoint3D fourthCoefficient = new FramePoint3D();
   private final FramePoint3D fifthCoefficient = new FramePoint3D();
   private final FramePoint3D sixthCoefficient = new FramePoint3D();

   private boolean computeInPlanningWindow(double timeInPhase, double omega)
   {
      int segmentNumber = getPreviewWindowSegmentNumber(timeInPhase);
      if (segmentNumber < 0)
         return false;

      return computeInPlanningWindow(segmentNumber, timeInPhase, omega);
   }

   boolean computeInPlanningWindow(int segmentNumber, double timeInPhase, double omega)
   {
      double timeInSegment = getTimeInSegment(segmentNumber, timeInPhase);

      int startIndex = 6 * segmentNumber;
      firstCoefficient.setX(xCoefficientVector.get(startIndex, 0));
      firstCoefficient.setY(yCoefficientVector.get(startIndex, 0));
      firstCoefficient.setZ(zCoefficientVector.get(startIndex, 0));

      int secondCoefficientIndex = startIndex + 1;
      secondCoefficient.setX(xCoefficientVector.get(secondCoefficientIndex, 0));
      secondCoefficient.setY(yCoefficientVector.get(secondCoefficientIndex, 0));
      secondCoefficient.setZ(zCoefficientVector.get(secondCoefficientIndex, 0));

      int thirdCoefficientIndex = startIndex + 2;
      thirdCoefficient.setX(xCoefficientVector.get(thirdCoefficientIndex, 0));
      thirdCoefficient.setY(yCoefficientVector.get(thirdCoefficientIndex, 0));
      thirdCoefficient.setZ(zCoefficientVector.get(thirdCoefficientIndex, 0));

      int fourthCoefficientIndex = startIndex + 3;
      fourthCoefficient.setX(xCoefficientVector.get(fourthCoefficientIndex, 0));
      fourthCoefficient.setY(yCoefficientVector.get(fourthCoefficientIndex, 0));
      fourthCoefficient.setZ(zCoefficientVector.get(fourthCoefficientIndex, 0));

      int fifthCoefficientIndex = startIndex + 4;
      fifthCoefficient.setX(xCoefficientVector.get(fifthCoefficientIndex, 0));
      fifthCoefficient.setY(yCoefficientVector.get(fifthCoefficientIndex, 0));
      fifthCoefficient.setZ(zCoefficientVector.get(fifthCoefficientIndex, 0));

      int sixthCoefficientIndex = startIndex + 5;
      sixthCoefficient.setX(xCoefficientVector.get(sixthCoefficientIndex, 0));
      sixthCoefficient.setY(yCoefficientVector.get(sixthCoefficientIndex, 0));
      sixthCoefficient.setZ(zCoefficientVector.get(sixthCoefficientIndex, 0));

      CoMMPCTools.constructDesiredCoMPosition(desiredCoMPosition,
                                              firstCoefficient,
                                              secondCoefficient,
                                              thirdCoefficient,
                                              fourthCoefficient,
                                              fifthCoefficient,
                                              sixthCoefficient,
                                              timeInSegment,
                                              omega);
      CoMMPCTools.constructDesiredCoMVelocity(desiredCoMVelocity,
                                              firstCoefficient,
                                              secondCoefficient,
                                              thirdCoefficient,
                                              fourthCoefficient,
                                              fifthCoefficient,
                                              sixthCoefficient,
                                              timeInSegment,
                                              omega);
      CoMMPCTools.constructDesiredCoMAcceleration(desiredCoMAcceleration,
                                                  firstCoefficient,
                                                  secondCoefficient,
                                                  thirdCoefficient,
                                                  fourthCoefficient,
                                                  fifthCoefficient,
                                                  sixthCoefficient,
                                                  timeInSegment,
                                                  omega);

      CoMMPCTools.constructDesiredVRPVelocity(desiredVRPVelocity,
                                              firstCoefficient,
                                              secondCoefficient,
                                              thirdCoefficient,
                                              fourthCoefficient,
                                              fifthCoefficient,
                                              sixthCoefficient,
                                              timeInSegment,
                                              omega);

      CapturePointTools.computeCapturePointPosition(desiredCoMPosition, desiredCoMVelocity, omega, desiredDCMPosition);
      CapturePointTools.computeCapturePointVelocity(desiredCoMVelocity, desiredCoMAcceleration, omega, desiredDCMVelocity);
      CapturePointTools.computeCentroidalMomentumPivot(desiredDCMPosition, desiredDCMVelocity, omega, desiredVRPPosition);

      double nominalHeight = gravityZ / MathTools.square(omega);
      desiredECMPPosition.set(desiredVRPPosition);
      desiredECMPPosition.subZ(nominalHeight);

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

   private void computeOutsideOfPlanningWindow(double time)
   {
      int segmentId = initializationCalculator.getSegmentNumber(time);
      computeOutsideOfPlanningWindow(segmentId, time);
   }

   void computeOutsideOfPlanningWindow(int segmentId, double time)
   {
      initializationCalculator.compute(segmentId, time);

      desiredCoMPosition.set(initializationCalculator.getDesiredCoMPosition());
      desiredCoMVelocity.set(initializationCalculator.getDesiredCoMVelocity());
      desiredCoMAcceleration.set(initializationCalculator.getDesiredCoMAcceleration());

      desiredVRPPosition.set(initializationCalculator.getDesiredVRPPosition());
      desiredVRPVelocity.set(initializationCalculator.getDesiredVRPVelocity());

      desiredDCMPosition.set(initializationCalculator.getDesiredDCMPosition());
      desiredDCMVelocity.set(initializationCalculator.getDesiredDCMVelocity());

      desiredECMPPosition.set(initializationCalculator.getDesiredECMPPosition());
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
}
