package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

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

   private double currentTimeInState;

   public LinearTrajectoryHandler(LinearMPCIndexHandler indexHandler, double gravityZ, double nominalCoMHeight, YoRegistry registry)
   {
      this.indexHandler = indexHandler;
      this.gravityZ = Math.abs(gravityZ);

      positionInitializationCalculator = new CoMTrajectoryPlanner(gravityZ, nominalCoMHeight, registry);
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
      if (planningWindow.size() > 0)
         return time < planningWindow.get(planningWindow.size() - 1).getTimeInterval().getEndTime();

      return false;
   }

   private final FramePoint3D firstPositionCoefficient = new FramePoint3D();
   private final FramePoint3D secondPositionCoefficient = new FramePoint3D();
   private final FramePoint3D thirdPositionCoefficient = new FramePoint3D();
   private final FramePoint3D fourthPositionCoefficient = new FramePoint3D();
   private final FramePoint3D fifthPositionCoefficient = new FramePoint3D();
   private final FramePoint3D sixthPositionCoefficient = new FramePoint3D();

   private boolean computeInPlanningWindow(double timeInPhase, double omega)
   {
      int segmentNumber = getPreviewWindowSegmentNumber(timeInPhase);
      if (segmentNumber < 0)
         return false;

      double timeInSegment = getTimeInSegment(segmentNumber, timeInPhase);

      return computeInPlanningWindow(segmentNumber, timeInSegment, omega);
   }

   private void setActivePositionCoefficients(int segmentNumber)
   {
      int positionStartIndex = 6 * segmentNumber;
      firstPositionCoefficient.setX(xCoefficientVector.get(positionStartIndex, 0));
      firstPositionCoefficient.setY(yCoefficientVector.get(positionStartIndex, 0));
      firstPositionCoefficient.setZ(zCoefficientVector.get(positionStartIndex, 0));

      int secondPositionCoefficientIndex = positionStartIndex + 1;
      secondPositionCoefficient.setX(xCoefficientVector.get(secondPositionCoefficientIndex, 0));
      secondPositionCoefficient.setY(yCoefficientVector.get(secondPositionCoefficientIndex, 0));
      secondPositionCoefficient.setZ(zCoefficientVector.get(secondPositionCoefficientIndex, 0));

      int thirdPositionCoefficientIndex = positionStartIndex + 2;
      thirdPositionCoefficient.setX(xCoefficientVector.get(thirdPositionCoefficientIndex, 0));
      thirdPositionCoefficient.setY(yCoefficientVector.get(thirdPositionCoefficientIndex, 0));
      thirdPositionCoefficient.setZ(zCoefficientVector.get(thirdPositionCoefficientIndex, 0));

      int fourthPositionCoefficientIndex = positionStartIndex + 3;
      fourthPositionCoefficient.setX(xCoefficientVector.get(fourthPositionCoefficientIndex, 0));
      fourthPositionCoefficient.setY(yCoefficientVector.get(fourthPositionCoefficientIndex, 0));
      fourthPositionCoefficient.setZ(zCoefficientVector.get(fourthPositionCoefficientIndex, 0));

      int fifthCoefficientIndex = positionStartIndex + 4;
      fifthPositionCoefficient.setX(xCoefficientVector.get(fifthCoefficientIndex, 0));
      fifthPositionCoefficient.setY(yCoefficientVector.get(fifthCoefficientIndex, 0));
      fifthPositionCoefficient.setZ(zCoefficientVector.get(fifthCoefficientIndex, 0));

      int sixthCoefficientIndex = positionStartIndex + 5;
      sixthPositionCoefficient.setX(xCoefficientVector.get(sixthCoefficientIndex, 0));
      sixthPositionCoefficient.setY(yCoefficientVector.get(sixthCoefficientIndex, 0));
      sixthPositionCoefficient.setZ(zCoefficientVector.get(sixthCoefficientIndex, 0));
   }

   protected boolean computeInPlanningWindow(int segmentNumber, double timeInSegment, double omega)
   {
      setActivePositionCoefficients(segmentNumber);

      CoMMPCTools.constructDesiredCoMPosition(desiredCoMPosition, firstPositionCoefficient, secondPositionCoefficient, thirdPositionCoefficient,
                                              fourthPositionCoefficient, fifthPositionCoefficient, sixthPositionCoefficient,
                                              timeInSegment,
                                              omega);
      CoMMPCTools.constructDesiredCoMVelocity(desiredCoMVelocity, firstPositionCoefficient, secondPositionCoefficient, thirdPositionCoefficient,
                                              fourthPositionCoefficient, fifthPositionCoefficient, sixthPositionCoefficient,
                                              timeInSegment,
                                              omega);
      CoMMPCTools.constructDesiredCoMAcceleration(desiredCoMAcceleration, firstPositionCoefficient, secondPositionCoefficient, thirdPositionCoefficient,
                                                  fourthPositionCoefficient, fifthPositionCoefficient, sixthPositionCoefficient,
                                                  timeInSegment,
                                                  omega);

      CoMMPCTools.constructDesiredVRPVelocity(desiredVRPVelocity, firstPositionCoefficient, secondPositionCoefficient, thirdPositionCoefficient,
                                              fourthPositionCoefficient, fifthPositionCoefficient, sixthPositionCoefficient,
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

   private boolean fastComputePositionInPlanningWindow(double timeInPhase, double omega, FixedFramePoint3DBasics desiredCoMPositionToPack)
   {
      int segmentNumber = getPreviewWindowSegmentNumber(timeInPhase);
      if (segmentNumber < 0)
         return false;

      double timeInSegment = getTimeInSegment(segmentNumber, timeInPhase);

      return fastComputePositionInPlanningWindow(segmentNumber, timeInSegment, omega, desiredCoMPositionToPack);
   }

   private boolean fastComputePositionInPlanningWindow(int segmentNumber, double timeInSegment, double omega, FixedFramePoint3DBasics desiredCoMPositionToPack)
   {
      setActivePositionCoefficients(segmentNumber);

      CoMMPCTools.constructDesiredCoMPosition(desiredCoMPositionToPack, firstPositionCoefficient, secondPositionCoefficient, thirdPositionCoefficient,
                                              fourthPositionCoefficient, fifthPositionCoefficient, sixthPositionCoefficient,
                                              timeInSegment,
                                              omega);

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
}
