package us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.LinearTrajectoryHandler;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class ContinuousTrajectoryHandler extends LinearTrajectoryHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final OrientationTrajectoryCalculator orientationInitializationCalculator;

   private final ContinuousMPCIndexHandler indexHandler;

   final DMatrixRMaj yawCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj pitchCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj rollCoefficientVector = new DMatrixRMaj(0, 1);

   private final FixedFrameOrientation3DBasics desiredBodyOrientation = new FrameQuaternion(worldFrame);
   private final FixedFrameVector3DBasics desiredBodyAngularVelocity = new FrameVector3D(worldFrame);

   public ContinuousTrajectoryHandler(ContinuousMPCIndexHandler indexHandler, double gravityZ, double nominalCoMHeight, YoRegistry registry)
   {
      super(indexHandler, gravityZ, nominalCoMHeight, registry);
      this.indexHandler = indexHandler;

      orientationInitializationCalculator = new OrientationTrajectoryCalculator(registry);
   }

   public void setInitialBodyOrientationState(FrameOrientation3DReadOnly bodyOrientation, FrameVector3DReadOnly angularVelocity)
   {
      orientationInitializationCalculator.setInitialBodyOrientation(bodyOrientation, angularVelocity);
   }

   public void solveForTrajectoryOutsidePreviewWindow(List<ContactPlaneProvider> fullContactSequence)
   {
      super.solveForTrajectoryOutsidePreviewWindow(fullContactSequence);

      orientationInitializationCalculator.solveForTrajectory(fullContactSequence);
   }

   public void extractSolutionForPreviewWindow(DMatrixRMaj solutionCoefficients,
                                               List<ContactPlaneProvider> planningWindow,
                                               List<? extends List<ContactPlaneHelper>> contactPlaneHelpers,
                                               double currentTimeInState)
   {
      super.extractSolutionForPreviewWindow(solutionCoefficients, planningWindow, contactPlaneHelpers, currentTimeInState);

      int numberOfPhases = planningWindow.size();

      yawCoefficientVector.reshape(ContinuousMPCIndexHandler.orientationCoefficientsPerSegment * numberOfPhases, 1);
      pitchCoefficientVector.reshape(ContinuousMPCIndexHandler.orientationCoefficientsPerSegment * numberOfPhases, 1);
      rollCoefficientVector.reshape(ContinuousMPCIndexHandler.orientationCoefficientsPerSegment * numberOfPhases, 1);
      yawCoefficientVector.zero();
      pitchCoefficientVector.zero();
      rollCoefficientVector.zero();

      for (int i = 0; i < numberOfPhases; i++)
      {
         int orientationVectorStart = ContinuousMPCIndexHandler.orientationCoefficientsPerSegment * i;

         for (int orientationIndex = 0; orientationIndex < ContinuousMPCIndexHandler.orientationCoefficientsPerSegment; orientationIndex++)
         {
            yawCoefficientVector.set(orientationVectorStart + orientationIndex,
                                     0,
                                     solutionCoefficients.get(indexHandler.getYawCoefficientsStartIndex(i) + orientationIndex));
            pitchCoefficientVector.set(orientationVectorStart + orientationIndex,
                                       0,
                                       solutionCoefficients.get(indexHandler.getPitchCoefficientsStartIndex(i) + orientationIndex));
            rollCoefficientVector.set(orientationVectorStart + orientationIndex,
                                      0,
                                      solutionCoefficients.get(indexHandler.getRollCoefficientsStartIndex(i) + orientationIndex));
         }
      }
   }

   public void computeOrientation(double timeInPhase,
                                  double omega,
                                  FixedFrameOrientation3DBasics desiredBodyOrientationToPack,
                                  FixedFrameVector3DBasics desiredBodyAngularVelocityToPack)
   {
      boolean success;

      if (isTimeInPlanningWindow(timeInPhase))
         success = computeOrientationInPlanningWindow(omega, timeInPhase, desiredBodyOrientationToPack, desiredBodyAngularVelocityToPack);
      else
         success = false;

      if (!success)
      {
         orientationInitializationCalculator.compute(timeInPhase);
         desiredBodyOrientationToPack.set(orientationInitializationCalculator.getDesiredOrientation());
         desiredBodyAngularVelocityToPack.set(orientationInitializationCalculator.getDesiredAngularVelocity());
      }
   }

   private final FramePoint3D firstOrientationCoefficient = new FramePoint3D();
   private final FramePoint3D secondOrientationCoefficient = new FramePoint3D();
   private final FramePoint3D thirdOrientationCoefficient = new FramePoint3D();
   private final FramePoint3D fourthOrientationCoefficient = new FramePoint3D();
   private final FramePoint3D fifthOrientationCoefficient = new FramePoint3D();
   private final FramePoint3D sixthOrientationCoefficient = new FramePoint3D();


   private void setActiveOrientationCoefficients(int segmentNumber)
   {
      int orientationStartIndex = ContinuousMPCIndexHandler.orientationCoefficientsPerSegment * segmentNumber;
      firstOrientationCoefficient.setX(yawCoefficientVector.get(orientationStartIndex, 0));
      firstOrientationCoefficient.setY(pitchCoefficientVector.get(orientationStartIndex, 0));
      firstOrientationCoefficient.setZ(rollCoefficientVector.get(orientationStartIndex, 0));

      int secondOrientationCoefficientIndex = orientationStartIndex + 1;
      secondOrientationCoefficient.setX(yawCoefficientVector.get(secondOrientationCoefficientIndex, 0));
      secondOrientationCoefficient.setY(pitchCoefficientVector.get(secondOrientationCoefficientIndex, 0));
      secondOrientationCoefficient.setZ(rollCoefficientVector.get(secondOrientationCoefficientIndex, 0));

      int thirdOrientationCoefficientIndex = orientationStartIndex + 2;
      thirdOrientationCoefficient.setX(yawCoefficientVector.get(thirdOrientationCoefficientIndex, 0));
      thirdOrientationCoefficient.setY(pitchCoefficientVector.get(thirdOrientationCoefficientIndex, 0));
      thirdOrientationCoefficient.setZ(rollCoefficientVector.get(thirdOrientationCoefficientIndex, 0));

      int fourthOrientationCoefficientIndex = orientationStartIndex + 3;
      fourthOrientationCoefficient.setX(yawCoefficientVector.get(fourthOrientationCoefficientIndex, 0));
      fourthOrientationCoefficient.setY(pitchCoefficientVector.get(fourthOrientationCoefficientIndex, 0));
      fourthOrientationCoefficient.setZ(rollCoefficientVector.get(fourthOrientationCoefficientIndex, 0));

      if (ContinuousMPCIndexHandler.includeExponentialInOrientation)
      {
         int fifthOrientationCoefficientIndex = orientationStartIndex + 4;
         fifthOrientationCoefficient.setX(yawCoefficientVector.get(fifthOrientationCoefficientIndex, 0));
         fifthOrientationCoefficient.setY(pitchCoefficientVector.get(fifthOrientationCoefficientIndex, 0));
         fifthOrientationCoefficient.setZ(rollCoefficientVector.get(fifthOrientationCoefficientIndex, 0));

         int sixthOrientationCoefficientIndex = orientationStartIndex + 5;
         sixthOrientationCoefficient.setX(yawCoefficientVector.get(sixthOrientationCoefficientIndex, 0));
         sixthOrientationCoefficient.setY(pitchCoefficientVector.get(sixthOrientationCoefficientIndex, 0));
         sixthOrientationCoefficient.setZ(rollCoefficientVector.get(sixthOrientationCoefficientIndex, 0));
      }
   }

   @Override
   protected boolean computeInPlanningWindow(int segmentNumber, double timeInSegment, double omega)
   {
      super.computeInPlanningWindow(segmentNumber, timeInSegment, omega);

      setActiveOrientationCoefficients(segmentNumber);

      CoMMPCTools.constructedDesiredBodyOrientation(desiredBodyOrientation,
                                                    firstOrientationCoefficient,
                                                    secondOrientationCoefficient,
                                                    thirdOrientationCoefficient,
                                                    fourthOrientationCoefficient,
                                                    fifthOrientationCoefficient,
                                                    sixthOrientationCoefficient,
                                                    omega,
                                                    timeInSegment);
      CoMMPCTools.constructedDesiredBodyAngularVelocity(desiredBodyAngularVelocity,
                                                        firstOrientationCoefficient,
                                                        secondOrientationCoefficient,
                                                        thirdOrientationCoefficient,
                                                        fourthOrientationCoefficient,
                                                        fifthOrientationCoefficient,
                                                        sixthOrientationCoefficient,
                                                        omega,
                                                        timeInSegment);

      return true;
   }

   private boolean computeOrientationInPlanningWindow(double omega,
                                                      double timeInPhase,
                                                      FixedFrameOrientation3DBasics desiredBodyOrientation,
                                                      FixedFrameVector3DBasics desiredBodyAngularVelocity)
   {
      int segmentNumber = getPreviewWindowSegmentNumber(timeInPhase);
      if (segmentNumber < 0)
         return false;

      double timeInSegment = getTimeInSegment(segmentNumber, timeInPhase);

      return computeOrientationInPlanningWindow(segmentNumber, omega, timeInSegment, desiredBodyOrientation, desiredBodyAngularVelocity);
   }

   boolean computeOrientationInPlanningWindow(int segmentNumber,
                                              double omega,
                                              double timeInSegment,
                                              FixedFrameOrientation3DBasics desiredBodyOrientationToPack,
                                              FixedFrameVector3DBasics desiredBodyAngularVelocityToPack)
   {
      setActiveOrientationCoefficients(segmentNumber);

      CoMMPCTools.constructedDesiredBodyOrientation(desiredBodyOrientationToPack,
                                                    firstOrientationCoefficient,
                                                    secondOrientationCoefficient,
                                                    thirdOrientationCoefficient,
                                                    fourthOrientationCoefficient,
                                                    fifthOrientationCoefficient,
                                                    sixthOrientationCoefficient,
                                                    omega,
                                                    timeInSegment);
      CoMMPCTools.constructedDesiredBodyAngularVelocity(desiredBodyAngularVelocityToPack,
                                                        firstOrientationCoefficient,
                                                        secondOrientationCoefficient,
                                                        thirdOrientationCoefficient,
                                                        fourthOrientationCoefficient,
                                                        fifthOrientationCoefficient,
                                                        sixthOrientationCoefficient,
                                                        omega,
                                                        timeInSegment);

      return true;
   }

   @Override
   protected void computeOutsideOfPlanningWindow(double time)
   {
      super.computeOutsideOfPlanningWindow(time);

      computeReferenceOrientations(time, desiredBodyOrientation, desiredBodyAngularVelocity);
   }

   public void computeReferenceOrientations(double time,
                                            FixedFrameOrientation3DBasics desiredBodyOrientation,
                                            FixedFrameVector3DBasics desiredBodyAngularVelocity)
   {
      orientationInitializationCalculator.compute(time);

      desiredBodyOrientation.set(orientationInitializationCalculator.getDesiredOrientation());
      desiredBodyAngularVelocity.set(orientationInitializationCalculator.getDesiredAngularVelocity());
   }

   public FrameOrientation3DReadOnly getDesiredBodyOrientation()
   {
      return desiredBodyOrientation;
   }

   public FrameVector3DReadOnly getDesiredBodyAngularVelocity()
   {
      return desiredBodyAngularVelocity;
   }
}
