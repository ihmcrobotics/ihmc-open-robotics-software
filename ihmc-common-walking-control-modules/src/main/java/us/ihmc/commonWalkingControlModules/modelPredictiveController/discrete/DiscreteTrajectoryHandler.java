package us.ihmc.commonWalkingControlModules.modelPredictiveController.discrete;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous.ContinuousMPCIndexHandler;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class DiscreteTrajectoryHandler extends LinearTrajectoryHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final OrientationTrajectoryCalculator orientationInitializationCalculator;

   private final ContinuousMPCIndexHandler indexHandler;

   final DMatrixRMaj yawCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj pitchCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj rollCoefficientVector = new DMatrixRMaj(0, 1);

   private final FixedFrameOrientation3DBasics desiredBodyOrientation = new FrameQuaternion(worldFrame);
   private final FixedFrameVector3DBasics desiredBodyAngularVelocity = new FrameVector3D(worldFrame);

   public DiscreteTrajectoryHandler(ContinuousMPCIndexHandler indexHandler, double gravityZ, double nominalCoMHeight, YoRegistry registry)
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



   private boolean computeOrientationInPlanningWindow(double omega,
                                                      double timeInPhase,
                                                      FixedFrameOrientation3DBasics desiredBodyOrientation,
                                                      FixedFrameVector3DBasics desiredBodyAngularVelocity)
   {
      orientationInitializationCalculator.compute(timeInPhase);
      desiredBodyOrientation.set(orientationInitializationCalculator.getDesiredOrientation());
      desiredBodyAngularVelocity.set(orientationInitializationCalculator.getDesiredAngularVelocity());

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
