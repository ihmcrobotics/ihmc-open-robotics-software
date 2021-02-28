package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.SE3MPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.SE3MPCQPSolver;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.OrientationMPCTrajectoryHandler;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public abstract class SE3ModelPredictiveController extends EuclideanModelPredictiveController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final double gravityZ;
   protected final double mass;

   protected final SE3MPCIndexHandler indexHandler;

   private final FrameOrientation3DBasics desiredBodyOrientation = new FrameQuaternion(worldFrame);
   private final FrameOrientation3DBasics desiredBodyOrientationFeedForward = new FrameQuaternion(worldFrame);

   private final FrameVector3DBasics desiredBodyAngularVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3DBasics desiredBodyAngularVelocityFeedForward = new FrameVector3D(worldFrame);

   protected final YoFrameQuaternion currentBodyOrientation = new YoFrameQuaternion("currentBodyOrientation", worldFrame, registry);
   protected final YoFrameVector3D currentBodyAngularVelocity = new YoFrameVector3D("currentBodyAngularVelocity", worldFrame, registry);

   private final YoFrameQuaternion orientationAtEndOfWindow = new YoFrameQuaternion("orientationAtEndOfWindow", worldFrame, registry);
   private final YoFrameVector3D angularVelocityAtEndOfWindow = new YoFrameVector3D("angularVelocityAtEndOfWindow", worldFrame, registry);

   private final YoDouble orientationPreviewWindowDuration = new YoDouble("orientationPreviewWindowDuration", registry);

   protected final YoVector3D currentBodyAxisAngleError = new YoVector3D("currentBodyAxisAngleError", registry);

   final OrientationMPCTrajectoryHandler orientationTrajectoryHandler;

   final SE3MPCQPSolver qpSolver;

   protected final Matrix3DReadOnly momentOfInertia;

   public SE3ModelPredictiveController(SE3MPCIndexHandler indexHandler,
                                       OrientationMPCTrajectoryHandler orientationTrajectoryHandler,
                                       Matrix3DReadOnly momentOfInertia,
                                       double gravityZ, double nominalCoMHeight, double mass, double dt,
                                       YoRegistry parentRegistry)
   {
      super(indexHandler, gravityZ, nominalCoMHeight, parentRegistry);

      this.indexHandler = indexHandler;
      this.gravityZ = Math.abs(gravityZ);
      this.mass = mass;
      this.orientationTrajectoryHandler = orientationTrajectoryHandler;

      registry.addChild(orientationTrajectoryHandler.getRegistry());

      this.momentOfInertia = momentOfInertia;

      orientationPreviewWindowDuration.set(0.25);

      qpSolver = new SE3MPCQPSolver(indexHandler, dt, gravityZ, mass, registry);
      qpSolver.setFirstOrientationVariableRegularization(1e-1);
      qpSolver.setSecondOrientationVariableRegularization(1e-10);

      parentRegistry.addChild(registry);
   }


   @Override
   protected void initializeIndexHandler()
   {
      List<ContactPlaneProvider> planningWindow = previewWindowCalculator.getPlanningWindow();
      indexHandler.initialize(planningWindow, orientationPreviewWindowDuration.getDoubleValue());
   }

   @Override
   protected void solveForTrajectoryOutsidePreviewWindow(List<ContactPlaneProvider> contactSequence)
   {
      super.solveForTrajectoryOutsidePreviewWindow(contactSequence);

      orientationTrajectoryHandler.solveForTrajectoryOutsidePreviewWindow(contactSequence);
      orientationTrajectoryHandler.computeDesiredTrajectory(currentTimeInState.getDoubleValue());
      orientationTrajectoryHandler.computeOutsidePreview(orientationPreviewWindowDuration.getDoubleValue() + currentTimeInState.getDoubleValue());
   }


   @Override
   protected void setTerminalConditions()
   {
      super.setTerminalConditions();

      orientationAtEndOfWindow.set(orientationTrajectoryHandler.getDesiredBodyOrientation());
      angularVelocityAtEndOfWindow.set(orientationTrajectoryHandler.getDesiredAngularVelocity());
   }

   @Override
   protected void extractSolution(DMatrixRMaj solutionCoefficients)
   {
      super.extractSolution(solutionCoefficients);

      orientationTrajectoryHandler.extractSolutionForPreviewWindow(solutionCoefficients, linearTrajectoryHandler.getComTrajectory(), currentTimeInState.getDoubleValue(), orientationPreviewWindowDuration.getDoubleValue());
   }

   @Override
   protected void computeObjectives(List<ContactPlaneProvider> contactSequence)
   {
      super.computeObjectives(contactSequence);

      computeOrientationObjectives();
   }

   protected abstract void computeOrientationObjectives();

   @Override
   protected void resetActiveSet()
   {
      qpSolver.notifyResetActiveSet();
      qpSolver.resetRateRegularization();
   }

   @Override
   protected DMatrixRMaj solveQP()
   {
      qpSolver.initialize();
      qpSolver.submitMPCCommandList(mpcCommands);
      if (!qpSolver.solve())
      {
         LogTools.info("Failed to find solution");
         return null;
      }

      return qpSolver.getSolution();
   }

   @Override
   public void compute(double timeInPhase,
                       FixedFramePoint3DBasics comPositionToPack,
                       FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack,
                       FixedFramePoint3DBasics dcmPositionToPack,
                       FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack,
                       FixedFrameVector3DBasics vrpVelocityToPack,
                       FixedFramePoint3DBasics ecmpPositionToPack)
   {
      linearTrajectoryHandler.compute(timeInPhase);
      orientationTrajectoryHandler.compute(timeInPhase);

      comPositionToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredCoMPosition());
      comVelocityToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredCoMVelocity());
      comAccelerationToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredCoMAcceleration());
      dcmPositionToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredDCMPosition());
      dcmVelocityToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredDCMVelocity());
      vrpPositionToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredVRPPosition());
      vrpVelocityToPack.setMatchingFrame(linearTrajectoryHandler.getDesiredVRPVelocity());

      desiredBodyOrientation.setMatchingFrame(orientationTrajectoryHandler.getDesiredBodyOrientation());
      desiredBodyOrientationFeedForward.setMatchingFrame(orientationTrajectoryHandler.getDesiredBodyOrientationOutsidePreview());

      desiredBodyAngularVelocity.setMatchingFrame(orientationTrajectoryHandler.getDesiredAngularVelocity());
      desiredBodyAngularVelocityFeedForward.setMatchingFrame(orientationTrajectoryHandler.getDesiredBodyVelocityOutsidePreview());

      ecmpPositionToPack.setMatchingFrame(vrpPositionToPack);
      double nominalHeight = gravityZ / MathTools.square(omega.getValue());
      ecmpPositionToPack.set(desiredVRPPosition);
      ecmpPositionToPack.subZ(nominalHeight);
   }

   public void setInitialBodyOrientationState(FrameOrientation3DReadOnly bodyOrientation, FrameVector3DReadOnly bodyAngularVelocity)
   {
      orientationTrajectoryHandler.setInitialBodyOrientationState(bodyOrientation, bodyAngularVelocity);
   }

   public void setCurrentState(FramePoint3DReadOnly centerOfMassPosition,
                               FrameVector3DReadOnly centerOfMassVelocity,
                               FrameOrientation3DReadOnly bodyOrientation,
                               FrameVector3DReadOnly bodyAngularVelocity,
                               double timeInState)
   {
      setCurrentCenterOfMassState(centerOfMassPosition, centerOfMassVelocity, timeInState);
      this.currentBodyOrientation.setMatchingFrame(bodyOrientation);
      this.currentBodyAngularVelocity.setMatchingFrame(bodyAngularVelocity);
   }



   public FrameOrientation3DReadOnly getDesiredBodyOrientationSolution()
   {
      return desiredBodyOrientation;
   }

   public FrameOrientation3DReadOnly getDesiredFeedForwardBodyOrientation()
   {
      return desiredBodyOrientationFeedForward;
   }

   public FrameVector3DReadOnly getDesiredBodyAngularVelocitySolution()
   {
      return desiredBodyAngularVelocity;
   }

   public FrameVector3DReadOnly getDesiredFeedForwardBodyAngularVelocity()
   {
      return desiredBodyAngularVelocityFeedForward;
   }
}
