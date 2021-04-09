package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DiscreteAngularVelocityOrientationCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommandList;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.ImplicitOrientationMPCTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.OrientationMPCTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.tools.MPCAngleTools;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.SE3MPCTrajectoryViewer;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.math.trajectories.FixedFramePolynomialEstimator3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class ImplicitSE3ModelPredictiveController extends EuclideanModelPredictiveController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final double gravityZ;
   protected final double mass;

   protected final ImplicitSE3MPCIndexHandler indexHandler;

   private final FrameOrientation3DBasics desiredBodyOrientation = new FrameQuaternion(worldFrame);
   private final FrameOrientation3DBasics desiredBodyOrientationFeedForward = new FrameQuaternion(worldFrame);

   private final FrameVector3DBasics desiredBodyAngularVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3DBasics desiredBodyAngularVelocityFeedForward = new FrameVector3D(worldFrame);

   private final FrameVector3DBasics desiredBodyAngularAcceleration = new FrameVector3D(worldFrame);

   private final WrenchBasics desiredWrench = new Wrench(worldFrame, worldFrame);

   protected final YoFrameQuaternion currentBodyOrientation = new YoFrameQuaternion("currentBodyOrientation", worldFrame, registry);
   protected final YoFrameVector3D currentBodyAngularVelocity = new YoFrameVector3D("currentBodyAngularVelocity", worldFrame, registry);

   private final YoFrameQuaternion orientationAtEndOfWindow = new YoFrameQuaternion("orientationAtEndOfWindow", worldFrame, registry);
   private final YoFrameVector3D angularVelocityAtEndOfWindow = new YoFrameVector3D("angularVelocityAtEndOfWindow", worldFrame, registry);

   private final YoDouble orientationPreviewWindowDuration = new YoDouble("orientationPreviewWindowDuration", registry);

   protected final YoVector3D currentBodyAxisAngleError = new YoVector3D("currentBodyAxisAngleError", registry);

   private final OrientationTrajectoryConstructor orientationTrajectoryConstructor;
   final ImplicitOrientationMPCTrajectoryHandler orientationTrajectoryHandler;
   private SE3MPCTrajectoryViewer trajectoryViewer = null;

   final ImplicitSE3MPCQPSolver qpSolver;

   protected final Matrix3DReadOnly momentOfInertia;

   private final MPCAngleTools angleTools = new MPCAngleTools();

   protected final YoVector3D currentBodyAngularVelocityError = new YoVector3D("currentBodyAngularVelocityError", registry);

   public ImplicitSE3ModelPredictiveController(Matrix3DReadOnly momentOfInertia,
                                               double gravityZ, double nominalCoMHeight, double mass, double dt,
                                               YoRegistry parentRegistry)
   {
      this(new ImplicitSE3MPCIndexHandler(numberOfBasisVectorsPerContactPoint),
           momentOfInertia,
           gravityZ,
           nominalCoMHeight,
           mass,
           dt,
           parentRegistry);
   }

   public ImplicitSE3ModelPredictiveController(ImplicitSE3MPCIndexHandler indexHandler,
                                               Matrix3DReadOnly momentOfInertia,
                                               double gravityZ, double nominalCoMHeight, double mass, double dt,
                                               YoRegistry parentRegistry)
   {
      super(indexHandler, mass, gravityZ, nominalCoMHeight, parentRegistry);

      this.indexHandler = indexHandler;
      this.gravityZ = Math.abs(gravityZ);
      this.mass = mass;
      orientationTrajectoryConstructor = new OrientationTrajectoryConstructor(indexHandler, mass, gravityZ);
      this.orientationTrajectoryHandler = new ImplicitOrientationMPCTrajectoryHandler(indexHandler, orientationTrajectoryConstructor);

      registry.addChild(orientationTrajectoryHandler.getRegistry());

      this.momentOfInertia = momentOfInertia;

      orientationPreviewWindowDuration.set(0.25);

      qpSolver = new ImplicitSE3MPCQPSolver(indexHandler, dt, gravityZ, mass, registry);
      qpSolver.setMaxNumberOfIterations(1000);

      qpSolver.setFirstOrientationVariableRegularization(1e-2);
      qpSolver.setSecondOrientationVariableRegularization(1e-2);

      parentRegistry.addChild(registry);
   }


   @Override
   protected void initializeIndexHandler()
   {
      List<ContactPlaneProvider> planningWindow = previewWindowCalculator.getPlanningWindow();
      indexHandler.initialize(planningWindow);
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

      orientationTrajectoryHandler.extractSolutionForPreviewWindow(solutionCoefficients, currentTimeInState.getDoubleValue(), orientationPreviewWindowDuration.getDoubleValue());
   }

   @Override
   protected void computeObjectives(List<ContactPlaneProvider> contactSequence)
   {
      super.computeObjectives(contactSequence);

      computeOrientationObjectives();
   }

   private final DMatrixRMaj initialError = new DMatrixRMaj(6, 1);

   private void computeOrientationObjectives()
   {
      linearTrajectoryHandler.compute(currentTimeInState.getDoubleValue());
      FrameOrientation3DReadOnly desiredOrientation = orientationTrajectoryHandler.getDesiredBodyOrientation();


      angleTools.computeRotationError(desiredOrientation, currentBodyOrientation, currentBodyAxisAngleError);
      currentBodyAxisAngleError.get(initialError);

      currentBodyAngularVelocityError.sub(currentBodyAngularVelocity, orientationTrajectoryHandler.getDesiredAngularVelocity());
      desiredOrientation.inverseTransform(currentBodyAngularVelocityError);
      currentBodyAngularVelocityError.get(3, initialError);


      orientationTrajectoryConstructor.compute(previewWindowCalculator.getPlanningWindow(),
                                               momentOfInertia,
                                               linearTrajectoryHandler,
                                               orientationTrajectoryHandler,
                                               contactPlaneHelperPool,
                                               initialError,
                                               omega.getValue());
      for (int i = 0; i < orientationTrajectoryConstructor.getOrientationTrajectoryCommands().size(); i++)
         mpcCommands.addCommand(orientationTrajectoryConstructor.getOrientationTrajectoryCommands().get(i));
   }


   @Override
   public void setupCoMTrajectoryViewer(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      trajectoryViewer = new SE3MPCTrajectoryViewer(registry, yoGraphicsListRegistry);
   }

   protected void updateCoMTrajectoryViewer()
   {
      if (trajectoryViewer != null)
         trajectoryViewer.compute(this, currentTimeInState.getDoubleValue());
   }

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
      wrenchTrajectoryHandler.compute(timeInPhase);
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

      desiredBodyAngularAcceleration.setMatchingFrame(orientationTrajectoryHandler.getDesiredAngularAcceleration());

      desiredWrench.setMatchingFrame(wrenchTrajectoryHandler.getDesiredWrench());

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

   public void setInternalAngularMomentumTrajectory(MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomialEstimator3D> internalAngularMomentumTrajectory)
   {
      this.orientationTrajectoryHandler.setInternalAngularMomentumTrajectory(internalAngularMomentumTrajectory);
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

   public FrameVector3DReadOnly getDesiredBodyAngularAccelerationSolution()
   {
      return desiredBodyAngularAcceleration;
   }

   public WrenchReadOnly getDesiredWrench()
   {
      return desiredWrench;
   }
}
