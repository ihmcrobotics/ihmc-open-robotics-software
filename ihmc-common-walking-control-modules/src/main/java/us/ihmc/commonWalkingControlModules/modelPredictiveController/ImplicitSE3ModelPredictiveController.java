package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationContinuityCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationValueCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.ImplicitOrientationMPCTrajectoryHandler;
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
   private static final double defaultOrientationAngleTrackingWeight = 1e-2;
   private static final double defaultOrientationVelocityTrackingWeight = 1e-6;

   private static final double initialOrientationWeight = 1e3;
   private static final double finalOrientationWeight = 1e-2;

   private final double gravityZ;
   protected final double mass;

   protected final ImplicitSE3MPCIndexHandler indexHandler;

   private final FrameOrientation3DBasics desiredBodyOrientation = new FrameQuaternion(worldFrame);
   private final FrameOrientation3DBasics referenceBodyOrientation = new FrameQuaternion(worldFrame);

   private final FrameVector3DBasics desiredBodyAngularVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3DBasics referenceBodyAngularVelocity = new FrameVector3D(worldFrame);

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

   private final YoDouble orientationAngleTrackingWeight = new YoDouble("orientationAngleTrackingWeight", registry);
   private final YoDouble orientationVelocityTrackingWeight = new YoDouble("orientationVelocityTrackingWeight", registry);

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
      orientationAngleTrackingWeight.set(defaultOrientationAngleTrackingWeight);
      orientationVelocityTrackingWeight.set(defaultOrientationVelocityTrackingWeight);
      orientationTrajectoryConstructor = new OrientationTrajectoryConstructor(indexHandler,
                                                                              orientationAngleTrackingWeight,
                                                                              orientationVelocityTrackingWeight,
                                                                              omega,
                                                                              mass,
                                                                              gravityZ);
      this.orientationTrajectoryHandler = new ImplicitOrientationMPCTrajectoryHandler(indexHandler, orientationTrajectoryConstructor);

      registry.addChild(orientationTrajectoryHandler.getRegistry());

      this.momentOfInertia = momentOfInertia;

      orientationPreviewWindowDuration.set(0.25);

      qpSolver = new ImplicitSE3MPCQPSolver(indexHandler, dt, gravityZ, registry);
      qpSolver.setMaxNumberOfIterations(1000);

      qpSolver.setFirstOrientationVariableRegularization(1e-10);
      qpSolver.setSecondOrientationVariableRegularization(1e-10);

      parentRegistry.addChild(registry);
   }


   @Override
   protected void initializeIndexHandler()
   {
      List<ContactPlaneProvider> planningWindow = previewWindowCalculator.getPlanningWindow();
      indexHandler.initialize(planningWindow, currentTimeInState.getDoubleValue());
   }

   @Override
   protected void solveForTrajectoryOutsidePreviewWindow(List<ContactPlaneProvider> contactSequence)
   {
      super.solveForTrajectoryOutsidePreviewWindow(contactSequence);

      orientationTrajectoryHandler.solveForTrajectoryOutsidePreviewWindow(contactSequence);
      orientationTrajectoryHandler.computeDiscretizedReferenceTrajectory(currentTimeInState.getDoubleValue());
      orientationTrajectoryHandler.computeReferenceValue(orientationPreviewWindowDuration.getDoubleValue() + currentTimeInState.getDoubleValue());
   }


   @Override
   protected void setTerminalConditions()
   {
      super.setTerminalConditions();

      orientationAtEndOfWindow.set(orientationTrajectoryHandler.getReferenceBodyOrientation());
      angularVelocityAtEndOfWindow.set(orientationTrajectoryHandler.getReferenceBodyVelocity());
   }

   @Override
   protected void extractSolution(DMatrixRMaj solutionCoefficients)
   {
      super.extractSolution(solutionCoefficients);

      orientationTrajectoryHandler.extractSolutionForPreviewWindow(solutionCoefficients,
                                                                   currentTimeInState.getDoubleValue(),
                                                                   previewWindowCalculator.getPreviewWindowDuration(),
                                                                   currentBodyOrientation,
                                                                   currentBodyAngularVelocity,
                                                                   currentBodyAxisAngleError,
                                                                   currentBodyAngularVelocityError);
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
      computeInitialError();

      orientationTrajectoryConstructor.compute(previewWindowCalculator.getPlanningWindow(),
                                               momentOfInertia,
                                               linearTrajectoryHandler,
                                               orientationTrajectoryHandler,
                                               contactPlaneHelperPool);

      int numberOfSegments = indexHandler.getNumberOfSegments();
      for (int i = 0; i < numberOfSegments; i++)
      {
         mpcCommands.addCommand(orientationTrajectoryConstructor.getOrientationTrajectoryCommands().get(i));
         if (i < numberOfSegments - 1)
            mpcCommands.addCommand(computeOrientationContinuityCommand(i, commandProvider.getNextOrientationContinuityCommand()));
      }

      mpcCommands.addCommand(computeInitialOrientationErrorCommand(commandProvider.getNextOrientationValueCommand()));
      mpcCommands.addCommand(computeFinalOrientationMinimizationCommand(commandProvider.getNextOrientationValueCommand()));
   }

   private void computeInitialError()
   {
      orientationTrajectoryHandler.computeReferenceValue(currentTimeInState.getDoubleValue());
      FrameOrientation3DReadOnly referenceOrientation = orientationTrajectoryHandler.getReferenceBodyOrientation();

      angleTools.computeRotationError(referenceOrientation, currentBodyOrientation, currentBodyAxisAngleError);
      currentBodyAxisAngleError.get(initialError);

      currentBodyAngularVelocityError.sub(currentBodyAngularVelocity, orientationTrajectoryHandler.getReferenceBodyVelocity());
      referenceOrientation.inverseTransform(currentBodyAngularVelocityError);
      currentBodyAngularVelocityError.get(3, initialError);
   }

   private MPCCommand<?> computeInitialOrientationErrorCommand(OrientationValueCommand commandToPack)
   {
      commandToPack.reset();
      CommonOps_DDRM.setIdentity(commandToPack.getAMatrix());
      commandToPack.getBMatrix().reshape(6, LinearMPCIndexHandler.comCoefficientsPerSegment + indexHandler.getRhoCoefficientsInSegment(0));
      commandToPack.getBMatrix().zero();

      commandToPack.getCMatrix().zero();

      commandToPack.setSegmentNumber(0);
      commandToPack.setObjectiveValue(initialError);
      commandToPack.setConstraintType(ConstraintType.OBJECTIVE);
      commandToPack.setObjectiveWeight(initialOrientationWeight);

      return commandToPack;
   }

   private MPCCommand<?>  computeFinalOrientationMinimizationCommand(OrientationValueCommand commandToPack)
   {
      commandToPack.reset();

      int segmentNumber = indexHandler.getNumberOfSegments() - 1;
      commandToPack.setSegmentNumber(segmentNumber);

      OrientationTrajectoryCommand trajectoryCommand = orientationTrajectoryConstructor.getOrientationTrajectoryCommands().get(segmentNumber);
      commandToPack.setAMatrix(trajectoryCommand.getLastAMatrix());
      commandToPack.setBMatrix(trajectoryCommand.getLastBMatrix());
      commandToPack.setCMatrix(trajectoryCommand.getLastCMatrix());

      commandToPack.getObjectiveValue().zero();
      commandToPack.setConstraintType(ConstraintType.OBJECTIVE);
      commandToPack.setObjectiveWeight(finalOrientationWeight);

      return commandToPack;
   }

   private MPCCommand<?> computeOrientationContinuityCommand(int segmentNumber, OrientationContinuityCommand commandToPack)
   {
      commandToPack.reset();

      commandToPack.setSegmentNumber(segmentNumber);

      OrientationTrajectoryCommand trajectoryCommand = orientationTrajectoryConstructor.getOrientationTrajectoryCommands().get(segmentNumber);
      commandToPack.setAMatrix(trajectoryCommand.getLastAMatrix());
      commandToPack.setBMatrix(trajectoryCommand.getLastBMatrix());
      commandToPack.setCMatrix(trajectoryCommand.getLastCMatrix());

      commandToPack.setConstraintType(ConstraintType.EQUALITY);

      return commandToPack;
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

      referenceBodyOrientation.setMatchingFrame(orientationTrajectoryHandler.getReferenceBodyOrientation());
      referenceBodyAngularVelocity.setMatchingFrame(orientationTrajectoryHandler.getReferenceBodyVelocity());

      desiredBodyOrientation.setMatchingFrame(orientationTrajectoryHandler.getDesiredBodyOrientation());
      desiredBodyAngularVelocity.setMatchingFrame(orientationTrajectoryHandler.getDesiredAngularVelocity());
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

   public FrameVector3DReadOnly getDesiredBodyAngularVelocitySolution()
   {
      return desiredBodyAngularVelocity;
   }

   public FrameVector3DReadOnly getDesiredBodyAngularAccelerationSolution()
   {
      return desiredBodyAngularAcceleration;
   }

   public FrameOrientation3DReadOnly getReferenceBodyOrientation()
   {
      return referenceBodyOrientation;
   }

   public FrameVector3DReadOnly getReferenceBodyAngularVelocity()
   {
      return referenceBodyAngularVelocity;
   }

   public WrenchReadOnly getDesiredWrench()
   {
      return desiredWrench;
   }
}
