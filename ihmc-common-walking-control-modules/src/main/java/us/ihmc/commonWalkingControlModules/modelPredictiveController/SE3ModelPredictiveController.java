package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.OrientationTrajectoryConstructor;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.SE3MPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.SE3MPCQPSolver;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.customPolicies.CustomMPCPolicy;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
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
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.math.trajectories.FixedFramePolynomialEstimator3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.IntUnaryOperator;

public class SE3ModelPredictiveController extends EuclideanModelPredictiveController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean defaultIncludeIntermediateOrientationTracking = true;

   private final double gravityZ;
   protected final double mass;

   protected final SE3MPCIndexHandler indexHandler;

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

   protected final YoVector3D currentBodyAxisAngleError = new YoVector3D("currentBodyAxisAngleError", registry);

   private final OrientationTrajectoryConstructor orientationTrajectoryConstructor;
   final OrientationMPCTrajectoryHandler orientationTrajectoryHandler;
   private SE3MPCTrajectoryViewer trajectoryViewer = null;

   final SE3MPCQPSolver qpSolver;

   protected final Matrix3DReadOnly momentOfInertia;

   private final MPCAngleTools angleTools = new MPCAngleTools();

   protected final YoVector3D currentBodyAngularVelocityError = new YoVector3D("currentBodyAngularVelocityError", registry);

   private final YoBoolean includeIntermediateOrientationTracking = new YoBoolean("includeIntermediateOrientationTracking", registry);

   private final IntUnaryOperator firstVariableIndex;

   private final List<CustomMPCPolicy> customMPCPoliciesToProcess = new ArrayList<>();

   public SE3ModelPredictiveController(Matrix3DReadOnly momentOfInertia,
                                       MPCParameters mpcParameters,
                                       double gravityZ,
                                       double nominalCoMHeight,
                                       double mass,
                                       double dt,
                                       YoRegistry parentRegistry)
   {
      this(new SE3MPCIndexHandler(numberOfBasisVectorsPerContactPoint),
           momentOfInertia,
           mpcParameters,
           gravityZ,
           nominalCoMHeight,
           mass,
           dt,
           parentRegistry);
   }

   public SE3ModelPredictiveController(SE3MPCIndexHandler indexHandler,
                                       Matrix3DReadOnly momentOfInertia,
                                       MPCParameters mpcParameters,
                                       double gravityZ,
                                       double nominalCoMHeight,
                                       double mass,
                                       double dt,
                                       YoRegistry parentRegistry)
   {
      super(indexHandler, mpcParameters, mass, gravityZ, nominalCoMHeight, parentRegistry);

      this.indexHandler = indexHandler;
      this.gravityZ = Math.abs(gravityZ);
      this.mass = mass;

      registry.addChild(indexHandler.getRegistry());

      firstVariableIndex = indexHandler::getOrientationStartIndex;

      orientationTrajectoryConstructor = new OrientationTrajectoryConstructor(indexHandler,
                                                                              mpcParameters.getOrientationAngleTrackingWeightProvider(),
                                                                              mpcParameters.getOrientationVelocityTrackingWeightProvider(),
                                                                              omega,
                                                                              mass,
                                                                              gravityZ);
      this.orientationTrajectoryHandler = new OrientationMPCTrajectoryHandler(indexHandler, orientationTrajectoryConstructor);

      registry.addChild(orientationTrajectoryHandler.getRegistry());

      this.momentOfInertia = momentOfInertia;

      qpSolver = new SE3MPCQPSolver(indexHandler, dt, gravityZ, registry);
      qpSolver.setMaxNumberOfIterations(10);

      qpSolver.setFirstOrientationVariableRegularization(1e-10);
      qpSolver.setSecondOrientationVariableRegularization(1e-10);

      includeIntermediateOrientationTracking.set(defaultIncludeIntermediateOrientationTracking);

      parentRegistry.addChild(registry);
   }
   
   public void addCustomPolicyToProcess(CustomMPCPolicy policyToProcess)
   {
      customMPCPoliciesToProcess.add(policyToProcess);
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
      orientationTrajectoryHandler.computeDiscretizedReferenceTrajectory(currentTimeInState.getDoubleValue());
      orientationTrajectoryHandler.computeReferenceValue(previewWindowCalculator.getPreviewWindowDuration() + currentTimeInState.getDoubleValue());
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
                                                                   currentBodyAngularVelocity);
   }

   @Override
   protected void computeObjectives(List<ContactPlaneProvider> contactSequence)
   {
      super.computeObjectives(contactSequence);

      computeOrientationObjectives();

      computeCustomMPCPolicyObjectives(contactSequence);
   }

   private final DMatrixRMaj initialError = new DMatrixRMaj(6, 1);

   private void computeOrientationObjectives()
   {
      computeInitialError();

      orientationTrajectoryConstructor.compute(previewWindowCalculator.getPlanningWindow(),
                                               momentOfInertia,
                                               linearTrajectoryHandler,
                                               orientationTrajectoryHandler,
                                               contactHandler.getContactPlanes());

      int numberOfSegments = indexHandler.getNumberOfSegments();
      for (int i = 0; i < numberOfSegments; i++)
      {
         if (includeIntermediateOrientationTracking.getBooleanValue())
            mpcCommands.addCommand(orientationTrajectoryConstructor.getOrientationTrajectoryCommands().get(i));
         if (i < numberOfSegments - 1)
            mpcCommands.addCommand(computeOrientationContinuityCommand(i, commandProvider.getNextOrientationContinuityCommand()));
      }

      mpcCommands.addCommand(computeInitialOrientationErrorCommand(commandProvider.getNextDirectOrientationValueCommand()));
      mpcCommands.addCommand(computeFinalOrientationMinimizationCommand(commandProvider.getNextOrientationValueCommand()));
   }

   private void computeCustomMPCPolicyObjectives(List<ContactPlaneProvider> contactSequence)
   {
      for (int i = 0; i < customMPCPoliciesToProcess.size(); i++)
      {
         mpcCommands.addCommand(customMPCPoliciesToProcess.get(i).computeMPCCommand(contactHandler, contactSequence, omega.getValue()));
      }

      customMPCPoliciesToProcess.clear();
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

   private MPCCommand<?> computeInitialOrientationErrorCommand(DirectOrientationValueCommand commandToPack)
   {
      commandToPack.reset();

      commandToPack.setSegmentNumber(0);
      commandToPack.setObjectiveValue(initialError);
      commandToPack.setConstraintType(ConstraintType.OBJECTIVE);
      commandToPack.setObjectiveWeight(mpcParameters.getInitialOrientationWeight());

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
      commandToPack.setObjectiveWeight(mpcParameters.getFinalOrientationWeight());

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
   protected NativeMatrix solveQP()
   {
      qpSolver.initialize();
      qpSolver.submitMPCCommandList(mpcCommands);

      qpSolver.setUseWarmStart(useWarmStart.getBooleanValue());
      if (useWarmStart.getBooleanValue())
      {
         assembleActiveSet(firstVariableIndex);
         qpSolver.setPreviousSolution(previousSolution);
         qpSolver.setActiveInequalityIndices(activeInequalityConstraints);
      }

      if (!qpSolver.solve())
      {
         LogTools.info("Failed to find solution");
         extractNewActiveSetData(false, qpSolver, firstVariableIndex);
         return null;
      }

      extractNewActiveSetData(true, qpSolver, firstVariableIndex);

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
      linearTrajectoryHandler.computeOutsidePreview(timeInPhase);
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

   private final FrameVector3D desiredMomentArm = new FrameVector3D();
   private final FrameVector3D desiredPointTorque = new FrameVector3D();

   public void computeTorque(double time, FixedFrameVector3DBasics torqueToPack)
   {
      List<MPCContactPlane> contactPlanes = contactHandler.getContactPlanesForSegment(0);
      linearTrajectoryHandler.compute(time);

      torqueToPack.setToZero();

      time -= currentTimeInState.getDoubleValue();

      for (int planeIdx = 0; planeIdx < contactPlanes.size(); planeIdx++)
      {
         MPCContactPlane contactPlane = contactPlanes.get(planeIdx);
         contactPlane.computeContactForce(omega.getValue(), time);
         for (int pointIdx = 0; pointIdx < contactPlane.getNumberOfContactPoints(); pointIdx++)
         {
            desiredMomentArm.sub(contactPlane.getContactPointHelper(pointIdx).getBasisVectorOrigin(), linearTrajectoryHandler.getDesiredCoMPosition());
            desiredPointTorque.cross(desiredMomentArm, contactPlane.getContactPointHelper(pointIdx).getContactAcceleration());
            desiredPointTorque.scale(mass);

            torqueToPack.add(desiredPointTorque);
         }
      }
   }
}
