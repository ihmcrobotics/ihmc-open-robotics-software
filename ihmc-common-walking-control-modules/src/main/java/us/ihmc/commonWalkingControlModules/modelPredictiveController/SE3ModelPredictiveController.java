package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DiscreteAngularVelocityOrientationCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommandList;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.SE3MPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.SE3MPCQPSolver;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.OrientationMPCTrajectoryHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.tools.MPCAngleTools;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.LinearMPCTrajectoryViewer;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization.SE3MPCTrajectoryViewer;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
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

public class SE3ModelPredictiveController extends EuclideanModelPredictiveController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final double gravityZ;
   protected final double mass;

   protected final SE3MPCIndexHandler indexHandler;

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

   final OrientationMPCTrajectoryHandler orientationTrajectoryHandler;
   private SE3MPCTrajectoryViewer trajectoryViewer = null;

   final SE3MPCQPSolver qpSolver;

   protected final Matrix3DReadOnly momentOfInertia;

   private final MPCAngleTools angleTools = new MPCAngleTools();

   protected final YoVector3D currentBodyAngularVelocityError = new YoVector3D("currentBodyAngularVelocityError", registry);

   public SE3ModelPredictiveController(Matrix3DReadOnly momentOfInertia,
                                       double gravityZ, double nominalCoMHeight, double mass, double dt,
                                       YoRegistry parentRegistry)
   {
      this(new SE3MPCIndexHandler(numberOfBasisVectorsPerContactPoint),
           momentOfInertia,
           gravityZ,
           nominalCoMHeight,
           mass,
           dt,
           parentRegistry);
   }

   public SE3ModelPredictiveController(SE3MPCIndexHandler indexHandler,
                                       Matrix3DReadOnly momentOfInertia,
                                       double gravityZ, double nominalCoMHeight, double mass, double dt,
                                       YoRegistry parentRegistry)
   {
      super(indexHandler, mass, gravityZ, nominalCoMHeight, parentRegistry);

      this.indexHandler = indexHandler;
      this.gravityZ = Math.abs(gravityZ);
      this.mass = mass;
      this.orientationTrajectoryHandler = new OrientationMPCTrajectoryHandler(indexHandler);

      registry.addChild(orientationTrajectoryHandler.getRegistry());

      this.momentOfInertia = momentOfInertia;

      orientationPreviewWindowDuration.set(0.25);

      qpSolver = new SE3MPCQPSolver(indexHandler, dt, gravityZ, mass, registry);
      qpSolver.setMaxNumberOfIterations(1000);

      qpSolver.setFirstOrientationVariableRegularization(1e-2);
      qpSolver.setSecondOrientationVariableRegularization(1e-2);

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

      orientationTrajectoryHandler.extractSolutionForPreviewWindow(solutionCoefficients, currentTimeInState.getDoubleValue(), orientationPreviewWindowDuration.getDoubleValue());
   }

   @Override
   protected void computeObjectives(List<ContactPlaneProvider> contactSequence)
   {
      super.computeObjectives(contactSequence);

      computeOrientationObjectives();
   }

   private void computeOrientationObjectives()
   {
      mpcCommands.addCommand(computeInitialDiscreteOrientationObjective(commandProvider.getNextDiscreteAngularVelocityOrientationCommand()));
      mpcCommands.addCommandList(computeDiscreteOrientationObjectives());
   }

   private final FrameVector3D tempVector = new FrameVector3D();

   private MPCCommand<?> computeInitialDiscreteOrientationObjective(DiscreteAngularVelocityOrientationCommand objectiveToPack)
   {
      objectiveToPack.clear();
      objectiveToPack.setOmega(omega.getValue());
      objectiveToPack.setSegmentNumber(0);
      objectiveToPack.setDurationOfHold(indexHandler.getOrientationTickDuration(0));
      objectiveToPack.setTimeOfConstraint(0.0);
      objectiveToPack.setEndingDiscreteTickId(0);

      linearTrajectoryHandler.compute(currentTimeInState.getDoubleValue());
      orientationTrajectoryHandler.compute(currentTimeInState.getDoubleValue());

      FrameOrientation3DReadOnly desiredOrientation = orientationTrajectoryHandler.getDesiredBodyOrientation();
      objectiveToPack.setDesiredBodyOrientation(desiredOrientation);
      // angular velocity in body frame
      tempVector.set(orientationTrajectoryHandler.getDesiredAngularVelocity());
      desiredOrientation.transform(tempVector);
      objectiveToPack.setDesiredBodyAngularVelocityInBodyFrame(tempVector);

      tempVector.setToZero();
      if (orientationTrajectoryHandler.hasInternalAngularMomentum())
      {
         objectiveToPack.setDesiredInternalAngularMomentumRate(orientationTrajectoryHandler.getDesiredInternalAngularMomentumRate());
         if (contactPlaneHelperPool.get(0).size() > 0)
            objectiveToPack.setDesiredNetAngularMomentumRate(orientationTrajectoryHandler.getDesiredInternalAngularMomentumRate());
         else
            objectiveToPack.setDesiredNetAngularMomentumRate(tempVector);
      }
      else
      {
         objectiveToPack.setDesiredNetAngularMomentumRate(tempVector);
         objectiveToPack.setDesiredInternalAngularMomentumRate(tempVector);
      }

      objectiveToPack.setMomentOfInertiaInBodyFrame(momentOfInertia);

      objectiveToPack.setDesiredCoMPosition(currentCoMPosition);
      objectiveToPack.setDesiredCoMAcceleration(linearTrajectoryHandler.getDesiredCoMAcceleration());

      angleTools.computeRotationError(desiredOrientation, currentBodyOrientation, currentBodyAxisAngleError);

      objectiveToPack.setCurrentAxisAngleError(currentBodyAxisAngleError);

      currentBodyAngularVelocityError.sub(currentBodyAngularVelocity, orientationTrajectoryHandler.getDesiredAngularVelocity());
      desiredOrientation.inverseTransform(currentBodyAngularVelocityError);
      objectiveToPack.setCurrentBodyAngularVelocityErrorInBodyFrame(currentBodyAngularVelocityError);

      for (int i = 0; i < contactPlaneHelperPool.get(0).size(); i++)
      {
         objectiveToPack.addContactPlaneHelper(contactPlaneHelperPool.get(0).get(i));
      }

      return objectiveToPack;
   }

   private final MPCCommandList commandList = new MPCCommandList();

   private MPCCommandList computeDiscreteOrientationObjectives()
   {
      commandList.clear();

      int tick = 1;
      int endTick = 0;
      double globalTime = currentTimeInState.getDoubleValue();
      for (int segment = 0; segment < indexHandler.getNumberOfSegments(); segment++)
      {
         endTick += indexHandler.getOrientationTicksInSegment(segment);
         double localTime = 0.0;
         for (; tick < endTick; tick++)
         {
            DiscreteAngularVelocityOrientationCommand objective = commandProvider.getNextDiscreteAngularVelocityOrientationCommand();

            double tickDuration = indexHandler.getOrientationTickDuration(segment);
            localTime += tickDuration;
            globalTime += tickDuration;

            objective.clear();
            objective.setOmega(omega.getValue());
            objective.setSegmentNumber(segment);
            objective.setDurationOfHold(tickDuration);
            objective.setTimeOfConstraint(localTime);
            objective.setEndingDiscreteTickId(tick);

            linearTrajectoryHandler.compute(globalTime);
            orientationTrajectoryHandler.computeOutsidePreview(globalTime);

            FrameOrientation3DReadOnly desiredOrientation = orientationTrajectoryHandler.getDesiredBodyOrientationOutsidePreview();
            objective.setDesiredBodyOrientation(desiredOrientation);
            // angular velocity in body frame
            tempVector.set(orientationTrajectoryHandler.getDesiredBodyVelocityOutsidePreview());
            desiredOrientation.transform(tempVector);
            objective.setDesiredBodyAngularVelocityInBodyFrame(tempVector);

            tempVector.setToZero();
            if (orientationTrajectoryHandler.hasInternalAngularMomentum())
            {
               objective.setDesiredInternalAngularMomentumRate(orientationTrajectoryHandler.getDesiredInternalAngularMomentumRate());
               if (contactPlaneHelperPool.get(0).size() > 0)
                  objective.setDesiredNetAngularMomentumRate(orientationTrajectoryHandler.getDesiredInternalAngularMomentumRate());
               else
                  objective.setDesiredNetAngularMomentumRate(tempVector);
            }
            else
            {
               objective.setDesiredNetAngularMomentumRate(tempVector);
               objective.setDesiredInternalAngularMomentumRate(tempVector);
            }

            objective.setMomentOfInertiaInBodyFrame(momentOfInertia);

            objective.setDesiredCoMPosition(linearTrajectoryHandler.getDesiredCoMPosition());
            objective.setDesiredCoMAcceleration(linearTrajectoryHandler.getDesiredCoMAcceleration());

            for (int i = 0; i < contactPlaneHelperPool.get(segment).size(); i++)
            {
               objective.addContactPlaneHelper(contactPlaneHelperPool.get(segment).get(i));
            }

            commandList.addCommand(objective);
         }
      }

      return commandList;
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
