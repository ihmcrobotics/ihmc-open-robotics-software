package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.ImplicitSE3MPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.OrientationTrajectoryConstructor;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.math.trajectories.FixedFramePolynomialEstimator3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

/**
 * This class is meant to handle the trajectory from the MPC module. It includes the trajectory for the full planning window, which is overwritten with the
 * solution for the planning window at the beginning.
 * <p>
 * It assembles all this solution into a continuous multi-segment trajectory for the center of mass, and a list of polynomials for the VRP.
 */
public class ImplicitOrientationMPCTrajectoryHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final OrientationTrajectoryCalculator orientationInitializationCalculator;

   private final ImplicitSE3MPCIndexHandler indexHandler;

   private final RecyclingArrayList<FrameOrientation3DBasics> desiredOrientation = new RecyclingArrayList<>(FrameQuaternion::new);
   private final RecyclingArrayList<FrameVector3DBasics> desiredAngularVelocity = new RecyclingArrayList<>(FrameVector3D::new);

   private final RecyclingArrayList<FrameQuaternionBasics> orientationSolution = new RecyclingArrayList<>(FrameQuaternion::new);
   private final RecyclingArrayList<FrameVector3DBasics> angularVelocitySolution = new RecyclingArrayList<>(FrameVector3D::new);

   private final MultipleWaypointsOrientationTrajectoryGenerator bodyOrientationTrajectory;

   private final MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomialEstimator3D> internalAngularMomentumTrajectory;
   private final YoDouble previewWindowEndTime;

   private final RecyclingArrayList<AxisAngleBasics> axisAngleErrorSolutions = new RecyclingArrayList<>(AxisAngle::new);
   private final RecyclingArrayList<FrameVector3DBasics> angularVelocityErrorSolutions = new RecyclingArrayList<>(FrameVector3D::new);

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final OrientationTrajectoryConstructor trajectoryConstructor;

   public ImplicitOrientationMPCTrajectoryHandler(ImplicitSE3MPCIndexHandler indexHandler, OrientationTrajectoryConstructor trajectoryConstructor)
   {
      this.indexHandler = indexHandler;
      this.trajectoryConstructor = trajectoryConstructor;

      previewWindowEndTime = new YoDouble("orientationPreviewWindowEndTime", registry);

      internalAngularMomentumTrajectory = new MultipleSegmentPositionTrajectoryGenerator<>("internalAngularMomentumTrajectory",
                                                                                   50,
                                                                                   worldFrame,
                                                                                   () -> new FixedFramePolynomialEstimator3D(worldFrame),
                                                                                   registry);

      orientationInitializationCalculator = new OrientationTrajectoryCalculator(registry);
      bodyOrientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator("desiredCoMTrajectory", 100, ReferenceFrame.getWorldFrame(), registry);
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   /**
    * Clears the CoM and VRP solution trajectories
    */
   public void clearTrajectory()
   {
      previewWindowEndTime.set(Double.NEGATIVE_INFINITY);
      bodyOrientationTrajectory.clear();
   }



   public void extractSolutionForPreviewWindow(DMatrixRMaj solutionCoefficients,
                                               double currentTimeInState,
                                               double previewWindowDuration,
                                               Vector3DReadOnly axisAngleError,
                                               Vector3DReadOnly angularVelocityError)
   {
      previewWindowEndTime.set(currentTimeInState + previewWindowDuration);
      extractSolutionVectors(solutionCoefficients, axisAngleError, angularVelocityError);

      clearTrajectory();

      orientationSolution.clear();
      angularVelocitySolution.clear();

      int globalTick = 0;
      for (int segment = 0; segment < indexHandler.getNumberOfSegments(); segment++)
      {
         OrientationTrajectoryCommand command = trajectoryConstructor.getOrientationTrajectoryCommands().get(segment);
         double tickDuration = indexHandler.getTickDuration(segment);

         int end = globalTick + command.getNumberOfTicksInSegment();
         for (;globalTick < end; globalTick++)
         {
            currentTimeInState += tickDuration;

            FrameQuaternionBasics orientation = orientationSolution.add();
            orientation.set(desiredOrientation.get(globalTick));
            orientation.append(axisAngleErrorSolutions.get(globalTick));

            FrameVector3DBasics angularVelocity = angularVelocitySolution.add();
            angularVelocity.set(angularVelocityErrorSolutions.get(globalTick));

            // TODO is this the right direction?
            orientation.transform(angularVelocity);
            angularVelocity.add(desiredAngularVelocity.get(globalTick));

            bodyOrientationTrajectory.appendWaypoint(currentTimeInState, orientation, angularVelocity);
         }
      }

      overwriteTrajectoryOutsidePreviewWindow();
   }


   private final DMatrixRMaj errorAtStartOfState = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj valueAtTick = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj segmentCoefficients = new DMatrixRMaj(10, 1);

   private void extractSolutionVectors(DMatrixRMaj solutionCoefficients, Vector3DReadOnly axisAngleError,
                                       Vector3DReadOnly angularVelocityError)
   {
      axisAngleErrorSolutions.clear();
      angularVelocityErrorSolutions.clear();

      axisAngleError.get(errorAtStartOfState);
      angularVelocityError.get(3, errorAtStartOfState);

      for (int segment = 0; segment < indexHandler.getNumberOfSegments(); segment++)
      {
         OrientationTrajectoryCommand command = trajectoryConstructor.getOrientationTrajectoryCommands().get(segment);
         int coefficientsInSegment = indexHandler.getRhoCoefficientsInSegment(segment) + LinearMPCIndexHandler.comCoefficientsPerSegment;
         segmentCoefficients.reshape(coefficientsInSegment, 1);
         MatrixTools.setMatrixBlock(segmentCoefficients,
                                    0,
                                    0,
                                    solutionCoefficients,
                                    indexHandler.getComCoefficientStartIndex(segment),
                                    0,
                                    coefficientsInSegment,
                                    1,
                                    1.0);

         int tickStart = segment > 0 ? 1 : 0;

         for (int tick = tickStart; tick < command.getNumberOfTicksInSegment(); tick++)
         {
            valueAtTick.set(command.getCMatrix(tick));
            CommonOps_DDRM.multAdd(command.getAMatrix(tick), errorAtStartOfState, valueAtTick);
            CommonOps_DDRM.multAdd(command.getBMatrix(tick), segmentCoefficients, valueAtTick);

            AxisAngleBasics axisAngleErrorSolution = axisAngleErrorSolutions.add();
            FrameVector3DBasics angularVelocityErrorSolution = angularVelocityErrorSolutions.add();

            axisAngleErrorSolution.setRotationVector(valueAtTick.get(0, 0), valueAtTick.get(1, 0), valueAtTick.get(2, 0));
            angularVelocityErrorSolution.set(3, valueAtTick);
         }

         errorAtStartOfState.set(valueAtTick);
      }
   }

   public void setInitialBodyOrientationState(FrameOrientation3DReadOnly bodyOrientation, FrameVector3DReadOnly bodyAngularVelocity)
   {
      orientationInitializationCalculator.setInitialBodyOrientation(bodyOrientation, bodyAngularVelocity);
   }

   public void solveForTrajectoryOutsidePreviewWindow(List<ContactPlaneProvider> fullContactSequence)
   {
      orientationInitializationCalculator.solveForTrajectory(fullContactSequence);

      removeInfoOutsidePreviewWindow();
      overwriteTrajectoryOutsidePreviewWindow();
   }

   public void computeDesiredTrajectory(double currentTimeInState)
   {
      desiredOrientation.clear();
      desiredAngularVelocity.clear();

      for (int segment = 0; segment < indexHandler.getNumberOfSegments(); segment++)
      {
         double tickDuration = indexHandler.getTickDuration(segment);
         for (int i = 0; i < indexHandler.getTicksInSegment(segment); i++)
         {
            currentTimeInState += tickDuration;

            bodyOrientationTrajectory.compute(currentTimeInState);

            desiredOrientation.add().set(bodyOrientationTrajectory.getOrientation());
            desiredAngularVelocity.add().set(bodyOrientationTrajectory.getAngularVelocity());
         }
      }
   }

   private void removeInfoOutsidePreviewWindow()
   {
      while (bodyOrientationTrajectory.getCurrentNumberOfWaypoints() > 0 && bodyOrientationTrajectory.getLastWaypointTime() > previewWindowEndTime.getValue())
      {
         bodyOrientationTrajectory.removeLastWaypoint();
      }
   }

   private void overwriteTrajectoryOutsidePreviewWindow()
   {
      MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryOutsideWindow = orientationInitializationCalculator.getOrientationTrajectory();

      boolean hasTrajectoryAlready = bodyOrientationTrajectory.getCurrentNumberOfWaypoints() > 0;
      double existingEndTime = hasTrajectoryAlready ? bodyOrientationTrajectory.getLastWaypointTime() : 0.0;
      if (existingEndTime >= orientationTrajectoryOutsideWindow.getLastWaypointTime())
         return;

      int waypointIndexToAdd = getWaypointIndexAfterTime(existingEndTime + 1e-5, orientationInitializationCalculator.getOrientationTrajectory());
      if (waypointIndexToAdd == -1)
         return;

      for (; waypointIndexToAdd < orientationTrajectoryOutsideWindow.getCurrentNumberOfWaypoints(); waypointIndexToAdd++)
      {
         bodyOrientationTrajectory.appendWaypoint(orientationTrajectoryOutsideWindow.getWaypoint(waypointIndexToAdd));
      }

      bodyOrientationTrajectory.initialize();
   }

   private static int getWaypointIndexAfterTime(double time, MultipleWaypointsOrientationTrajectoryGenerator trajectory)
   {
      for (int i = 0; i < trajectory.getCurrentNumberOfWaypoints(); i++)
      {
         if (trajectory.getWaypoint(i).getTime() > time)
            return i;
      }

      return -1;
   }

   public void compute(double timeInPhase)
   {
      bodyOrientationTrajectory.compute(timeInPhase);
      orientationInitializationCalculator.compute(timeInPhase);
      if (!internalAngularMomentumTrajectory.isEmpty())
         internalAngularMomentumTrajectory.compute(timeInPhase);
   }

   public void computeOutsidePreview(double timeInPhase)
   {
      orientationInitializationCalculator.compute(timeInPhase);
   }

   public void setInternalAngularMomentumTrajectory(MultipleSegmentPositionTrajectoryGenerator<FixedFramePolynomialEstimator3D> trajectory)
   {
      internalAngularMomentumTrajectory.clear();
      for (int i = 0; i < trajectory.getCurrentNumberOfSegments(); i++)
         internalAngularMomentumTrajectory.appendSegment(trajectory.getSegment(i));
      internalAngularMomentumTrajectory.initialize();
   }

   public FrameOrientation3DReadOnly getDesiredBodyOrientationOutsidePreview()
   {
      return orientationInitializationCalculator.getDesiredOrientation();
   }

   public FrameVector3DReadOnly getDesiredBodyVelocityOutsidePreview()
   {
      return orientationInitializationCalculator.getDesiredAngularVelocity();
   }

   public FrameOrientation3DReadOnly getDesiredBodyOrientation()
   {
      return bodyOrientationTrajectory.getOrientation();
   }

   public FrameVector3DReadOnly getDesiredAngularVelocity()
   {
      return bodyOrientationTrajectory.getAngularVelocity();
   }

   public FrameVector3DReadOnly getDesiredAngularAcceleration()
   {
      return bodyOrientationTrajectory.getAngularAcceleration();
   }

   public boolean hasInternalAngularMomentum()
   {
      return internalAngularMomentumTrajectory.isEmpty();
   }

   public FramePoint3DReadOnly getDesiredInternalAngularMomentum()
   {
      return internalAngularMomentumTrajectory.getPosition();
   }

   public FrameVector3DReadOnly getDesiredInternalAngularMomentumRate()
   {
      return internalAngularMomentumTrajectory.getVelocity();
   }
}
