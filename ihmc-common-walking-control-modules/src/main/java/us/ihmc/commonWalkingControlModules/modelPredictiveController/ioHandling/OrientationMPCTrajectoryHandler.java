package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.MultipleCoMSegmentTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.SE3MPCIndexHandler;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.math.trajectories.core.FramePolynomial3D;
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
public abstract class OrientationMPCTrajectoryHandler
{
   protected final OrientationTrajectoryCalculator orientationInitializationCalculator;

   protected final SE3MPCIndexHandler indexHandler;

   protected final RecyclingArrayList<FrameOrientation3DBasics> desiredOrientation = new RecyclingArrayList<>(FrameQuaternion::new);
   protected final RecyclingArrayList<FrameVector3DBasics> desiredAngularVelocity = new RecyclingArrayList<>(FrameVector3D::new);

   protected final RecyclingArrayList<FrameQuaternionBasics> orientationSolution = new RecyclingArrayList<>(FrameQuaternion::new);
   protected final RecyclingArrayList<FrameVector3DBasics> angularVelocitySolution = new RecyclingArrayList<>(FrameVector3D::new);

   protected final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectory;

   protected final FramePolynomial3D internalAngularMomentumTrajectory = new FramePolynomial3D(3, ReferenceFrame.getWorldFrame());
   protected final YoDouble previewWindowEndTime;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   public OrientationMPCTrajectoryHandler(SE3MPCIndexHandler indexHandler)
   {
      this.indexHandler = indexHandler;

      previewWindowEndTime = new YoDouble("orientationPreviewWindowEndTime", registry);
      internalAngularMomentumTrajectory.setConstant(new Point3D());

      orientationInitializationCalculator = new OrientationTrajectoryCalculator(registry);
      orientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator("desiredCoMTrajectory", 100, ReferenceFrame.getWorldFrame(), registry);
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
      orientationTrajectory.clear();
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
         for (int i = 0; i < indexHandler.getOrientationTicksInSegment(segment); i++)
         {
            currentTimeInState += indexHandler.getOrientationTickDuration(segment);

            orientationTrajectory.compute(currentTimeInState);

            desiredOrientation.add().set(orientationTrajectory.getOrientation());
            desiredAngularVelocity.add().set(orientationTrajectory.getAngularVelocity());
         }
      }
   }

   public abstract void extractSolutionForPreviewWindow(DMatrixRMaj solutionCoefficients,
                                               MultipleCoMSegmentTrajectoryGenerator comTrajectorySolution,
                                               double currentTimeInState,
                                               double previewWindowDuration);

   protected void removeInfoOutsidePreviewWindow()
   {
      while (orientationTrajectory.getCurrentNumberOfWaypoints() > 0 && orientationTrajectory.getLastWaypointTime() > previewWindowEndTime.getValue())
      {
         orientationTrajectory.removeLastWaypoint();
      }
   }

   protected void overwriteTrajectoryOutsidePreviewWindow()
   {
      MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryOutsideWindow = orientationInitializationCalculator.getOrientationTrajectory();

      boolean hasTrajectoryAlready = orientationTrajectory.getCurrentNumberOfWaypoints() > 0;
      double existingEndTime = hasTrajectoryAlready ? orientationTrajectory.getLastWaypointTime() : 0.0;
      if (existingEndTime >= orientationTrajectoryOutsideWindow.getLastWaypointTime())
         return;

      int waypointIndexToAdd = getWaypointIndexAfterTime(existingEndTime + 1e-5, orientationInitializationCalculator.getOrientationTrajectory());
      if (waypointIndexToAdd == -1)
         return;

      for (; waypointIndexToAdd < orientationTrajectoryOutsideWindow.getCurrentNumberOfWaypoints(); waypointIndexToAdd++)
      {
         orientationTrajectory.appendWaypoint(orientationTrajectoryOutsideWindow.getWaypoint(waypointIndexToAdd));
      }

      orientationTrajectory.initialize();
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
      orientationTrajectory.compute(timeInPhase);
      orientationInitializationCalculator.compute(timeInPhase);
      internalAngularMomentumTrajectory.compute(timeInPhase);
   }

   public void computeOutsidePreview(double timeInPhase)
   {
      orientationInitializationCalculator.compute(timeInPhase);
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
      return orientationTrajectory.getOrientation();
   }

   public FrameVector3DReadOnly getDesiredAngularVelocity()
   {
      return orientationTrajectory.getAngularVelocity();
   }

   public FrameVector3DReadOnly getDesiredAngularAcceleration()
   {
      return orientationTrajectory.getAngularAcceleration();
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
