package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.MultipleCoMSegmentTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.SE3MPCIndexHandler;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.math.trajectories.core.FramePolynomial3D;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.time.TimeIntervalProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

/**
 * This class is meant to handle the trajectory from the MPC module. It includes the trajectory for the full planning window, which is overwritten with the
 * solution for the planning window at the beginning.
 * <p>
 * It assembles all this solution into a continuous multi-segment trajectory for the center of mass, and a list of polynomials for the VRP.
 */
public class MomentumOrientationMPCTrajectoryHandler extends OrientationMPCTrajectoryHandler
{
   private final RecyclingArrayList<AxisAngleBasics> axisAngleErrorSolution = new RecyclingArrayList<>(AxisAngle::new);
   private final RecyclingArrayList<FrameVector3DBasics> bodyAngularMomentaAboutPointSolution = new RecyclingArrayList<>(FrameVector3D::new);

   private final Matrix3DReadOnly momentOfInertiaMatrix;
   private final double mass;

   public MomentumOrientationMPCTrajectoryHandler(SE3MPCIndexHandler indexHandler, Matrix3DReadOnly momentOfInertiaMatrix, double mass, YoRegistry registry)
   {
      super(indexHandler, registry);

      this.momentOfInertiaMatrix = momentOfInertiaMatrix;
      this.mass = mass;
   }

   @Override
   public void extractSolutionForPreviewWindow(DMatrixRMaj solutionCoefficients,
                                               MultipleCoMSegmentTrajectoryGenerator comTrajectorySolution,
                                               double currentTimeInState,
                                               double previewWindowDuration)
   {
      previewWindowEndTime.set(currentTimeInState + previewWindowDuration);
      extractSolutionVectors(solutionCoefficients);

      clearTrajectory();

      orientationSolution.clear();
      angularVelocitySolution.clear();

      int globalTick = 0;
      for (int segment = 0; segment < indexHandler.getNumberOfSegments(); segment++)
      {
         int end = globalTick + indexHandler.getOrientationTicksInSegment(segment);
         for (;globalTick < end; globalTick++)
         {
            currentTimeInState += indexHandler.getOrientationTickDuration(segment);

            FrameQuaternionBasics orientation = orientationSolution.add();
            orientation.set(desiredOrientation.get(globalTick));
            orientation.append(axisAngleErrorSolution.get(globalTick));

            internalAngularMomentumTrajectory.compute(currentTimeInState);
            comTrajectorySolution.compute(currentTimeInState);

            FrameVector3DBasics angularVelocity = angularVelocitySolution.add();
            angularVelocity.cross(comTrajectorySolution.getPosition(), comTrajectorySolution.getVelocity());
            angularVelocity.scaleAdd(-mass, internalAngularMomentumTrajectory.getPosition());

            orientation.inverseTransform(angularVelocity);
            momentOfInertiaMatrix.inverseTransform(angularVelocity);

            orientationTrajectory.appendWaypoint(currentTimeInState, orientation, angularVelocity);
         }
      }

      overwriteTrajectoryOutsidePreviewWindow();
   }

   private void extractSolutionVectors(DMatrixRMaj solutionCoefficients)
   {
      int totalNumberOfTicks = 0;
      for (int i = 0; i < indexHandler.getNumberOfSegments(); i++)
         totalNumberOfTicks += indexHandler.getOrientationTicksInSegment(i);

      axisAngleErrorSolution.clear();
      bodyAngularMomentaAboutPointSolution.clear();

      for (int i = 0; i < totalNumberOfTicks; i++)
      {
         AxisAngleBasics axisAngleError = axisAngleErrorSolution.add();
         FrameVector3DBasics bodyAngularMomentumAboutPoint = bodyAngularMomentaAboutPointSolution.add();
         axisAngleError.setRotationVector(solutionCoefficients.get(indexHandler.getOrientationTickStartIndex(i), 0),
                                          solutionCoefficients.get(indexHandler.getOrientationTickStartIndex(i) + 1, 0),
                                          solutionCoefficients.get(indexHandler.getOrientationTickStartIndex(i) + 2, 0));
         bodyAngularMomentumAboutPoint.set(solutionCoefficients.get(indexHandler.getOrientationTickStartIndex(i) + 3, 0),
                                           solutionCoefficients.get(indexHandler.getOrientationTickStartIndex(i) + 4, 0),
                                           solutionCoefficients.get(indexHandler.getOrientationTickStartIndex(i) + 5, 0));
      }
   }
}
