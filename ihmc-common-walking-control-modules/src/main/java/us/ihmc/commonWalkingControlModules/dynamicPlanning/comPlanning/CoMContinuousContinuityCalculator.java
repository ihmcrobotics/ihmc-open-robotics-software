package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.*;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.interfaces.linsol.LinearSolverSparse;
import org.ejml.sparse.FillReducing;
import org.ejml.sparse.csc.CommonOps_DSCC;
import org.ejml.sparse.csc.factory.LinearSolverFactory_DSCC;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class CoMContinuousContinuityCalculator implements CoMContinuityCalculator
{
   private final FramePoint3D initialCoMPosition = new FramePoint3D();
   private final FrameVector3D initialCoMVelocity = new FrameVector3D();

   private final FramePoint3D finalICPToAchieve = new FramePoint3D();

   private final List<ContactStateProvider> contactSequenceInternal = new ArrayList<>();

   private int segmentContinuityDepth = 2;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DMatrixSparseCSC coefficientMultipliersSparse = new DMatrixSparseCSC(0, 0);

   private final DMatrixSparseCSC tempSparse = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC xEquivalents = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC yEquivalents = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC zEquivalents = new DMatrixSparseCSC(0, 1);

   private final DMatrixSparseCSC xConstants = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC yConstants = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC zConstants = new DMatrixSparseCSC(0, 1);

   private final DMatrixSparseCSC vrpWaypointJacobian = new DMatrixSparseCSC(0, 1);

   private final DMatrixSparseCSC vrpXWaypoints = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC vrpYWaypoints = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC vrpZWaypoints = new DMatrixSparseCSC(0, 1);

   // FIXME fill reducing?
   private final LinearSolverSparse<DMatrixSparseCSC, DMatrixRMaj> sparseSolver = LinearSolverFactory_DSCC.lu(FillReducing.NONE);

   final DMatrixSparseCSC xCoefficientVector = new DMatrixSparseCSC(0, 1);
   final DMatrixSparseCSC yCoefficientVector = new DMatrixSparseCSC(0, 1);
   final DMatrixSparseCSC zCoefficientVector = new DMatrixSparseCSC(0, 1);

   private final CoMTrajectoryPlannerIndexHandler indexHandler = new CoMTrajectoryPlannerIndexHandler();

   private final RecyclingArrayList<FramePoint3D> startVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> endVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);

   private final DoubleProvider omega;
   private final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final double gravityZ;

   private int numberOfConstraints = 0;

   public CoMContinuousContinuityCalculator(double gravityZ, double nominalCoMHeight, YoRegistry parentRegistry)
   {
      this.gravityZ = Math.abs(gravityZ);
      YoDouble omega = new YoDouble("omegaForPlanning", registry);

      comHeight.addListener(v -> omega.set(Math.sqrt(Math.abs(gravityZ) / comHeight.getDoubleValue())));
      comHeight.set(nominalCoMHeight);

      this.omega = omega;

      parentRegistry.addChild(registry);
   }

   public CoMContinuousContinuityCalculator(double gravityZ, YoDouble omega, YoRegistry parentRegistry)
   {
      this.omega = omega;
      this.gravityZ = Math.abs(gravityZ);

      omega.addListener(v -> comHeight.set(gravityZ / MathTools.square(omega.getValue())));
      omega.notifyListeners();

      parentRegistry.addChild(registry);
   }

   public void setInitialCoMPosition(FramePoint3DReadOnly initialCoMPosition)
   {
      this.initialCoMPosition.setMatchingFrame(initialCoMPosition);
   }

   public void setInitialCoMVelocity(FrameVector3DReadOnly initialCoMVelocity)
   {
      this.initialCoMVelocity.setMatchingFrame(initialCoMVelocity);
   }

   public void setFinalICPToAchieve(FramePoint3DReadOnly finalICPToAchieve)
   {
      this.finalICPToAchieve.setMatchingFrame(finalICPToAchieve);
   }

   private final IGrowArray gw = new IGrowArray();
   private final DGrowArray gx = new DGrowArray();

   public boolean solve(List<? extends ContactStateProvider> contactSequence)
   {
      if (contactSequence.size() < 2)
         return false;
      for (int i = 0; i < segmentContinuityDepth; i++)
      {
         if (!contactSequence.get(i).getContactState().isLoadBearing())
            return false;
      }

      contactSequenceInternal.clear();
      for (int i = 0; i < segmentContinuityDepth; i++)
         contactSequenceInternal.add(contactSequence.get(i));

      indexHandler.update(contactSequenceInternal);

      resetMatrices();

      CoMTrajectoryPlannerTools.computeVRPWaypoints(comHeight.getDoubleValue(),
                                                    gravityZ,
                                                    omega.getValue(),
                                                    initialCoMVelocity,
                                                    contactSequenceInternal,
                                                    startVRPPositions,
                                                    endVRPPositions,
                                                    false);

      numberOfConstraints = 0;

      // set initial constraint
      double firstSegmentDuration = contactSequenceInternal.get(0).getTimeInterval().getDuration();
      double secondSegmentDuration = Math.min(contactSequenceInternal.get(1).getTimeInterval().getDuration(), 2.0);

      setCoMPositionConstraint(initialCoMPosition);
      setCoMVelocityConstraint(initialCoMVelocity);
      setDynamicsInitialConstraint(firstSegmentDuration);

      // add a moveable waypoint for the center of mass velocity constraint
      setCoMPositionContinuity(firstSegmentDuration);
      setCoMVelocityContinuity(firstSegmentDuration);

      setDynamicsContinuityConstraint(firstSegmentDuration, secondSegmentDuration);

      // set terminal constraint
      setFinalDCMPositionConstraint(secondSegmentDuration, finalICPToAchieve);
      setDynamicsFinalConstraint(secondSegmentDuration);

      sparseSolver.setA(coefficientMultipliersSparse);

      CommonOps_DSCC.mult(vrpWaypointJacobian, vrpXWaypoints, tempSparse);
      CommonOps_DSCC.add(1.0, tempSparse, 1.0, xConstants, xEquivalents, gw, gx);

      CommonOps_DSCC.mult(vrpWaypointJacobian, vrpYWaypoints, tempSparse);
      CommonOps_DSCC.add(1.0, tempSparse, 1.0, yConstants, yEquivalents, gw, gx);

      CommonOps_DSCC.mult(vrpWaypointJacobian, vrpZWaypoints, tempSparse);
      CommonOps_DSCC.add(1.0, tempSparse, 1.0, zConstants, zEquivalents, gw, gx);

      sparseSolver.solveSparse(xEquivalents, xCoefficientVector);
      sparseSolver.solveSparse(yEquivalents, yCoefficientVector);
      sparseSolver.solveSparse(zEquivalents, zCoefficientVector);

      return true;
   }

   public int getDepthForCalculation()
   {
      return segmentContinuityDepth;
   }

   public void getXCoefficientOverrides(DMatrix xCoefficientVectorToPack)
   {
      int size = xCoefficientVector.getNumRows();
      MatrixMissingTools.setMatrixBlock(xCoefficientVectorToPack, 0, 0, xCoefficientVector, 0, 0, size, 1, 1.0);
   }

   public void getYCoefficientOverrides(DMatrix yCoefficientVectorToPack)
   {
      int size = yCoefficientVector.getNumRows();
      MatrixMissingTools.setMatrixBlock(yCoefficientVectorToPack, 0, 0, yCoefficientVector, 0, 0, size, 1, 1.0);
   }

   public void getZCoefficientOverrides(DMatrix zCoefficientVectorToPack)
   {
      int size = zCoefficientVector.getNumRows();
      MatrixMissingTools.setMatrixBlock(zCoefficientVectorToPack, 0, 0, zCoefficientVector, 0, 0, size, 1, 1.0);
   }

   private void resetMatrices()
   {
      int size = indexHandler.getTotalNumberOfCoefficients();
      int numberOfVRPWaypoints = indexHandler.getNumberOfVRPWaypoints();

      coefficientMultipliersSparse.reshape(size, size);
      tempSparse.reshape(size, 1);
      xEquivalents.reshape(size, 1);
      yEquivalents.reshape(size, 1);
      zEquivalents.reshape(size, 1);
      xConstants.reshape(size, 1);
      yConstants.reshape(size, 1);
      zConstants.reshape(size, 1);
      vrpWaypointJacobian.reshape(size, numberOfVRPWaypoints); // only position
      vrpXWaypoints.reshape(numberOfVRPWaypoints, 1);
      vrpYWaypoints.reshape(numberOfVRPWaypoints, 1);
      vrpZWaypoints.reshape(numberOfVRPWaypoints, 1);
      xCoefficientVector.reshape(size, 1);
      yCoefficientVector.reshape(size, 1);
      zCoefficientVector.reshape(size, 1);

      coefficientMultipliersSparse.zero();
      xEquivalents.zero();
      yEquivalents.zero();
      zEquivalents.zero();
      xConstants.zero();
      yConstants.zero();
      zConstants.zero();
      vrpWaypointJacobian.zero();
      vrpXWaypoints.zero();
      vrpYWaypoints.zero();
      vrpZWaypoints.zero();
   }

   private void setCoMPositionConstraint(FramePoint3DReadOnly centerOfMassLocationForConstraint)
   {
      CoMTrajectoryPlannerTools.addCoMPositionConstraint(centerOfMassLocationForConstraint, omega.getValue(), 0.0, 0, numberOfConstraints,
                                                         coefficientMultipliersSparse, xConstants, yConstants, zConstants);
      numberOfConstraints++;
   }

   private void setCoMVelocityConstraint(FrameVector3DReadOnly centerOfMassVelocityForConstraint)
   {
      CoMTrajectoryPlannerTools.addCoMVelocityConstraint(centerOfMassVelocityForConstraint,
                                                         omega.getValue(),
                                                         0.0,
                                                         0,
                                                         numberOfConstraints++,
                                                         coefficientMultipliersSparse,
                                                         xConstants,
                                                         yConstants,
                                                         zConstants);
   }

   private void setCoMPositionContinuity(double previousDuration)
   {
      CoMTrajectoryPlannerTools.addCoMPositionContinuityConstraint(0,
                                                                   1,
                                                                   numberOfConstraints++,
                                                                   omega.getValue(),
                                                                   previousDuration,
                                                                   coefficientMultipliersSparse);
   }

   private void setCoMVelocityContinuity(double previousDuration)
   {
      CoMTrajectoryPlannerTools.addCoMVelocityContinuityConstraint(0,
                                                                   1,
                                                                   numberOfConstraints++,
                                                                   omega.getValue(),
                                                                   previousDuration,
                                                                   coefficientMultipliersSparse);
   }

   private final FrameVector3D desiredVelocity = new FrameVector3D();

   private void setDynamicsInitialConstraint(double duration)
   {
      desiredVelocity.sub(endVRPPositions.get(0), startVRPPositions.get(0));
      desiredVelocity.scale(1.0 / duration);
      constrainVRPPosition(0, indexHandler.getVRPWaypointStartPositionIndex(0), 0.0, startVRPPositions.get(0));
      constrainVRPVelocity(0, indexHandler.getVRPWaypointStartVelocityIndex(0), 0.0, desiredVelocity);
   }

   private void setDynamicsFinalConstraint(double segmentDuration)
   {
      desiredVelocity.sub(endVRPPositions.get(1), startVRPPositions.get(1));
      desiredVelocity.scale(1.0 / segmentDuration);
      constrainVRPPosition(1, indexHandler.getVRPWaypointFinalPositionIndex(1), segmentDuration, endVRPPositions.get(1));
      constrainVRPVelocity(1, indexHandler.getVRPWaypointFinalVelocityIndex(1), segmentDuration, desiredVelocity);
   }

   private void setDynamicsContinuityConstraint(double previousDuration, double nextDuration)
   {
      setVRPPositionContinuity(previousDuration);

      addImplicitVRPVelocityConstraint(0,
                                       indexHandler.getVRPWaypointStartPositionIndex(0),
                                       previousDuration,
                                       0.0,
                                       startVRPPositions.get(0));
      addImplicitVRPVelocityConstraint(1,
                                       indexHandler.getVRPWaypointFinalPositionIndex(1),
                                       0.0,
                                       nextDuration,
                                       endVRPPositions.get(1));
   }

   private void constrainVRPPosition(int sequenceId, int vrpWaypointPositionIndex, double time, FramePoint3DReadOnly desiredVRPPosition)
   {
      CoMTrajectoryPlannerTools.addVRPPositionConstraint(sequenceId,
                                                         numberOfConstraints++,
                                                         vrpWaypointPositionIndex,
                                                         time,
                                                         omega.getValue(),
                                                         desiredVRPPosition,
                                                         coefficientMultipliersSparse,
                                                         vrpXWaypoints,
                                                         vrpYWaypoints,
                                                         vrpZWaypoints,
                                                         vrpWaypointJacobian);
   }

   private void constrainVRPVelocity(int sequenceId, int vrpWaypointVelocityIndex, double time, FrameVector3DReadOnly desiredVRPVelocity)
   {
      CoMTrajectoryPlannerTools.addVRPVelocityConstraint(sequenceId,
                                                         numberOfConstraints++,
                                                         vrpWaypointVelocityIndex,
                                                         omega.getValue(),
                                                         time,
                                                         desiredVRPVelocity,
                                                         coefficientMultipliersSparse,
                                                         vrpXWaypoints,
                                                         vrpYWaypoints,
                                                         vrpZWaypoints,
                                                         vrpWaypointJacobian);
   }

   private void setVRPPositionContinuity(double previousDuration)
   {
      CoMTrajectoryPlannerTools.addVRPPositionContinuityConstraint(0,
                                                                   1,
                                                                   numberOfConstraints++,
                                                                   omega.getValue(),
                                                                   previousDuration,
                                                                   coefficientMultipliersSparse);
   }

   private void addImplicitVRPVelocityConstraint(int sequenceId,
                                                 int vrpWaypointPositionIndex,
                                                 double time,
                                                 double timeAtWaypoint,
                                                 FramePoint3DReadOnly desiredVRPPosition)
   {
      CoMTrajectoryPlannerTools.addImplicitVRPVelocityConstraint(sequenceId,
                                                                 numberOfConstraints++,
                                                                 vrpWaypointPositionIndex,
                                                                 time,
                                                                 timeAtWaypoint,
                                                                 omega.getValue(),
                                                                 desiredVRPPosition,
                                                                 coefficientMultipliersSparse,
                                                                 vrpXWaypoints,
                                                                 vrpYWaypoints,
                                                                 vrpZWaypoints,
                                                                 vrpWaypointJacobian);
   }

   private void setFinalDCMPositionConstraint(double time, FramePoint3DReadOnly desiredDCMPosition)
   {
      CoMTrajectoryPlannerTools.addDCMPositionConstraint(1, numberOfConstraints, time, omega.getValue(), desiredDCMPosition, coefficientMultipliersSparse,
                                                         xConstants, yConstants, zConstants);
      numberOfConstraints++;
   }
}
