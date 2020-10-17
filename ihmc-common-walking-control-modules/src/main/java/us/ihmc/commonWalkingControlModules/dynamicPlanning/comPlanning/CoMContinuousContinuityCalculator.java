package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
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
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class CoMContinuousContinuityCalculator
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

   private final RecyclingArrayList<FramePoint3D> startVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> endVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);

   private final DoubleProvider omega;
   private final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final double gravityZ;

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

   public boolean solve(List<? extends ContactStateProvider> contactSequence)
   {
      if (contactSequence.size() < 3)
         return false;
      for (int i = 0; i < segmentContinuityDepth; i++)
      {
         if (!contactSequence.get(i).getContactState().isLoadBearing())
            return false;
      }

      contactSequenceInternal.clear();
      for (int i = 0; i < segmentContinuityDepth; i++)
         contactSequenceInternal.add(contactSequence.get(i));

      CoMTrajectoryPlannerTools.computeVRPWaypoints(comHeight.getDoubleValue(),
                                                    gravityZ,
                                                    omega.getValue(),
                                                    initialCoMVelocity,
                                                    contactSequenceInternal,
                                                    startVRPPositions,
                                                    endVRPPositions,
                                                    false);

      int numberOfPhases = contactSequenceInternal.size();
      int numberOfTransitions = numberOfPhases - 1;

      numberOfConstraints = 0;

      // set initial constraint
      setCoMPositionConstraint(initialCoMPosition);
      setCoMVelocityConstraint(initialCoMVelocity);
      setDynamicsInitialConstraint(contactSequenceInternal, 0);

      // add a moveable waypoint for the center of mass velocity constraint
      setCoMPositionContinuity(contactSequenceInternal, 0, 1);
      setCoMVelocityContinuity(contactSequenceInternal, 0, 1);

      setDynamicsContinuityConstraint(contactSequenceInternal, 0, 1);

      // set terminal constraint
      ContactStateProvider lastContactPhase = contactSequenceInternal.get(numberOfPhases - 1);
      double finalDuration = lastContactPhase.getTimeInterval().getDuration();
      setDCMPositionConstraint(numberOfPhases - 1, finalDuration, finalICPToAchieve);
      setDynamicsFinalConstraint(contactSequenceInternal, numberOfPhases - 1);

      sparseSolver.setA(coefficientMultipliersSparse);

      // TODO make an add equals method. Also don't pass in null, as that apparently makes garbage.
      CommonOps_DSCC.mult(vrpWaypointJacobian, vrpXWaypoints, tempSparse);
      CommonOps_DSCC.add(1.0, tempSparse, 1.0, xConstants, xEquivalents, gw, gx);

      CommonOps_DSCC.mult(vrpWaypointJacobian, vrpYWaypoints, tempSparse);
      CommonOps_DSCC.add(1.0, tempSparse, 1.0, yConstants, yEquivalents, gw, gx);

      CommonOps_DSCC.mult(vrpWaypointJacobian, vrpZWaypoints, tempSparse);
      CommonOps_DSCC.add(1.0, tempSparse, 1.0, zConstants, zEquivalents, gw, gx);

      sparseSolver.solveSparse(xEquivalents, xCoefficientVector);
      sparseSolver.solveSparse(yEquivalents, yCoefficientVector);
      sparseSolver.solveSparse(zEquivalents, zCoefficientVector);
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
                                                         numberOfConstraints,
                                                         coefficientMultipliersSparse,
                                                         xConstants,
                                                         yConstants,
                                                         zConstants);
      numberOfConstraints++;
   }

   private void setCoMPositionContinuity(List<? extends ContactStateProvider> contactSequence, int previousSequence, int nextSequence)
   {
      double previousDuration = contactSequence.get(previousSequence).getTimeInterval().getDuration();
      CoMTrajectoryPlannerTools.addCoMPositionContinuityConstraint(previousSequence,
                                                                   nextSequence,
                                                                   numberOfConstraints,
                                                                   omega.getValue(),
                                                                   previousDuration,
                                                                   coefficientMultipliersSparse);
      numberOfConstraints++;
   }

   private void setCoMVelocityContinuity(List<? extends ContactStateProvider> contactSequence, int previousSequence, int nextSequence)
   {
      double previousDuration = contactSequence.get(previousSequence).getTimeInterval().getDuration();
      CoMTrajectoryPlannerTools.addCoMVelocityContinuityConstraint(previousSequence,
                                                                   nextSequence,
                                                                   numberOfConstraints,
                                                                   omega.getValue(),
                                                                   previousDuration,
                                                                   coefficientMultipliersSparse);
      numberOfConstraints++;
   }

   private final FrameVector3D desiredVelocity = new FrameVector3D();

   private void setDynamicsInitialConstraint(List<? extends ContactStateProvider> contactSequence, int sequenceId)
   {
      ContactStateProvider contactStateProvider = contactSequence.get(sequenceId);
      double duration = contactStateProvider.getTimeInterval().getDuration();
      desiredVelocity.sub(endVRPPositions.get(sequenceId), startVRPPositions.get(sequenceId));
      desiredVelocity.scale(1.0 / duration);
      constrainVRPPosition(sequenceId, indexHandler.getVRPWaypointStartPositionIndex(sequenceId), 0.0, startVRPPositions.get(sequenceId));
      constrainVRPVelocity(sequenceId, indexHandler.getVRPWaypointStartVelocityIndex(sequenceId), 0.0, desiredVelocity);
   }

   private void setDynamicsFinalConstraint(List<? extends ContactStateProvider> contactSequence, int sequenceId)
   {
      ContactStateProvider contactStateProvider = contactSequence.get(sequenceId);
      double duration = contactStateProvider.getTimeInterval().getDuration();

      desiredVelocity.sub(endVRPPositions.get(sequenceId), startVRPPositions.get(sequenceId));
      desiredVelocity.scale(1.0 / duration);
      constrainVRPPosition(sequenceId, indexHandler.getVRPWaypointFinalPositionIndex(sequenceId), duration, endVRPPositions.get(sequenceId));
      constrainVRPVelocity(sequenceId, indexHandler.getVRPWaypointFinalVelocityIndex(sequenceId), duration, desiredVelocity);
   }

   private void setDynamicsContinuityConstraint(List<? extends ContactStateProvider> contactSequence, int previousSequenceId, int nextSequenceId)
   {
      ContactStateProvider previousSequence = contactSequence.get(previousSequenceId);
      ContactStateProvider nextSequence = contactSequence.get(nextSequenceId);
      if (previousSequence.getContactState().isLoadBearing() != nextSequence.getContactState().isLoadBearing())
         throw new IllegalArgumentException("Cannot constrain two sequences of different types to have equivalent dynamics.");

      setVRPPositionContinuity(contactSequence, previousSequenceId, nextSequenceId);

      double previousDuration = previousSequence.getTimeInterval().getDuration();
      double nextDuration = nextSequence.getTimeInterval().getDuration();
      addImplicitVRPVelocityConstraint(previousSequenceId,
                                       indexHandler.getVRPWaypointStartPositionIndex(previousSequenceId),
                                       previousDuration,
                                       0.0,
                                       startVRPPositions.get(previousSequenceId));
      addImplicitVRPVelocityConstraint(nextSequenceId,
                                       indexHandler.getVRPWaypointFinalPositionIndex(nextSequenceId),
                                       0.0,
                                       nextDuration,
                                       endVRPPositions.get(nextSequenceId));
   }

   private void constrainVRPPosition(int sequenceId, int vrpWaypointPositionIndex, double time, FramePoint3DReadOnly desiredVRPPosition)
   {
      CoMTrajectoryPlannerTools.addVRPPositionConstraint(sequenceId,
                                                         numberOfConstraints,
                                                         vrpWaypointPositionIndex,
                                                         time,
                                                         omega.getValue(),
                                                         desiredVRPPosition,
                                                         coefficientMultipliersSparse,
                                                         vrpXWaypoints,
                                                         vrpYWaypoints,
                                                         vrpZWaypoints,
                                                         vrpWaypointJacobian);
      numberOfConstraints++;
   }

   private void constrainVRPVelocity(int sequenceId, int vrpWaypointVelocityIndex, double time, FrameVector3DReadOnly desiredVRPVelocity)
   {
      CoMTrajectoryPlannerTools.addVRPVelocityConstraint(sequenceId,
                                                         numberOfConstraints,
                                                         vrpWaypointVelocityIndex,
                                                         omega.getValue(),
                                                         time,
                                                         desiredVRPVelocity,
                                                         coefficientMultipliersSparse,
                                                         vrpXWaypoints,
                                                         vrpYWaypoints,
                                                         vrpZWaypoints,
                                                         vrpWaypointJacobian);
      numberOfConstraints++;
   }

   private void setVRPPositionContinuity(List<? extends ContactStateProvider> contactSequence, int previousSequence, int nextSequence)
   {
      double previousDuration = contactSequence.get(previousSequence).getTimeInterval().getDuration();
      CoMTrajectoryPlannerTools.addVRPPositionContinuityConstraint(previousSequence,
                                                                   nextSequence,
                                                                   numberOfConstraints,
                                                                   omega.getValue(),
                                                                   previousDuration,
                                                                   coefficientMultipliersSparse);
      numberOfConstraints++;
   }

   private void addImplicitVRPVelocityConstraint(int sequenceId,
                                                 int vrpWaypointPositionIndex,
                                                 double time,
                                                 double timeAtWaypoint,
                                                 FramePoint3DReadOnly desiredVRPPosition)
   {
      CoMTrajectoryPlannerTools.addImplicitVRPVelocityConstraint(sequenceId,
                                                                 numberOfConstraints,
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
      numberOfConstraints++;
   }
}
