package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.*;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class CoMOptimalContinuityCalculator implements CoMContinuityCalculator
{
   private final static boolean CREATE_DEBUG_TRAJECTORY = true;

   private final FramePoint3D initialCoMPosition = new FramePoint3D();
   private final FrameVector3D initialCoMVelocity = new FrameVector3D();

   private final FramePoint3D finalICPToAchieve = new FramePoint3D();

   private final List<ContactStateProvider<?>> contactSequenceInternal = new ArrayList<>();

   private static final int segmentContinuityDepth = 2;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DMatrixRMaj tempJacobian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj tempXObjective = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj tempYObjective = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj tempZObjective = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj constraintAMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj xConstraintObjective = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj yConstraintObjective = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj zConstraintObjective = new DMatrixRMaj(0, 1);

   private final DMatrixSparseCSC vrpWaypointJacobian = new DMatrixSparseCSC(0, 1);

   private final DMatrixSparseCSC vrpXWaypoints = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC vrpYWaypoints = new DMatrixSparseCSC(0, 1);
   private final DMatrixSparseCSC vrpZWaypoints = new DMatrixSparseCSC(0, 1);

   private final DMatrixRMaj costHessian = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj costGradientX = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj costGradientY = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj costGradientZ = new DMatrixRMaj(1, 1);

   private final NativeMatrix nativeA = new NativeMatrix(1, 1);
   private final NativeMatrix nativebx = new NativeMatrix(1, 1);
   private final NativeMatrix nativeby = new NativeMatrix(1, 1);
   private final NativeMatrix nativebz = new NativeMatrix(1, 1);
   private final NativeMatrix nativeQ = new NativeMatrix(1, 1);
   private final NativeMatrix nativeqx = new NativeMatrix(1, 1);
   private final NativeMatrix nativeqy = new NativeMatrix(1, 1);
   private final NativeMatrix nativeqz = new NativeMatrix(1, 1);
   private final NativeMatrix nativeQInverse = new NativeMatrix(1, 1);

   private final NativeMatrix AQinverse = new NativeMatrix(1, 1);
   private final NativeMatrix AQinverseATranspose = new NativeMatrix(1, 1);
   private final NativeMatrix lagrangeMultiplierXObjective = new NativeMatrix(1, 1);
   private final NativeMatrix lagrangeMultiplierYObjective = new NativeMatrix(1, 1);
   private final NativeMatrix lagrangeMultiplierZObjective = new NativeMatrix(1, 1);
   private final NativeMatrix lagrangeMultiplierXSolution = new NativeMatrix(1, 1);
   private final NativeMatrix lagrangeMultiplierYSolution = new NativeMatrix(1, 1);
   private final NativeMatrix lagrangeMultiplierZSolution = new NativeMatrix(1, 1);

   private final NativeMatrix xCoefficientSolution = new NativeMatrix(1, 1);
   private final NativeMatrix yCoefficientSolution = new NativeMatrix(1, 1);
   private final NativeMatrix zCoefficientSolution = new NativeMatrix(1, 1);

   private final DMatrixRMaj xCoefficientVector = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj yCoefficientVector = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj zCoefficientVector = new DMatrixRMaj(0, 1);

   private final CoMTrajectoryPlannerIndexHandler indexHandler = new CoMTrajectoryPlannerIndexHandler();

   private final RecyclingArrayList<FramePoint3D> startVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> endVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);

   private final DoubleProvider omega;
   private final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final double gravityZ;

   private final LinearCoMTrajectoryHandler debugTrajectory;

   private int numberOfConstraints = 0;

   public CoMOptimalContinuityCalculator(double gravityZ, double nominalCoMHeight, YoRegistry parentRegistry)
   {
      this.gravityZ = Math.abs(gravityZ);
      YoDouble omega = new YoDouble("omegaForPlanning", registry);

      comHeight.addListener(v -> omega.set(Math.sqrt(Math.abs(gravityZ) / comHeight.getDoubleValue())));
      comHeight.set(nominalCoMHeight);

      this.omega = omega;

      if (CREATE_DEBUG_TRAJECTORY)
         debugTrajectory = new LinearCoMTrajectoryHandler(registry);
      else
         debugTrajectory = null;

      parentRegistry.addChild(registry);
   }

   public CoMOptimalContinuityCalculator(double gravityZ, YoDouble omega, YoRegistry parentRegistry)
   {
      this.omega = omega;
      this.gravityZ = Math.abs(gravityZ);

      omega.addListener(v -> comHeight.set(gravityZ / MathTools.square(omega.getValue())));
      omega.notifyListeners();

      if (CREATE_DEBUG_TRAJECTORY)
         debugTrajectory = new LinearCoMTrajectoryHandler(registry);
      else
         debugTrajectory = null;

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

      resetMatrices(7);

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
      double secondSegmentDuration = Math.min(contactSequenceInternal.get(1).getTimeInterval().getDuration(), CoMTrajectoryPlannerTools.sufficientlyLarge);

      FramePoint3DReadOnly startVRPPosition = startVRPPositions.get(0);
      FramePoint3DReadOnly endVRPPosition = endVRPPositions.get(1);

      // set the continuity objective for the initial continuity
      addCoMVelocityObjective(5.0, 0, 0.0, initialCoMVelocity);
      // encourage the VRP motions to be linear and smooth
      addVRPSmoothnessObjective(50.0, 0, firstSegmentDuration);
      addVRPSmoothnessObjective(50.0, 1, secondSegmentDuration);
      // set an objective for the midpoint
      addVRPPositionObjective(1.0, 0, firstSegmentDuration, endVRPPositions.get(0));

      // set the initial CoM state
      addCoMPositionConstraint(0, 0.0, initialCoMPosition);

      // constrain the initial VRP position
      addVRPPositionConstraint(0, indexHandler.getVRPWaypointStartPositionIndex(0), 0.0, startVRPPosition);

      // set the continuity for the knot point.
      addCoMPositionContinuityConstraint(firstSegmentDuration);
      addCoMVelocityContinuityConstraint(firstSegmentDuration);
      addVRPPositionContinuityConstraint(firstSegmentDuration);

      // set the final VRP position
      addVRPPositionConstraint(1, indexHandler.getVRPWaypointFinalPositionIndex(1), secondSegmentDuration, endVRPPosition);

      // set terminal CoM position and velocity
      addFinalDCMPositionConstraint(secondSegmentDuration, finalICPToAchieve);

      solve();

      if (debugTrajectory != null)
         debugTrajectory.setCoefficientsFromSolution(omega.getValue(), contactSequenceInternal, xCoefficientVector, yCoefficientVector, zCoefficientVector);

      return true;
   }

   private void solve()
   {

      // move over the cost terms
      nativeQ.set(costHessian);
      nativeQ.addDiagonal(1e-5);

      nativeqx.set(costGradientX);
      nativeqy.set(costGradientY);
      nativeqz.set(costGradientZ);
      nativeQInverse.invert(nativeQ);

      // move over the constraint terms
      nativeA.set(constraintAMatrix);
      nativebx.set(xConstraintObjective);
      nativeby.set(yConstraintObjective);
      nativebz.set(zConstraintObjective);

      AQinverse.mult(nativeA, nativeQInverse);
      AQinverseATranspose.multTransB(AQinverse, nativeA);

      lagrangeMultiplierXObjective.set(nativebx);
      lagrangeMultiplierYObjective.set(nativeby);
      lagrangeMultiplierZObjective.set(nativebz);
      lagrangeMultiplierXObjective.multAdd(AQinverse, nativeqx);
      lagrangeMultiplierYObjective.multAdd(AQinverse, nativeqy);
      lagrangeMultiplierZObjective.multAdd(AQinverse, nativeqz);

      lagrangeMultiplierXSolution.solve(AQinverseATranspose, lagrangeMultiplierXObjective);
      lagrangeMultiplierYSolution.solve(AQinverseATranspose, lagrangeMultiplierYObjective);
      lagrangeMultiplierZSolution.solve(AQinverseATranspose, lagrangeMultiplierZObjective);
      // todo maybe apply to A Q A^T
      lagrangeMultiplierXSolution.scale(-1.0);
      lagrangeMultiplierYSolution.scale(-1.0);
      lagrangeMultiplierZSolution.scale(-1.0);

      xCoefficientSolution.mult(-1.0, nativeQInverse, nativeqx);
      yCoefficientSolution.mult(-1.0, nativeQInverse, nativeqy);
      zCoefficientSolution.mult(-1.0, nativeQInverse, nativeqz);

      xCoefficientSolution.multAddTransA(-1.0, AQinverse, lagrangeMultiplierXSolution);
      yCoefficientSolution.multAddTransA(-1.0, AQinverse, lagrangeMultiplierYSolution);
      zCoefficientSolution.multAddTransA(-1.0, AQinverse, lagrangeMultiplierZSolution);

      // get the solution out
      xCoefficientSolution.get(xCoefficientVector);
      yCoefficientSolution.get(yCoefficientVector);
      zCoefficientSolution.get(zCoefficientVector);
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

   private void resetMatrices(int constraints)
   {
      int size = indexHandler.getTotalNumberOfCoefficients();
      int numberOfVRPWaypoints = indexHandler.getNumberOfVRPWaypoints();

      constraintAMatrix.reshape(constraints, size);
      xConstraintObjective.reshape(constraints, 1);
      yConstraintObjective.reshape(constraints, 1);
      zConstraintObjective.reshape(constraints, 1);
      vrpWaypointJacobian.reshape(size, numberOfVRPWaypoints); // only position
      vrpXWaypoints.reshape(numberOfVRPWaypoints, 1);
      vrpYWaypoints.reshape(numberOfVRPWaypoints, 1);
      vrpZWaypoints.reshape(numberOfVRPWaypoints, 1);
      xCoefficientVector.reshape(size, 1);
      yCoefficientVector.reshape(size, 1);
      zCoefficientVector.reshape(size, 1);

      costHessian.reshape(size, size);
      costGradientX.reshape(size, 1);
      costGradientY.reshape(size, 1);
      costGradientZ.reshape(size, 1);

      constraintAMatrix.zero();
      xConstraintObjective.zero();
      yConstraintObjective.zero();
      zConstraintObjective.zero();
      vrpWaypointJacobian.zero();
      vrpXWaypoints.zero();
      vrpYWaypoints.zero();
      vrpZWaypoints.zero();

      costHessian.zero();
      costGradientX.zero();
      costGradientY.zero();
      costGradientZ.zero();

      tempJacobian.reshape(1, size);
      tempXObjective.reshape(1, 1);
      tempYObjective.reshape(1, 1);
      tempZObjective.reshape(1, 1);

   }

   private void addCoMVelocityObjective(double weight, int segmentId, double timeForConstraint, FrameVector3DReadOnly centerOfMassVelocityObjective)
   {
      tempJacobian.zero();
      tempXObjective.zero();
      tempYObjective.zero();
      tempZObjective.zero();
      CoMTrajectoryPlannerTools.addCoMVelocityObjective(weight,
                                                        centerOfMassVelocityObjective,
                                                        omega.getValue(),
                                                        timeForConstraint,
                                                        segmentId,
                                                        0,
                                                        tempJacobian,
                                                        tempXObjective,
                                                        tempYObjective,
                                                        tempZObjective);

      addObjective(tempJacobian, tempXObjective, tempYObjective, tempZObjective);
   }

   private void addVRPPositionObjective(double weight, int segmentId, double timeForConstraint, FramePoint3DReadOnly vrpPositionObjective)
   {
      tempJacobian.zero();
      tempXObjective.zero();
      tempYObjective.zero();
      tempZObjective.zero();
      CoMTrajectoryPlannerTools.addVRPPositionObjective(weight,
                                                        vrpPositionObjective,
                                                        omega.getValue(),
                                                        timeForConstraint,
                                                        segmentId,
                                                        0,
                                                        tempJacobian,
                                                        tempXObjective,
                                                        tempYObjective,
                                                        tempZObjective);

      addObjective(tempJacobian, tempXObjective, tempYObjective, tempZObjective);
   }

   private void addVRPSmoothnessObjective(double weight, int segmentId, double segmentDuration)
   {
      tempJacobian.zero();
      CoMTrajectoryPlannerTools.addSegmentVRPSmoothnessObjective(weight, segmentId, omega.getValue(), segmentDuration, 0, tempJacobian);

      MatrixTools.multAddInner(1.0, tempJacobian, costHessian);
   }

   private void addObjective(DMatrixRMaj jacobian, DMatrixRMaj xObjective, DMatrixRMaj yObjective, DMatrixRMaj zObjective)
   {
      // Q += w J^T J
      MatrixTools.multAddInner(1.0, jacobian, costHessian);

      // q += w J b^T
      CommonOps_DDRM.multAddTransA(-1.0, jacobian, xObjective, costGradientX);
      CommonOps_DDRM.multAddTransA(-1.0, jacobian, yObjective, costGradientY);
      CommonOps_DDRM.multAddTransA(-1.0, jacobian, zObjective, costGradientZ);
   }

   private void addCoMPositionConstraint(int segmentId, double timeForConstraint, FramePoint3DReadOnly centerOfMassLocationForConstraint)
   {
      CoMTrajectoryPlannerTools.addCoMPositionConstraint(centerOfMassLocationForConstraint,
                                                         omega.getValue(),
                                                         timeForConstraint,
                                                         segmentId,
                                                         numberOfConstraints++,
                                                         constraintAMatrix,
                                                         xConstraintObjective,
                                                         yConstraintObjective,
                                                         zConstraintObjective);
   }

   private void setCoMVelocityConstraint(int segmentId, double timeForConstraint, FrameVector3DReadOnly centerOfMassVelocityForConstraint)
   {
      CoMTrajectoryPlannerTools.addCoMVelocityConstraint(centerOfMassVelocityForConstraint,
                                                         omega.getValue(),
                                                         timeForConstraint,
                                                         segmentId,
                                                         numberOfConstraints++,
                                                         constraintAMatrix,
                                                         xConstraintObjective,
                                                         yConstraintObjective,
                                                         zConstraintObjective);
   }

   private void addCoMPositionContinuityConstraint(double previousDuration)
   {
      CoMTrajectoryPlannerTools.addCoMPositionContinuityConstraint(0, 1, numberOfConstraints++, omega.getValue(), previousDuration, constraintAMatrix);
   }

   private void addCoMVelocityContinuityConstraint(double previousDuration)
   {
      CoMTrajectoryPlannerTools.addCoMVelocityContinuityConstraint(0, 1, numberOfConstraints++, omega.getValue(), previousDuration, constraintAMatrix);
   }

   private void addVRPPositionConstraint(int segmentId, int vrpWaypointPositionIndex, double time, FramePoint3DReadOnly desiredVRPPosition)
   {
      CoMTrajectoryPlannerTools.addVRPPositionConstraint(segmentId,
                                                         numberOfConstraints++,
                                                         vrpWaypointPositionIndex,
                                                         time,
                                                         omega.getValue(),
                                                         desiredVRPPosition,
                                                         constraintAMatrix,
                                                         vrpXWaypoints,
                                                         vrpYWaypoints,
                                                         vrpZWaypoints,
                                                         vrpWaypointJacobian);
   }

   private void addVRPPositionContinuityConstraint(double previousDuration)
   {
      CoMTrajectoryPlannerTools.addVRPPositionContinuityConstraint(0, 1, numberOfConstraints++, omega.getValue(), previousDuration, constraintAMatrix);
   }

   private void addFinalDCMPositionConstraint(double timeInSegment, FramePoint3DReadOnly desiredDCMPosition)
   {
      CoMTrajectoryPlannerTools.addDCMPositionConstraint(1, numberOfConstraints++, timeInSegment, omega.getValue(), desiredDCMPosition,
                                                         constraintAMatrix, xConstraintObjective, yConstraintObjective, zConstraintObjective);
   }

   LinearCoMTrajectoryHandler getDebugTrajectory()
   {
      return debugTrajectory;
   }
}
