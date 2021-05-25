package us.ihmc.commonWalkingControlModules.orientationControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.S1Function;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;

public class DifferentialVariationalSegment implements VariationalFunction
{
   private final double dt;
   private final DMatrixRMaj PB = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj BTransposeP = new DMatrixRMaj(3, 6);
   private final DMatrixRMaj PDot = new DMatrixRMaj(6, 6);

   private final VariationalDynamicsCalculator dynamicsCalculator = new VariationalDynamicsCalculator();
   private final RecyclingArrayList<DMatrixRMaj> PReverseTrajectory = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 6));
   private final RecyclingArrayList<DMatrixRMaj> KReverseTrajectory = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 6));
   final List<DMatrixRMaj> PTrajectory = new ArrayList<>();
   final List<DMatrixRMaj> KTrajectory = new ArrayList<>();

   private final Vector3DBasics angularVelocityInBodyFrame = new Vector3D();

   public DifferentialVariationalSegment(double dt)
   {
      this.dt = dt;
   }


   public void set(VariationalCommonValues commonValues,
                   MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectory,
                   MultipleWaypointsPositionTrajectoryGenerator angularMomentumTrajectory,
                   DMatrixRMaj PAtEnd,
                   double startTime,
                   double endTime)
   {
      set(commonValues.getQ(),
          commonValues.getRInverse(),
          commonValues.getInertia(),
          commonValues.getInertiaInverse(),
          orientationTrajectory,
          angularMomentumTrajectory,
          PAtEnd,
          startTime,
          endTime);
   }

   public void set(DMatrixRMaj Q,
                   DMatrixRMaj RInverse,
                   DMatrixRMaj inertia,
                   DMatrixRMaj inertiaInverse,
                   MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectory,
                   MultipleWaypointsPositionTrajectoryGenerator angularMomentumTrajectory,
                   DMatrixRMaj PAtEnd,
                   double startTime,
                   double endTime)
   {
      PTrajectory.clear();
      KTrajectory.clear();
      PReverseTrajectory.clear();
      KReverseTrajectory.clear();
      PReverseTrajectory.add().set(PAtEnd);

      computeDesireds(endTime, inertia, inertiaInverse, orientationTrajectory, angularMomentumTrajectory);
      computeGainMatrix(dynamicsCalculator.getB(), PAtEnd, RInverse, KReverseTrajectory.add());

      for (double time = endTime - dt; time >= startTime + dt / 10.0; time -= dt)
      {
         computeDesireds(time, inertia, inertiaInverse, orientationTrajectory, angularMomentumTrajectory);

         DMatrixRMaj previousP = PReverseTrajectory.getLast();
         DMatrixRMaj newP = PReverseTrajectory.add();

         CommonOps_DDRM.mult(previousP, dynamicsCalculator.getB(), PB);
         computePDot(Q, PB, RInverse, previousP, dynamicsCalculator.getA());

         CommonOps_DDRM.add(previousP, -dt, PDot, newP);

         computeGainMatrix(dynamicsCalculator.getB(), newP, RInverse, KReverseTrajectory.add());
      }

      for (int i = PReverseTrajectory.size() - 1; i >= 0; i--)
      {
         PTrajectory.add(PReverseTrajectory.get(i));
         KTrajectory.add(KReverseTrajectory.get(i));
      }
   }

   private void computeDesireds(double time,
                                DMatrixRMaj inertia,
                                DMatrixRMaj inertiaInverse,
                                MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectory,
                                MultipleWaypointsPositionTrajectoryGenerator angularMomentumTrajectory)
   {
      orientationTrajectory.compute(time);
      angularMomentumTrajectory.compute(time);

      FrameQuaternionReadOnly desiredOrientation = orientationTrajectory.getOrientation();
      angularVelocityInBodyFrame.set(orientationTrajectory.getAngularVelocity());
      desiredOrientation.transform(angularVelocityInBodyFrame);

      dynamicsCalculator.compute(desiredOrientation,
                                 angularVelocityInBodyFrame,
                                 angularMomentumTrajectory.getVelocity(),
                                 inertia,
                                 inertiaInverse);
   }

   private void computeGainMatrix(DMatrixRMaj B, DMatrixRMaj P, DMatrixRMaj RInverse, DMatrixRMaj KMatrixToPack)
   {
      CommonOps_DDRM.mult(B, P, BTransposeP);
      CommonOps_DDRM.mult(RInverse, BTransposeP, KMatrixToPack);
   }

   @Override
   public void compute(double timeInState, DMatrixRMaj PToPack, DMatrixRMaj KToPack)
   {
      int startIndex = getStartIndex(timeInState);
      DMatrixRMaj startP = PTrajectory.get(startIndex);
      DMatrixRMaj startK = KTrajectory.get(startIndex);
      if (startIndex == PTrajectory.size() - 1)
      {
         PToPack.set(PTrajectory.get(PTrajectory.size() - 1));
         KToPack.set(KTrajectory.get(KTrajectory.size() - 1));
         return;
      }
      DMatrixRMaj endP = PTrajectory.get(startIndex + 1);
      DMatrixRMaj endK = KTrajectory.get(startIndex + 1);
      double alpha = getAlphaBetweenSegments(timeInState);
      interpolate(startP, endP, alpha, PToPack);
      interpolate(startK, endK, alpha, KToPack);
   }

   int getStartIndex(double timeInState)
   {
      return (int) Math.floor(timeInState / dt + dt / 10.0);
   }

   double getAlphaBetweenSegments(double timeInState)
   {
      return (timeInState % dt) / dt;
   }

   private static void interpolate(DMatrixRMaj start, DMatrixRMaj end, double alpha, DMatrixRMaj ret)
   {
      CommonOps_DDRM.scale(1.0 - alpha, start, ret);
      CommonOps_DDRM.addEquals(ret, alpha, end);
   }

   private void computePDot(DMatrixRMaj Q, DMatrixRMaj PB, DMatrixRMaj RInverse, DMatrixRMaj P, DMatrixRMaj A)
   {
      computePDot(Q, PB, RInverse, P, A, PDot);
   }

   static void computePDot(DMatrixRMaj Q, DMatrixRMaj PB, DMatrixRMaj RInverse, DMatrixRMaj P, DMatrixRMaj A, DMatrixRMaj PDotToPack)
   {
      NativeCommonOps.multQuad(PB, RInverse, PDotToPack);
      CommonOps_DDRM.addEquals(PDotToPack, -1.0, Q);
      CommonOps_DDRM.multAdd(-1.0, P, A, PDotToPack);
      CommonOps_DDRM.multAddTransA(-1.0, A, P, PDotToPack);
   }
}
