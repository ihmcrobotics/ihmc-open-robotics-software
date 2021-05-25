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

public class DifferentialVariationalSegment implements S1Function
{
   private final double dt;
   private final DMatrixRMaj PB = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj PDot = new DMatrixRMaj(6, 6);

   private final VariationalDynamicsCalculator dynamicsCalculator = new VariationalDynamicsCalculator();
   private final RecyclingArrayList<DMatrixRMaj> PReverseTrajectory = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 6));
   final List<DMatrixRMaj> PTrajectory = new ArrayList<>();

   private final Vector3DBasics angularVelocityInBodyFrame = new Vector3D();

   public DifferentialVariationalSegment(double dt)
   {
      this.dt = dt;
   }

   /*
   public void set(LQRCommonValues lqrCommonValues, DMatrixRMaj S1AtEnd, double duration)
   {
      set(lqrCommonValues.getQ1(),
          lqrCommonValues.getR1Inverse(),
          lqrCommonValues.getNTranspose(),
          lqrCommonValues.getA(),
          lqrCommonValues.getB(),
          S1AtEnd,
          duration);
   }

    */

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
      PReverseTrajectory.clear();
      PReverseTrajectory.add().set(PAtEnd);

      for (double time = endTime - dt; time >= startTime + dt / 10.0; time -= dt)
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

         DMatrixRMaj previousP = PReverseTrajectory.getLast();
         DMatrixRMaj newP = PReverseTrajectory.add();

         CommonOps_DDRM.mult(previousP, dynamicsCalculator.getB(), PB);
         computePDot(Q, PB, RInverse, previousP, dynamicsCalculator.getA());

         CommonOps_DDRM.add(previousP, -dt, PDot, newP);
      }

      for (int i = PReverseTrajectory.size() - 1; i >= 0; i--)
      {
         PTrajectory.add(PReverseTrajectory.get(i));
      }
   }

   @Override
   public void compute(double timeInState, DMatrixRMaj S1ToPack)
   {
      int startIndex = getStartIndex(timeInState);
      DMatrixRMaj start = PTrajectory.get(startIndex);
      if (startIndex == PTrajectory.size() - 1)
      {
         S1ToPack.set(PTrajectory.get(PTrajectory.size() - 1));
         return;
      }
      DMatrixRMaj end = PTrajectory.get(startIndex + 1);
      interpolate(start, end, getAlphaBetweenSegments(timeInState), S1ToPack);
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
