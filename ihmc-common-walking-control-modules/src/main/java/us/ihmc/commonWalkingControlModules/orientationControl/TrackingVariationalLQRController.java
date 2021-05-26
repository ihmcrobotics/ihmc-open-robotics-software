package us.ihmc.commonWalkingControlModules.orientationControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.tools.MPCAngleTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class TrackingVariationalLQRController
{
   private static final double discreteDt = 5e-3;

   private static final double defaultQR = 1100;
   private static final double defaultQw = 5;
   private static final double defaultR = 1.25;

   private final MPCAngleTools angleTools = new MPCAngleTools();

   private final VariationalCommonValues commonValues = new VariationalCommonValues();

   private final Vector3DBasics axisAngleError = new Vector3D();

   private final DMatrixRMaj intermediateP = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj intermediateK = new DMatrixRMaj(3, 6);

   private final DMatrixRMaj activeP = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj activeK = new DMatrixRMaj(3, 6);

   private final DMatrixRMaj wBd = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj wB = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj RBerrorVector = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj wBerror = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj state = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj desiredTorque = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj feedbackTorque = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj deltaTau = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj inertia = new DMatrixRMaj(3, 3);

   private final Vector3D desiredBodyVelocityInBodyFrame = new Vector3D();
   private final AlgebraicVariationalFunction finalPFunction = new AlgebraicVariationalFunction();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator(
         "lqrOrientationTrajectory",
         ReferenceFrame.getWorldFrame(),
         registry);

   private final MultipleWaypointsPositionTrajectoryGenerator angularMomentumTrajectory = new MultipleWaypointsPositionTrajectoryGenerator(
         "lqrAngularMomentumTrajectory",
         ReferenceFrame.getWorldFrame(),
         registry);
   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   private final List<VariationalFunction> reversedPFunctionList = new ArrayList<>();
   private final List<VariationalFunction> pFunctionList = new ArrayList<>();

   public TrackingVariationalLQRController()
   {
      CommonOps_DDRM.setIdentity(inertia);
      commonValues.setInertia(inertia);

      commonValues.computeCostMatrices(defaultQR, defaultQw, defaultR);
   }

   public void setInertia(SpatialInertiaReadOnly inertia)
   {
      inertia.getMomentOfInertia().get(this.inertia);
      commonValues.setInertia(this.inertia);
   }

   public void setTrajectories(MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectory,
                               MultipleWaypointsPositionTrajectoryGenerator angularMomentumTrajectory,
                               List<? extends ContactStateProvider<?>> contactStateProviders)
   {
      this.orientationTrajectory.clear();
      this.angularMomentumTrajectory.clear();
      this.contactStateProviders.clear();
      for (int i = 0; i < orientationTrajectory.getCurrentNumberOfWaypoints(); i++)
         this.orientationTrajectory.appendWaypoint(orientationTrajectory.getWaypoint(i));
      for (int i = 0; i < angularMomentumTrajectory.getCurrentNumberOfWaypoints(); i++)
         this.angularMomentumTrajectory.appendWaypoint(angularMomentumTrajectory.getWaypoint(i));
      for (int i = 0; i < contactStateProviders.size(); i++)
         this.contactStateProviders.add().set(contactStateProviders.get(i));

      double finalTime = this.contactStateProviders.getLast().getTimeInterval().getEndTime();
      orientationTrajectory.compute(finalTime);
      angularMomentumTrajectory.compute(finalTime);

      desiredBodyVelocityInBodyFrame.set(orientationTrajectory.getAngularVelocity());
      orientationTrajectory.getOrientation().transform(desiredBodyVelocityInBodyFrame);

      finalPFunction.setDesired(orientationTrajectory.getOrientation(), desiredBodyVelocityInBodyFrame, angularMomentumTrajectory.getVelocity(), commonValues);

      computePSegments();
   }

   public void compute(double currentTime, QuaternionReadOnly currentRotation, Vector3DReadOnly currentAngularVelocityInBodyFrame)
   {
      currentAngularVelocityInBodyFrame.get(wB);

      int activeSegment = getActiveSegment(currentTime);
      double timeInSegment = currentTime - contactStateProviders.get(activeSegment).getTimeInterval().getStartTime();
      pFunctionList.get(activeSegment).compute(timeInSegment, activeP, activeK);

      orientationTrajectory.compute(currentTime);
      desiredBodyVelocityInBodyFrame.set(orientationTrajectory.getAngularVelocity());
      orientationTrajectory.getOrientation().transform(desiredBodyVelocityInBodyFrame);
      desiredBodyVelocityInBodyFrame.get(wBd);

      angleTools.computeRotationError(orientationTrajectory.getOrientation(), currentRotation, axisAngleError);
      axisAngleError.get(RBerrorVector);

      CommonOps_DDRM.subtract(wB, wBd, wBerror);

      MatrixTools.setMatrixBlock(state, 0, 0, RBerrorVector, 0, 0, 3, 1, 1.0);
      MatrixTools.setMatrixBlock(state, 3, 0, wBerror, 0, 0, 3, 1, 1.0);

      CommonOps_DDRM.mult(-1.0, activeK, state, deltaTau);

      CommonOps_DDRM.add(deltaTau, desiredTorque, feedbackTorque);
   }

   public void getDesiredTorque(Vector3DBasics tau)
   {
      tau.set(this.feedbackTorque);
   }

   private int getActiveSegment(double currentTime)
   {
      for (int i = 0; i < contactStateProviders.size(); i++)
      {
         if (contactStateProviders.get(i).getTimeInterval().intervalContains(currentTime))
            return i;
      }

      return contactStateProviders.size() - 1;
   }

   private void computePSegments()
   {
      reversedPFunctionList.clear();
      pFunctionList.clear();

      int numberOfSegments = contactStateProviders.size() - 1;

      if (numberOfSegments < 0)
      {
         reversedPFunctionList.add(finalPFunction);
      }
      else
      {
         finalPFunction.compute(0.0, intermediateP, intermediateK);
         reversedPFunctionList.add(finalPFunction);

         for (int j = numberOfSegments - 1; j >= 0; j--)
         {
            ContactStateProvider<?> thisContactProvider = contactStateProviders.get(j);

            DifferentialVariationalSegment pSegment = new DifferentialVariationalSegment(discreteDt);
            pSegment.set(commonValues,
                         orientationTrajectory,
                         angularMomentumTrajectory,
                         intermediateP,
                         thisContactProvider.getTimeInterval().getStartTime(),
                         thisContactProvider.getTimeInterval().getEndTime());

            pSegment.compute(0.0, intermediateP, intermediateK);
            reversedPFunctionList.add(pSegment);
         }
      }

      for (int i = reversedPFunctionList.size() - 1; i >= 0; i--)
         pFunctionList.add(reversedPFunctionList.get(i));
   }
}
