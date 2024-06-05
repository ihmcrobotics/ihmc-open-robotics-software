package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.centerOfMassEstimator;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.MomentumCalculator;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.euclid.YoPoint2D;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * Following the paper titled: "Humanoid Momentum Estimation Using Sensed Contact Wrenches".
 */
public class WrenchBasedMomentumStateUpdater implements MomentumStateUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public enum Estimator
   {
      MomentumEstimator, OffsetEstimator, CoPBasedOffsetEstimator
   };

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final StateEstimator momentumEstimator;

   private final YoPoint3D measuredCoMPosition = new YoPoint3D("kinematicsBasedCoMPosition", registry);
   private final YoVector3D measuredLinearMomentum = new YoVector3D("kinematicsBasedLinearMomentum", registry);
   private final YoVector3D measuredAngularMomentum = new YoVector3D("kinematicsBasedAngularMomentum", registry);
   private final YoPoint2D measuredCenterOfPressure = new YoPoint2D("measuredCenterOfPressure", registry);
   private final CenterOfMassReferenceFrame measuredCoMFrame;
   private final MomentumCalculator momentumCalculator;

   private final YoBoolean hasBeenInitialized = new YoBoolean("hasBeenInitialized", registry);

   private final double mass;
   private final RobotState robotState = new RobotState(null, Collections.emptyList());
   private final Estimator estimator = Estimator.CoPBasedOffsetEstimator;
   private final MomentumEKFEstimatorIndexProvider indexProvider = MomentumEKFEstimatorIndexProvider.newStateIndexProvider(estimator);
   private final MomentumKinematicsBasedEKFSensor sensor;
   private final MomentumEKFState state;

   private final DMatrixRMaj stateVector;

   private final YoBoolean enableOutput = new YoBoolean("enableWrenchBasedMomentumEstimatorOutput", registry);
   private final CenterOfMassDataHolder centerOfMassDataHolder;
   private final YoFrameVector3D centerOfMassVelocity = new YoFrameVector3D("centerOfMassVelocity", worldFrame, registry);

   public WrenchBasedMomentumStateUpdater(JointReadOnly rootJoint,
                                          List<WrenchSensor> wrenchSensors,
                                          double dt,
                                          double gravity,
                                          CenterOfMassDataHolder centerOfMassDataHolder)
   {
      this.centerOfMassDataHolder = centerOfMassDataHolder;
      gravity = Math.abs(gravity);
      mass = TotalMassCalculator.computeSubTreeMass(MultiBodySystemTools.getRootBody(rootJoint.getPredecessor()));
      wrenchSensors = addFrameCorruptors(wrenchSensors, registry); // TODO Remove me
      measuredCoMFrame = new CenterOfMassReferenceFrame("comFrame", worldFrame, rootJoint.getPredecessor());

      state = new MomentumEKFState(indexProvider, measuredCoMFrame, wrenchSensors, mass, gravity, dt, registry);
      sensor = MomentumKinematicsBasedEKFSensor.createEstimator(estimator,
                                                             measuredCoMPosition,
                                                             measuredLinearMomentum,
                                                             measuredAngularMomentum,
                                                             measuredCenterOfPressure,
                                                             indexProvider,
                                                             state.getMomentumRateCalculator(),
                                                             dt,
                                                             registry);

      robotState.addState(state);
      momentumEstimator = new StateEstimator(Collections.singletonList(sensor), robotState, registry);

      momentumCalculator = new MomentumCalculator(rootJoint.getPredecessor());
      stateVector = new DMatrixRMaj(indexProvider.getSize(), 1);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void update()
   {
      updateKinematicsBasedMomemtum();

      if (!hasBeenInitialized.getValue())
      {
         momentumEstimator.reset();
         stateVector.zero();
         if (indexProvider.hasCoMPosition())
            measuredCoMPosition.get(indexProvider.getCoMPosition(), stateVector);
         if (indexProvider.hasLinearMomentum())
            measuredLinearMomentum.get(indexProvider.getLinearMomentum(), stateVector);
         if (indexProvider.hasAngularMomentum())
            measuredAngularMomentum.get(indexProvider.getAngularMomentum(), stateVector);
         robotState.setStateVector(stateVector);
         hasBeenInitialized.set(true);
      }
      else
      {
         momentumEstimator.predict();
         updateCoP();
         momentumEstimator.correct();
      }

      centerOfMassVelocity.setMatchingFrame(state.getCorrectedCoMFrame(), state.getLinearMomentumState());
      centerOfMassVelocity.scale(1.0 / mass);

      if (enableOutput.getValue())
      {
         centerOfMassDataHolder.setCenterOfMassPosition(worldFrame, state.getCenterOfMassPositionState());
         centerOfMassDataHolder.setCenterOfMassVelocity(centerOfMassVelocity);
      }
      else
      {
         centerOfMassDataHolder.clear();
      }
   }

   private final Momentum tempMomentum = new Momentum();

   private void updateKinematicsBasedMomemtum()
   {
      measuredCoMFrame.update();
      measuredCoMPosition.set(measuredCoMFrame.getTransformToRoot().getTranslation());
      tempMomentum.setReferenceFrame(measuredCoMFrame);
      momentumCalculator.computeAndPack(tempMomentum);
      measuredLinearMomentum.set(tempMomentum.getLinearPart());
      measuredAngularMomentum.set(tempMomentum.getAngularPart());
   }

   private void updateCoP()
   {
      computeCoP(measuredCoMPosition, state.getMomentumRateCalculator().getTotalSpatialForceAtCoM(), measuredCenterOfPressure);
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   public static void computeCoP(Point3DReadOnly centerOfMass, SpatialForceReadOnly spatialForce, Tuple2DBasics copToPack)
   {
      computeCoP(centerOfMass, spatialForce.getLinearPart(), spatialForce.getAngularPart(), copToPack);
   }

   public static void computeCoP(Point3DReadOnly centerOfMass, Vector3DReadOnly force, Vector3DReadOnly torque, Tuple2DBasics copToPack)
   {
      if (EuclidCoreTools.isZero(force.getZ(), 1.0e-3))
      {
         copToPack.set(centerOfMass);
      }
      else
      {
         copToPack.setX(centerOfMass.getX() - (centerOfMass.getZ() * force.getX() + torque.getY()) / force.getZ());
         copToPack.setY(centerOfMass.getY() - (centerOfMass.getZ() * force.getY() - torque.getX()) / force.getZ());
      }
   }

   /**
    * Inserts the following 3-by-3 skew-matrix in {@code matrix}:
    * 
    * <pre>
    *        /  0 -z  y \
    * skew = |  z  0 -x |
    *        \ -y  x  0 /
    * </pre>
    * 
    * where x, y, and z are the components of the given {@code vector}.
    */
   public static void insertSkewMatrix(int rowStart, int colStart, Tuple3DReadOnly vector, DMatrixRMaj matrix)
   {
      insertSkewMatrix(rowStart, colStart, vector.getX(), vector.getY(), vector.getZ(), matrix);
   }

   public static void insertSkewMatrix(int rowStart, int colStart, double scale, Tuple3DReadOnly vector, DMatrixRMaj matrix)
   {
      insertSkewMatrix(rowStart, colStart, scale * vector.getX(), scale * vector.getY(), scale * vector.getZ(), matrix);
   }

   /**
    * Inserts the following 3-by-3 skew-matrix in {@code matrix}:
    * 
    * <pre>
    *        /  0 -z  y \
    * skew = |  z  0 -x |
    *        \ -y  x  0 /
    * </pre>
    */
   public static void insertSkewMatrix(int rowStart, int colStart, double x, double y, double z, DMatrixRMaj matrix)
   {
      EuclidCoreTools.checkMatrixMinimumSize(rowStart + 3, colStart + 3, matrix);

      matrix.set(rowStart + 0, colStart + 0, 0.0);
      matrix.set(rowStart + 0, colStart + 1, -z);
      matrix.set(rowStart + 0, colStart + 2, +y);
      matrix.set(rowStart + 1, colStart + 0, +z);
      matrix.set(rowStart + 1, colStart + 1, 0.0);
      matrix.set(rowStart + 1, colStart + 2, -x);
      matrix.set(rowStart + 2, colStart + 0, -y);
      matrix.set(rowStart + 2, colStart + 1, +x);
      matrix.set(rowStart + 2, colStart + 2, 0.0);
   }

   public static void setDiagonalElements(int startRow, int startCol, int numberElements, double value, DMatrix1Row matrix)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + numberElements, startCol + numberElements, matrix);

      for (int i = 0; i < numberElements; i++)
      {
         matrix.unsafe_set(startRow + i, startCol + i, value);
      }
   }

   public static void addEquals(Tuple2DBasics a, DMatrix1Row b, int startRow, int column)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 1, column, b);
      a.addX(b.unsafe_get(startRow, column));
      a.addY(b.unsafe_get(startRow + 1, column));
   }

   public static void addEquals(Tuple3DBasics a, DMatrix1Row b, int startRow, int column)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 2, column, b);
      a.addX(b.unsafe_get(startRow, column));
      a.addY(b.unsafe_get(startRow + 1, column));
      a.addZ(b.unsafe_get(startRow + 2, column));
   }

   public static interface WrenchSensor
   {
      ReferenceFrame getMeasurementFrame();

      void getMeasuredWrench(Wrench wrenchToPack);
   }

   public static List<WrenchSensor> addFrameCorruptors(List<WrenchSensor> wrenchSensors, YoRegistry registry)
   {
      return wrenchSensors.stream().map(sensor -> addFrameCorruptor(sensor, registry)).collect(Collectors.toList());
   }

   public static WrenchSensor addFrameCorruptor(WrenchSensor wrenchSensor, YoRegistry registry)
   {
      ReferenceFrame measurementFrame = wrenchSensor.getMeasurementFrame();
      YoFramePoseUsingYawPitchRoll offset = new YoFramePoseUsingYawPitchRoll(measurementFrame.getName() + "CorruptOffset", measurementFrame, registry);

      ReferenceFrame corruptedFrame = new ReferenceFrame(measurementFrame.getName() + "Corrupted", measurementFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            offset.get(transformToParent);
         }
      };

      offset.attachVariableChangedListener(v -> corruptedFrame.update());

      return new WrenchSensor()
      {
         @Override
         public ReferenceFrame getMeasurementFrame()
         {
            return corruptedFrame;
         }

         @Override
         public void getMeasuredWrench(Wrench wrenchToPack)
         {
            wrenchSensor.getMeasuredWrench(wrenchToPack);
            wrenchToPack.setBodyFrame(corruptedFrame);
            wrenchToPack.setReferenceFrame(corruptedFrame);
         }
      };
   }

   public static List<WrenchSensor> wrapFootSwitchInterfaces(List<FootSwitchInterface> footSwitchInterfaces)
   {
      return footSwitchInterfaces.stream().map(WrenchBasedMomentumStateUpdater::wrapFootSwitchInterface).collect(Collectors.toList());
   }

   public static WrenchSensor wrapFootSwitchInterface(FootSwitchInterface footSwitchInterface)
   {
      return new WrenchSensor()
      {
         @Override
         public ReferenceFrame getMeasurementFrame()
         {
            return footSwitchInterface.getMeasurementFrame();
         }

         @Override
         public void getMeasuredWrench(Wrench wrenchToPack)
         {
            footSwitchInterface.getMeasuredWrench(wrenchToPack);
         }
      };
   }
}
