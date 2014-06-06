package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.sensors.WrenchBasedFootSwitch;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCStateEstimatorInterface;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.imu.FusedIMUSensor;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;

public class DRCKinematicsBasedStateEstimator implements DRCStateEstimatorInterface, StateEstimator
{
   public static final boolean INITIALIZE_HEIGHT_WITH_FOOT = true;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final DoubleYoVariable yoTime = new DoubleYoVariable("t_stateEstimator", registry);

   private final FusedIMUSensor fusedIMUSensor;
   private final JointStateUpdater jointStateUpdater;
   private final PelvisRotationalStateUpdater pelvisRotationalStateUpdater;
   private final PelvisLinearStateUpdater pelvisLinearStateUpdater;

   private final double estimatorDT;

   private boolean visualizeMeasurementFrames = false;
   private final ArrayList<DynamicGraphicReferenceFrame> dynamicGraphicMeasurementFrames = new ArrayList<>();

   private final CenterOfPressureVisualizer copVisualizer;

   public DRCKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, StateEstimatorParameters stateEstimatorParameters,
         SensorOutputMapReadOnly sensorOutputMapReadOnly, double gravitationalAcceleration, SideDependentList<WrenchBasedFootSwitch> footSwitches,
         SideDependentList<ContactablePlaneBody> bipedFeet, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.estimatorDT = stateEstimatorParameters.getEstimatorDT();

      jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorOutputMapReadOnly, registry);

      List<? extends IMUSensorReadOnly> imuProcessedOutputs = sensorOutputMapReadOnly.getIMUProcessedOutputs();
      List<IMUSensorReadOnly> imusToUse = new ArrayList<>();

      if (stateEstimatorParameters.createFusedIMUSensor())
      {
         if (imuProcessedOutputs.size() != 2)
            throw new RuntimeException("Cannot create FusedIMUSensor.");
         fusedIMUSensor = new FusedIMUSensor(imuProcessedOutputs.get(0), imuProcessedOutputs.get(1), estimatorDT, stateEstimatorParameters.getIMUDriftFilterFreqInHertz(), registry);
         imusToUse.add(fusedIMUSensor);
      }
      else
      {
         fusedIMUSensor = null;
         imusToUse.addAll(imuProcessedOutputs);
      }

      pelvisRotationalStateUpdater = new PelvisRotationalStateUpdater(inverseDynamicsStructure, imusToUse, registry);

      pelvisLinearStateUpdater = new PelvisLinearStateUpdater(inverseDynamicsStructure, imusToUse, footSwitches, bipedFeet, gravitationalAcceleration, yoTime,
            stateEstimatorParameters, dynamicGraphicObjectsListRegistry, registry);

      if (dynamicGraphicObjectsListRegistry != null)
      {
         copVisualizer = new CenterOfPressureVisualizer(footSwitches, dynamicGraphicObjectsListRegistry, registry);
      }
      else
      {
         copVisualizer = null;
      }

      visualizeMeasurementFrames = visualizeMeasurementFrames && dynamicGraphicObjectsListRegistry != null;

      List<IMUSensorReadOnly> imusToDisplay = new ArrayList<>();
      imusToDisplay.addAll(imuProcessedOutputs);
      if (fusedIMUSensor != null)
         imusToDisplay.add(fusedIMUSensor);

      if (visualizeMeasurementFrames)
         setupDynamicGraphicObjects(dynamicGraphicObjectsListRegistry, imusToDisplay);
   }

   private void setupDynamicGraphicObjects(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
         List<? extends IMUSensorReadOnly> imuProcessedOutputs)
   {
      for (int i = 0; i < imuProcessedOutputs.size(); i++)
      {
         DynamicGraphicReferenceFrame dynamicGraphicMeasurementFrame = new DynamicGraphicReferenceFrame(imuProcessedOutputs.get(i).getMeasurementFrame(), registry, 1.0);
         dynamicGraphicMeasurementFrames.add(dynamicGraphicMeasurementFrame);
      }
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("imuFrame", dynamicGraphicMeasurementFrames);
   }

   public StateEstimator getStateEstimator()
   {
      return this;
   }

   public void initialize()
   {
      if (fusedIMUSensor != null)
         fusedIMUSensor.update();

      jointStateUpdater.initialize();
      pelvisRotationalStateUpdater.initialize();
      pelvisLinearStateUpdater.initialize();
   }

   public void doControl()
   {
      yoTime.add(estimatorDT); //Hack to have a yoTime in the state estimator

      if (fusedIMUSensor != null)
         fusedIMUSensor.update();

      jointStateUpdater.updateJointState();
      pelvisRotationalStateUpdater.updateRootJointOrientationAndAngularVelocity();
      pelvisLinearStateUpdater.updateRootJointPositionAndLinearVelocity();

      updateVisualizers();
   }

   private void updateVisualizers()
   {
      if (copVisualizer != null)
         copVisualizer.update();

      if (visualizeMeasurementFrames)
      {
         for (int i = 0; i < dynamicGraphicMeasurementFrames.size(); i++)
            dynamicGraphicMeasurementFrames.get(i).update();
      }
   }

   public void initializeEstimatorToActual(Point3d initialCoMPosition, Quat4d initialEstimationLinkOrientation)
   {
      pelvisLinearStateUpdater.initializeCoMPositionToActual(initialCoMPosition);
      // Do nothing for the orientation since the IMU is trusted
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void getEstimatedOrientation(FrameOrientation estimatedOrientationToPack)
   {
      pelvisRotationalStateUpdater.getEstimatedOrientation(estimatedOrientationToPack);
   }

   public void setEstimatedOrientation(FrameOrientation estimatedOrientation)
   {
      // Do nothing, IMU is trusted
   }

   public void getEstimatedAngularVelocity(FrameVector estimatedAngularVelocityToPack)
   {
      pelvisRotationalStateUpdater.getEstimatedAngularVelocity(estimatedAngularVelocityToPack);
   }

   public void setEstimatedAngularVelocity(FrameVector estimatedAngularVelocity)
   {
      // Do nothing, IMU is trusted
   }

   public void getEstimatedCoMPosition(FramePoint estimatedCoMPositionToPack)
   {
      pelvisLinearStateUpdater.getEstimatedCoMPosition(estimatedCoMPositionToPack);
   }

   public void setEstimatedCoMPosition(FramePoint estimatedCoMPosition)
   {
      pelvisLinearStateUpdater.initializeCoMPositionToActual(estimatedCoMPosition);
   }

   public void getEstimatedCoMVelocity(FrameVector estimatedCoMVelocityToPack)
   {
      pelvisLinearStateUpdater.getEstimatedCoMVelocity(estimatedCoMVelocityToPack);
   }

   public void setEstimatedCoMVelocity(FrameVector estimatedCoMVelocity)
   {
   }

   public void getEstimatedPelvisPosition(FramePoint estimatedPelvisPositionToPack)
   {
      pelvisLinearStateUpdater.getEstimatedPelvisPosition(estimatedPelvisPositionToPack);
   }

   public void getEstimatedPelvisLinearVelocity(FrameVector estimatedPelvisLinearVelocityToPack)
   {
      pelvisLinearStateUpdater.getEstimatedPelvisLinearVelocity(estimatedPelvisLinearVelocityToPack);
   }

   public DenseMatrix64F getCovariance()
   {
      return null;
   }

   public DenseMatrix64F getState()
   {
      return null;
   }

   public void setState(DenseMatrix64F x, DenseMatrix64F covariance)
   {
   }

   public void initializeOrientationEstimateToMeasurement()
   {
      // Do nothing
   }
}
