package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.sensorProcessing.imu.FusedIMUSensor;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.humanoid.DRCStateEstimatorInterface;
import us.ihmc.utilities.humanoidRobot.model.CenterOfPressureDataHolder;
import us.ihmc.utilities.humanoidRobot.model.RobotMotionStatusHolder;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.utilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;

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

   private final PelvisPoseHistoryCorrection pelvisPoseHistoryCorrection;

   private final double estimatorDT;

   private boolean visualizeMeasurementFrames = false;
   private final ArrayList<YoGraphicReferenceFrame> dynamicGraphicMeasurementFrames = new ArrayList<>();

   private final CenterOfPressureVisualizer copVisualizer;

   private final BooleanYoVariable usePelvisCorrector;
   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;

   public DRCKinematicsBasedStateEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, StateEstimatorParameters stateEstimatorParameters,
         SensorOutputMapReadOnly sensorOutputMapReadOnly, String[] imuSensorsToUseInStateEstimator, double gravitationalAcceleration,
         SideDependentList<FootSwitchInterface> footSwitches, CenterOfPressureDataHolder centerOfPressureDataHolderFromController,
         RobotMotionStatusHolder robotMotionStatusFromController, SideDependentList<ContactablePlaneBody> bipedFeet,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      estimatorDT = stateEstimatorParameters.getEstimatorDT();
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;

      usePelvisCorrector = new BooleanYoVariable("useExternalPelvisCorrector", registry);
      usePelvisCorrector.set(true);
      jointStateUpdater = new JointStateUpdater(inverseDynamicsStructure, sensorOutputMapReadOnly, stateEstimatorParameters, registry);

      pelvisPoseHistoryCorrection = new PelvisPoseHistoryCorrection(inverseDynamicsStructure, stateEstimatorParameters.getEstimatorDT(), registry, 1000);

      List<IMUSensorReadOnly> imuProcessedOutputs = new ArrayList<>();
      List<String> imuSensorsToUse = Arrays.asList(imuSensorsToUseInStateEstimator);
      for (IMUSensorReadOnly imu : sensorOutputMapReadOnly.getIMUProcessedOutputs())
      {
         if (imuSensorsToUse.contains(imu.getSensorName()))
            imuProcessedOutputs.add(imu);
      }

      List<IMUSensorReadOnly> imusToUse = new ArrayList<>();

      if (stateEstimatorParameters.createFusedIMUSensor())
      {
         if (imuProcessedOutputs.size() != 2)
            throw new RuntimeException("Cannot create FusedIMUSensor.");
         fusedIMUSensor = new FusedIMUSensor(imuProcessedOutputs.get(0), imuProcessedOutputs.get(1), estimatorDT,
               stateEstimatorParameters.getIMUDriftFilterFreqInHertz(), registry);
         imusToUse.add(fusedIMUSensor);
      }
      else
      {
         fusedIMUSensor = null;
         imusToUse.addAll(imuProcessedOutputs);
      }

      pelvisRotationalStateUpdater = new PelvisRotationalStateUpdater(inverseDynamicsStructure, imusToUse, registry);

      pelvisLinearStateUpdater = new PelvisLinearStateUpdater(inverseDynamicsStructure, imusToUse, footSwitches, centerOfPressureDataHolderFromController, bipedFeet, gravitationalAcceleration, yoTime,
            stateEstimatorParameters, yoGraphicsListRegistry, registry);

      if (yoGraphicsListRegistry != null)
      {
         copVisualizer = new CenterOfPressureVisualizer(footSwitches, yoGraphicsListRegistry, registry);
      }
      else
      {
         copVisualizer = null;
      }

      visualizeMeasurementFrames = visualizeMeasurementFrames && yoGraphicsListRegistry != null;

      List<IMUSensorReadOnly> imusToDisplay = new ArrayList<>();
      imusToDisplay.addAll(imuProcessedOutputs);
      if (fusedIMUSensor != null)
         imusToDisplay.add(fusedIMUSensor);

      if (visualizeMeasurementFrames)
         setupDynamicGraphicObjects(yoGraphicsListRegistry, imusToDisplay);
   }

   private void setupDynamicGraphicObjects(YoGraphicsListRegistry yoGraphicsListRegistry, List<? extends IMUSensorReadOnly> imuProcessedOutputs)
   {
      for (int i = 0; i < imuProcessedOutputs.size(); i++)
      {
         YoGraphicReferenceFrame dynamicGraphicMeasurementFrame = new YoGraphicReferenceFrame(imuProcessedOutputs.get(i).getMeasurementFrame(), registry, 1.0);
         dynamicGraphicMeasurementFrames.add(dynamicGraphicMeasurementFrame);
      }
      yoGraphicsListRegistry.registerYoGraphics("imuFrame", dynamicGraphicMeasurementFrames);
   }

   @Override
   public StateEstimator getStateEstimator()
   {
      return this;
   }

   @Override
   public void initialize()
   {
      if (fusedIMUSensor != null)
         fusedIMUSensor.update();

      jointStateUpdater.initialize();
      pelvisRotationalStateUpdater.initialize();
      pelvisLinearStateUpdater.initialize();
   }

   @Override
   public void doControl()
   {
      yoTime.set(TimeTools.nanoSecondstoSeconds(sensorOutputMapReadOnly.getTimestamp()));

      if (fusedIMUSensor != null)
         fusedIMUSensor.update();

      jointStateUpdater.updateJointState();
      pelvisRotationalStateUpdater.updateRootJointOrientationAndAngularVelocity();
      pelvisLinearStateUpdater.updateRootJointPositionAndLinearVelocity();

      if (usePelvisCorrector.getBooleanValue() && pelvisPoseHistoryCorrection != null)
      {
         pelvisPoseHistoryCorrection.doControl(sensorOutputMapReadOnly.getVisionSensorTimestamp());
      }

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

   @Override
   public void initializeEstimatorToActual(Point3d initialCoMPosition, Quat4d initialEstimationLinkOrientation)
   {
      pelvisLinearStateUpdater.initializeCoMPositionToActual(initialCoMPosition);
      // Do nothing for the orientation since the IMU is trusted
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void getEstimatedOrientation(FrameOrientation estimatedOrientationToPack)
   {
      pelvisRotationalStateUpdater.getEstimatedOrientation(estimatedOrientationToPack);
   }

   @Override
   public void setEstimatedOrientation(FrameOrientation estimatedOrientation)
   {
      // Do nothing, IMU is trusted
   }

   @Override
   public void getEstimatedAngularVelocity(FrameVector estimatedAngularVelocityToPack)
   {
      pelvisRotationalStateUpdater.getEstimatedAngularVelocity(estimatedAngularVelocityToPack);
   }

   @Override
   public void setEstimatedAngularVelocity(FrameVector estimatedAngularVelocity)
   {
      // Do nothing, IMU is trusted
   }

   @Override
   public void getEstimatedCoMPosition(FramePoint estimatedCoMPositionToPack)
   {
      pelvisLinearStateUpdater.getEstimatedCoMPosition(estimatedCoMPositionToPack);
   }

   @Override
   public void setEstimatedCoMPosition(FramePoint estimatedCoMPosition)
   {
      pelvisLinearStateUpdater.initializeCoMPositionToActual(estimatedCoMPosition);
   }

   @Override
   public void getEstimatedCoMVelocity(FrameVector estimatedCoMVelocityToPack)
   {
      pelvisLinearStateUpdater.getEstimatedCoMVelocity(estimatedCoMVelocityToPack);
   }

   @Override
   public void setEstimatedCoMVelocity(FrameVector estimatedCoMVelocity)
   {
   }

   @Override
   public void getEstimatedPelvisPosition(FramePoint estimatedPelvisPositionToPack)
   {
      pelvisLinearStateUpdater.getEstimatedPelvisPosition(estimatedPelvisPositionToPack);
   }

   @Override
   public void getEstimatedPelvisLinearVelocity(FrameVector estimatedPelvisLinearVelocityToPack)
   {
      pelvisLinearStateUpdater.getEstimatedPelvisLinearVelocity(estimatedPelvisLinearVelocityToPack);
   }

   @Override
   public DenseMatrix64F getCovariance()
   {
      return null;
   }

   @Override
   public DenseMatrix64F getState()
   {
      return null;
   }

   @Override
   public void setState(DenseMatrix64F x, DenseMatrix64F covariance)
   {
   }

   @Override
   public void initializeOrientationEstimateToMeasurement()
   {
      // Do nothing
   }

   public void setExternelPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      pelvisPoseHistoryCorrection.setExternelPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
   }
}
