package us.ihmc.valkyrieRosControl;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.yaml.snakeyaml.Yaml;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.valkyrie.imu.MicroStrainData;
import us.ihmc.valkyrieRosControl.dataHolders.YoForceTorqueSensorHandle;
import us.ihmc.valkyrieRosControl.dataHolders.YoIMUHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoJointHandleHolder;
import us.ihmc.wholeBodyController.JointTorqueOffsetProcessor;

public class ValkyrieRosControlSensorReader implements SensorReader, JointTorqueOffsetProcessor
{
   private final SensorProcessing sensorProcessing;

   private final TimestampProvider timestampProvider;

   private final List<YoJointHandleHolder> yoJointHandleHolders;
   private final List<YoIMUHandleHolder> yoIMUHandleHolders;
   private final List<YoForceTorqueSensorHandle> yoForceTorqueSensorHandles;

   private final Vector3d linearAcceleration = new Vector3d();
   private final Vector3d angularVelocity = new Vector3d();
   private final Quat4d orientation = new Quat4d();

   private final DenseMatrix64F torqueForce = new DenseMatrix64F(6, 1);

   private final ArrayList<ValkyrieRosControlJointControlCommandCalculator> controlCommandCalculators = new ArrayList<>();
   private final LinkedHashMap<String, ValkyrieRosControlJointControlCommandCalculator> jointToControlCommandCalculatorMap = new LinkedHashMap<>();

   private final DoubleYoVariable doIHMCControlRatio;
   private final DoubleYoVariable timeInStandprep;
   private final DoubleYoVariable masterGain;

   private final BooleanYoVariable startStandPrep;
   private long standPrepStartTime = -1;

   private final Matrix3d quaternionConversionMatrix = new Matrix3d();
   private final Matrix3d orientationMatrix = new Matrix3d();

   private final ValkyrieTorqueHysteresisCompensator torqueHysteresisCompensator;

   @SuppressWarnings("unchecked")
   public ValkyrieRosControlSensorReader(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, SensorProcessingConfiguration sensorProcessingConfiguration,
         TimestampProvider timestampProvider, List<YoJointHandleHolder> yoJointHandleHolders, List<YoIMUHandleHolder> yoIMUHandleHolders,
         List<YoForceTorqueSensorHandle> yoForceTorqueSensorHandles, YoVariableRegistry registry)
   {

      this.sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, registry);
      this.timestampProvider = timestampProvider;
      this.yoJointHandleHolders = yoJointHandleHolders;
      this.yoIMUHandleHolders = yoIMUHandleHolders;
      this.yoForceTorqueSensorHandles = yoForceTorqueSensorHandles;

      doIHMCControlRatio = new DoubleYoVariable("doIHMCControlRatio", registry);
      masterGain = new DoubleYoVariable("StandPrepMasterGain", registry);
      timeInStandprep = new DoubleYoVariable("timeInStandprep", registry);
      startStandPrep = new BooleanYoVariable("startStandPrep", registry);
      startStandPrep.set(true);
      masterGain.set(0.3);

      RigidBody rootBody = ScrewTools.getRootBody(stateEstimatorSensorDefinitions.getJointSensorDefinitions().get(0).getSuccessor());
      torqueHysteresisCompensator = new ValkyrieTorqueHysteresisCompensator(rootBody, timeInStandprep, registry);

      Yaml yaml = new Yaml();

      InputStream gainStream = getClass().getClassLoader().getResourceAsStream("standPrep/gains.yaml");
      InputStream setpointsStream = getClass().getClassLoader().getResourceAsStream("standPrep/setpoints.yaml");
      InputStream offsetsStream;
      try
      {
         offsetsStream = new FileInputStream(new File(ValkyrieTorqueOffsetPrinter.IHMC_TORQUE_OFFSET_FILE));
      }
      catch (FileNotFoundException e1)
      {
         offsetsStream = null;
      }

      Map<String, Map<String, Double>> gainMap = (Map<String, Map<String, Double>>) yaml.load(gainStream);
      Map<String, Double> setPointMap = (Map<String, Double>) yaml.load(setpointsStream);
      Map<String, Double> offsetMap = null;
      if (offsetsStream != null)
         offsetMap = (Map<String, Double>) yaml.load(offsetsStream);

      try
      {
         gainStream.close();
         setpointsStream.close();
         if (offsetsStream != null)
            offsetsStream.close();
      }
      catch (IOException e)
      {
      }

      for (YoJointHandleHolder jointHandleHolder : yoJointHandleHolders)
      {
         String jointName = jointHandleHolder.getName();
         Map<String, Double> standPrepGains = gainMap.get(jointName);
         double torqueOffset = 0.0;
         if (offsetMap != null && offsetMap.containsKey(jointName))
            torqueOffset = offsetMap.get(jointName);

         double standPrepAngle = 0.0;
         if (setPointMap.containsKey(jointName))
         {
            standPrepAngle = setPointMap.get(jointName);
         }
         ValkyrieRosControlJointControlCommandCalculator controlCommandCalculator = new ValkyrieRosControlJointControlCommandCalculator(jointHandleHolder,
               standPrepGains, torqueOffset, standPrepAngle, sensorProcessingConfiguration.getEstimatorDT(), registry);
         controlCommandCalculators.add(controlCommandCalculator);

         jointToControlCommandCalculatorMap.put(jointName, controlCommandCalculator);
      }
   }

   public void setDoIHMCControlRatio(double controlRatio)
   {
      doIHMCControlRatio.set(MathTools.clipToMinMax(controlRatio, 0.0, 1.0));
   }

   @Override
   public void read()
   {
      readSensors();
      writeCommandsToRobot();
   }

   public void readSensors()
   {
      for (int i = 0; i < yoJointHandleHolders.size(); i++)
      {
         YoJointHandleHolder yoJointHandleHolder = yoJointHandleHolders.get(i);
         yoJointHandleHolder.update();

         sensorProcessing.setJointPositionSensorValue(yoJointHandleHolder.getOneDoFJoint(), yoJointHandleHolder.getQ());
         sensorProcessing.setJointVelocitySensorValue(yoJointHandleHolder.getOneDoFJoint(), yoJointHandleHolder.getQd());
         sensorProcessing.setJointTauSensorValue(yoJointHandleHolder.getOneDoFJoint(), yoJointHandleHolder.getTauMeasured());
      }

      for (int i = 0; i < yoIMUHandleHolders.size(); i++)
      {
         YoIMUHandleHolder yoIMUHandleHolder = yoIMUHandleHolders.get(i);
         yoIMUHandleHolder.update();

         yoIMUHandleHolder.packLinearAcceleration(linearAcceleration);
         yoIMUHandleHolder.packAngularVelocity(angularVelocity);
         yoIMUHandleHolder.packOrientation(orientation);

         quaternionConversionMatrix.set(orientation);
         orientationMatrix.mul(MicroStrainData.MICROSTRAIN_TO_ZUP_WORLD, quaternionConversionMatrix);
         RotationTools.setQuaternionBasedOnMatrix3d(orientation, orientationMatrix);

         sensorProcessing.setLinearAccelerationSensorValue(yoIMUHandleHolder.getImuDefinition(), linearAcceleration);
         sensorProcessing.setAngularVelocitySensorValue(yoIMUHandleHolder.getImuDefinition(), angularVelocity);
         sensorProcessing.setOrientationSensorValue(yoIMUHandleHolder.getImuDefinition(), orientation);
      }

      for (int i = 0; i < yoForceTorqueSensorHandles.size(); i++)
      {
         YoForceTorqueSensorHandle yoForceTorqueSensorHandle = yoForceTorqueSensorHandles.get(i);
         yoForceTorqueSensorHandle.update();

         yoForceTorqueSensorHandle.packWrench(torqueForce);
         sensorProcessing.setForceSensorValue(yoForceTorqueSensorHandle.getForceSensorDefinition(), torqueForce);
      }

      long timestamp = timestampProvider.getTimestamp();
      sensorProcessing.startComputation(timestamp, timestamp, -1);
   }

   public void writeCommandsToRobot()
   {
      long timestamp = timestampProvider.getTimestamp();

      if (standPrepStartTime > 0)
      {
         timeInStandprep.set(TimeTools.nanoSecondstoSeconds(timestamp - standPrepStartTime));

         torqueHysteresisCompensator.compute();

         for (int i = 0; i < controlCommandCalculators.size(); i++)
         {
            ValkyrieRosControlJointControlCommandCalculator commandCalculator = controlCommandCalculators.get(i);
            commandCalculator.computeAndUpdateJointTorque(timeInStandprep.getDoubleValue(), doIHMCControlRatio.getDoubleValue(), masterGain.getDoubleValue());
         }

      }
      else if (startStandPrep.getBooleanValue())
      {
         standPrepStartTime = timestamp;
         for (int i = 0; i < controlCommandCalculators.size(); i++)
         {
            ValkyrieRosControlJointControlCommandCalculator commandCalculator = controlCommandCalculators.get(i);
            commandCalculator.initialize();
         }
      }
   }

   @Override
   public SensorOutputMapReadOnly getSensorOutputMapReadOnly()
   {
      return sensorProcessing;
   }

   @Override
   public SensorRawOutputMapReadOnly getSensorRawOutputMapReadOnly()
   {
      return sensorProcessing;
   }

   @Override
   public AuxiliaryRobotData newAuxiliaryRobotDataInstance()
   {
      return null;
   }

   @Override
   public void subtractTorqueOffset(OneDoFJoint oneDoFJoint, double torqueOffset)
   {
      ValkyrieRosControlJointControlCommandCalculator jointCommandCalculator = jointToControlCommandCalculatorMap.get(oneDoFJoint.getName());
      if (jointCommandCalculator != null)
         jointCommandCalculator.subtractTorqueOffset(torqueOffset);
      else
         PrintTools.error("Command calculator is NULL for the joint: " + oneDoFJoint.getName());
   }
}
