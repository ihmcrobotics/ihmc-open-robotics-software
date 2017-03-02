package us.ihmc.avatar.logProcessor;

import java.util.LinkedHashMap;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;

public class LogDataRawSensorMap
{
   private final String sensorProcessingName = SensorProcessing.class.getSimpleName();

   private final StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;

   private final LongYoVariable timestamp, visionSensorTimestamp;

   private final LinkedHashMap<String, DoubleYoVariable> rawJointPositionMap = new LinkedHashMap<>();
   private final LinkedHashMap<String, DoubleYoVariable> rawJointVelocityMap = new LinkedHashMap<>();
   private final LinkedHashMap<String, DoubleYoVariable> rawJointTauMap = new LinkedHashMap<>();

   private final LinkedHashMap<String, YoFrameQuaternion> rawIMUOrientationMap = new LinkedHashMap<>();
   private final LinkedHashMap<String, YoFrameVector> rawIMUAngularVelocityMap = new LinkedHashMap<>();
   private final LinkedHashMap<String, YoFrameVector> rawIMULinearAccelerationMap = new LinkedHashMap<>();

   public LogDataRawSensorMap(FullRobotModel fullRobotModel, YoVariableHolder yoVariableHolder)
   {
      stateEstimatorSensorDefinitions = buildStateEstimatorSensorDefinitions(fullRobotModel);

      timestamp = (LongYoVariable) yoVariableHolder.getVariable(sensorProcessingName, "timestamp");
      visionSensorTimestamp = (LongYoVariable) yoVariableHolder.getVariable(sensorProcessingName, "visionSensorTimestamp");
      
      for (OneDoFJoint joint : stateEstimatorSensorDefinitions.getJointSensorDefinitions())
      {
         String jointName = joint.getName();
         DoubleYoVariable rawJointPosition = (DoubleYoVariable) yoVariableHolder.getVariable(sensorProcessingName, "raw_q_" + jointName);
         rawJointPositionMap.put(jointName, rawJointPosition);
         
         DoubleYoVariable rawJointVelocity = (DoubleYoVariable) yoVariableHolder.getVariable(sensorProcessingName, "raw_qd_" + jointName);
         rawJointVelocityMap.put(jointName, rawJointVelocity);
         
         DoubleYoVariable rawJointTau = (DoubleYoVariable) yoVariableHolder.getVariable(sensorProcessingName, "raw_tau_" + jointName);
         rawJointTauMap.put(jointName, rawJointTau);
      }
      
      for (IMUDefinition imuDefinition : stateEstimatorSensorDefinitions.getIMUSensorDefinitions())
      {
         String imuName = imuDefinition.getName();
         DoubleYoVariable qx = (DoubleYoVariable) yoVariableHolder.getVariable(sensorProcessingName, "raw_q_Qx" + imuName);
         DoubleYoVariable qy = (DoubleYoVariable) yoVariableHolder.getVariable(sensorProcessingName, "raw_q_Qy" + imuName);
         DoubleYoVariable qz = (DoubleYoVariable) yoVariableHolder.getVariable(sensorProcessingName, "raw_q_Qz" + imuName);
         DoubleYoVariable qs = (DoubleYoVariable) yoVariableHolder.getVariable(sensorProcessingName, "raw_q_Qs" + imuName);
         if (qx != null && qy != null && qz != null && qs != null)
         {
            YoFrameQuaternion rawOrientation = new YoFrameQuaternion(qx, qy, qz, qs, null);
            rawIMUOrientationMap.put(imuName, rawOrientation);
         }
         
         DoubleYoVariable qd_wx = (DoubleYoVariable) yoVariableHolder.getVariable(sensorProcessingName, YoFrameVariableNameTools.createXName("raw_qd_w", imuName));
         DoubleYoVariable qd_wy = (DoubleYoVariable) yoVariableHolder.getVariable(sensorProcessingName, YoFrameVariableNameTools.createYName("raw_qd_w", imuName));
         DoubleYoVariable qd_wz = (DoubleYoVariable) yoVariableHolder.getVariable(sensorProcessingName, YoFrameVariableNameTools.createZName("raw_qd_w", imuName));
         if (qd_wx != null && qd_wy != null && qd_wz != null)
         {
            YoFrameVector rawAngularVelocity = new YoFrameVector(qd_wx, qd_wy, qd_wz, null);
            rawIMUAngularVelocityMap.put(imuName, rawAngularVelocity);
         }
         
         DoubleYoVariable qdd_x = (DoubleYoVariable) yoVariableHolder.getVariable(sensorProcessingName, YoFrameVariableNameTools.createXName("raw_qdd_", imuName));
         DoubleYoVariable qdd_y = (DoubleYoVariable) yoVariableHolder.getVariable(sensorProcessingName, YoFrameVariableNameTools.createYName("raw_qdd_", imuName));
         DoubleYoVariable qdd_z = (DoubleYoVariable) yoVariableHolder.getVariable(sensorProcessingName, YoFrameVariableNameTools.createZName("raw_qdd_", imuName));
         if (qdd_x != null && qdd_y != null && qdd_z != null)
         {
            YoFrameVector rawLinearAcceleration = new YoFrameVector(qdd_x, qdd_y, qdd_z, null);
            rawIMULinearAccelerationMap.put(imuName, rawLinearAcceleration);
         }
      }
   }

   private StateEstimatorSensorDefinitions buildStateEstimatorSensorDefinitions(FullRobotModel fullRobotModel)
   {
      InverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();

      StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();

      for (InverseDynamicsJoint joint : ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor()))
      {
         if (joint instanceof OneDoFJoint)
         {
            OneDoFJoint oneDoFJoint = (OneDoFJoint) joint;
            stateEstimatorSensorDefinitions.addJointSensorDefinition(oneDoFJoint);
         }
      }

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         stateEstimatorSensorDefinitions.addIMUSensorDefinition(imuDefinition);
      }

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         stateEstimatorSensorDefinitions.addForceSensorDefinition(forceSensorDefinition);
      }
      
      return stateEstimatorSensorDefinitions;
   }

   public long getTimestamp()
   {
      return timestamp.getLongValue();
   }

   public long getVisionSensorTimestamp()
   {
      return visionSensorTimestamp.getLongValue();
   }

   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   public double getRawJointPosition(String jointName)
   {
      return rawJointPositionMap.get(jointName).getDoubleValue();
   }

   public double getRawJointVelocity(String jointName)
   {
      return rawJointVelocityMap.get(jointName).getDoubleValue();
   }

   public double getRawJointTau(String jointName)
   {
      return rawJointTauMap.get(jointName).getDoubleValue();
   }

   public void getRawIMUOrientation(String imuName, Quaternion orientationToPack)
   {
      rawIMUOrientationMap.get(imuName).get(orientationToPack);
   }

   public void getRawIMUAngularVelocity(String imuName, Vector3D angularVelocityToPack)
   {
      rawIMUAngularVelocityMap.get(imuName).get(angularVelocityToPack);
   }

   public void getRawIMULinearAcceleration(String imuName, Vector3D linearAccelerationToPack)
   {
      rawIMULinearAccelerationMap.get(imuName).get(linearAccelerationToPack);
   }
}
