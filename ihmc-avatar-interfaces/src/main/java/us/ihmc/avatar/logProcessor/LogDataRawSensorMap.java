package us.ihmc.avatar.logProcessor;

import java.util.LinkedHashMap;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class LogDataRawSensorMap
{
   private final String sensorProcessingName = SensorProcessing.class.getSimpleName();

   private final StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;

   private final YoLong timestamp, visionSensorTimestamp;

   private final LinkedHashMap<String, YoDouble> rawJointPositionMap = new LinkedHashMap<>();
   private final LinkedHashMap<String, YoDouble> rawJointVelocityMap = new LinkedHashMap<>();
   private final LinkedHashMap<String, YoDouble> rawJointTauMap = new LinkedHashMap<>();

   private final LinkedHashMap<String, YoFrameQuaternion> rawIMUOrientationMap = new LinkedHashMap<>();
   private final LinkedHashMap<String, YoFrameVector3D> rawIMUAngularVelocityMap = new LinkedHashMap<>();
   private final LinkedHashMap<String, YoFrameVector3D> rawIMULinearAccelerationMap = new LinkedHashMap<>();

   public LogDataRawSensorMap(FullRobotModel fullRobotModel, YoVariableHolder yoVariableHolder)
   {
      stateEstimatorSensorDefinitions = buildStateEstimatorSensorDefinitions(fullRobotModel);

      timestamp = (YoLong) yoVariableHolder.findVariable(sensorProcessingName, "timestamp");
      visionSensorTimestamp = (YoLong) yoVariableHolder.findVariable(sensorProcessingName, "visionSensorTimestamp");
      
      for (OneDoFJointBasics joint : stateEstimatorSensorDefinitions.getJointSensorDefinitions())
      {
         String jointName = joint.getName();
         YoDouble rawJointPosition = (YoDouble) yoVariableHolder.findVariable(sensorProcessingName, "raw_q_" + jointName);
         rawJointPositionMap.put(jointName, rawJointPosition);
         
         YoDouble rawJointVelocity = (YoDouble) yoVariableHolder.findVariable(sensorProcessingName, "raw_qd_" + jointName);
         rawJointVelocityMap.put(jointName, rawJointVelocity);
         
         YoDouble rawJointTau = (YoDouble) yoVariableHolder.findVariable(sensorProcessingName, "raw_tau_" + jointName);
         rawJointTauMap.put(jointName, rawJointTau);
      }
      
      for (IMUDefinition imuDefinition : stateEstimatorSensorDefinitions.getIMUSensorDefinitions())
      {
         String imuName = imuDefinition.getName();
         YoDouble qx = (YoDouble) yoVariableHolder.findVariable(sensorProcessingName, "raw_q_Qx" + imuName);
         YoDouble qy = (YoDouble) yoVariableHolder.findVariable(sensorProcessingName, "raw_q_Qy" + imuName);
         YoDouble qz = (YoDouble) yoVariableHolder.findVariable(sensorProcessingName, "raw_q_Qz" + imuName);
         YoDouble qs = (YoDouble) yoVariableHolder.findVariable(sensorProcessingName, "raw_q_Qs" + imuName);
         if (qx != null && qy != null && qz != null && qs != null)
         {
            YoFrameQuaternion rawOrientation = new YoFrameQuaternion(qx, qy, qz, qs, null);
            rawIMUOrientationMap.put(imuName, rawOrientation);
         }
         
         YoDouble qd_wx = (YoDouble) yoVariableHolder.findVariable(sensorProcessingName, YoGeometryNameTools.createXName("raw_qd_w", imuName));
         YoDouble qd_wy = (YoDouble) yoVariableHolder.findVariable(sensorProcessingName, YoGeometryNameTools.createYName("raw_qd_w", imuName));
         YoDouble qd_wz = (YoDouble) yoVariableHolder.findVariable(sensorProcessingName, YoGeometryNameTools.createZName("raw_qd_w", imuName));
         if (qd_wx != null && qd_wy != null && qd_wz != null)
         {
            YoFrameVector3D rawAngularVelocity = new YoFrameVector3D(qd_wx, qd_wy, qd_wz, null);
            rawIMUAngularVelocityMap.put(imuName, rawAngularVelocity);
         }
         
         YoDouble qdd_x = (YoDouble) yoVariableHolder.findVariable(sensorProcessingName, YoGeometryNameTools.createXName("raw_qdd_", imuName));
         YoDouble qdd_y = (YoDouble) yoVariableHolder.findVariable(sensorProcessingName, YoGeometryNameTools.createYName("raw_qdd_", imuName));
         YoDouble qdd_z = (YoDouble) yoVariableHolder.findVariable(sensorProcessingName, YoGeometryNameTools.createZName("raw_qdd_", imuName));
         if (qdd_x != null && qdd_y != null && qdd_z != null)
         {
            YoFrameVector3D rawLinearAcceleration = new YoFrameVector3D(qdd_x, qdd_y, qdd_z, null);
            rawIMULinearAccelerationMap.put(imuName, rawLinearAcceleration);
         }
      }
   }

   private StateEstimatorSensorDefinitions buildStateEstimatorSensorDefinitions(FullRobotModel fullRobotModel)
   {
      JointBasics rootJoint = fullRobotModel.getRootJoint();
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();

      StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();

      for (JointBasics joint : rootJoint.subtreeIterable())
      {
         if (joint instanceof OneDoFJointBasics)
         {
            OneDoFJointBasics oneDoFJoint = (OneDoFJointBasics) joint;
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
      orientationToPack.set(rawIMUOrientationMap.get(imuName));
   }

   public void getRawIMUAngularVelocity(String imuName, Vector3D angularVelocityToPack)
   {
      angularVelocityToPack.set(rawIMUAngularVelocityMap.get(imuName));
   }

   public void getRawIMULinearAcceleration(String imuName, Vector3D linearAccelerationToPack)
   {
      linearAccelerationToPack.set(rawIMULinearAccelerationMap.get(imuName));
   }
}
