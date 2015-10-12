package us.ihmc.valkyrieRosControl;

import java.util.List;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.valkyrieRosControl.dataHolders.YoForceTorqueSensorHandle;
import us.ihmc.valkyrieRosControl.dataHolders.YoIMUHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoJointHandleHolder;

public class ValkyrieRosControlSensorReader implements SensorReader
{

   private final SensorProcessing sensorProcessing;

   private final TimestampProvider timestampProvider;

   
   private final List<YoJointHandleHolder> yoJointHandleHolders;
   private final List<YoIMUHandleHolder> yoIMUHandleHolders;
   private final List<YoForceTorqueSensorHandle> yoForceTorqueSensorHandles;
   
   
   private final Vector3d linearAcceleration = new Vector3d();
   private final Vector3d angularVelocity = new Vector3d();
   private final Quat4d orientation = new Quat4d();
   
   private final DenseMatrix64F torqueForce = new DenseMatrix64F(6,1);
   
   public ValkyrieRosControlSensorReader(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, StateEstimatorParameters stateEstimatorParameters, TimestampProvider timestampProvider, List<YoJointHandleHolder> yoJointHandleHolders, List<YoIMUHandleHolder> yoIMUHandleHolders, List<YoForceTorqueSensorHandle> yoForceTorqueSensorHandles, YoVariableRegistry registry)
   {
      
      this.sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, stateEstimatorParameters, registry);
      this.timestampProvider = timestampProvider;
      this.yoJointHandleHolders = yoJointHandleHolders;
      this.yoIMUHandleHolders = yoIMUHandleHolders;
      this.yoForceTorqueSensorHandles = yoForceTorqueSensorHandles;
   }

   @Override
   public void read()
   {
      for(YoJointHandleHolder yoJointHandleHolder : yoJointHandleHolders)
      {
         yoJointHandleHolder.update();
         
         sensorProcessing.setJointPositionSensorValue(yoJointHandleHolder.getOneDoFJoint(), yoJointHandleHolder.getQ());
         sensorProcessing.setJointVelocitySensorValue(yoJointHandleHolder.getOneDoFJoint(), yoJointHandleHolder.getQd());
         sensorProcessing.setJointTauSensorValue(yoJointHandleHolder.getOneDoFJoint(), yoJointHandleHolder.getTauMeasured());
      }
      
      for(YoIMUHandleHolder yoIMUHandleHolder : yoIMUHandleHolders)
      {
         yoIMUHandleHolder.update();
         
         yoIMUHandleHolder.packLinearAcceleration(linearAcceleration);
         yoIMUHandleHolder.packAngularVelocity(angularVelocity);
         yoIMUHandleHolder.packOrientation(orientation);
         
         sensorProcessing.setLinearAccelerationSensorValue(yoIMUHandleHolder.getImuDefinition(), linearAcceleration);
         sensorProcessing.setAngularVelocitySensorValue(yoIMUHandleHolder.getImuDefinition(), angularVelocity);
         sensorProcessing.setOrientationSensorValue(yoIMUHandleHolder.getImuDefinition(), orientation);
      }
      
      for(YoForceTorqueSensorHandle yoForceTorqueSensorHandle : yoForceTorqueSensorHandles)
      {
         yoForceTorqueSensorHandle.update();
         
         yoForceTorqueSensorHandle.packWrench(torqueForce);
         sensorProcessing.setForceSensorValue(yoForceTorqueSensorHandle.getForceSensorDefinition(), torqueForce);
      }
      
      
      sensorProcessing.startComputation(timestampProvider.getTimestamp(), timestampProvider.getTimestamp(), -1);
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

}
