package us.ihmc.valkyrieRosControl;

import java.util.List;

import org.ejml.data.DenseMatrix64F;

import controller_msgs.msg.dds.AtlasAuxiliaryRobotData;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoForceTorqueSensorHandle;
import us.ihmc.valkyrieRosControl.dataHolders.YoIMUHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoJointStateHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoPositionJointHandleHolder;
import us.ihmc.wholeBodyController.JointTorqueOffsetProcessor;
import us.ihmc.wholeBodyController.diagnostics.JointTorqueOffsetEstimator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ValkyrieRosControlSensorReader implements SensorReader, JointTorqueOffsetProcessor
{
   private final SensorProcessing sensorProcessing;

   private final TimestampProvider timestampProvider;

   private final List<YoEffortJointHandleHolder> yoEffortJointHandleHolders;
   private final List<YoPositionJointHandleHolder> yoPositionJointHandleHolders;
   private final List<YoJointStateHandleHolder> yoJointStateHandleHolders;
   private final List<YoIMUHandleHolder> yoIMUHandleHolders;
   private final List<YoForceTorqueSensorHandle> yoForceTorqueSensorHandles;

   private final Vector3D linearAcceleration = new Vector3D();
   private final Vector3D angularVelocity = new Vector3D();
   private final Quaternion orientation = new Quaternion();

   private final DenseMatrix64F torqueForce = new DenseMatrix64F(6, 1);

   private final ValkyrieRosControlLowLevelController lowlLevelController;

   public ValkyrieRosControlSensorReader(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions,
         SensorProcessingConfiguration sensorProcessingConfiguration, TimestampProvider timestampProvider,
         List<YoEffortJointHandleHolder> yoEffortJointHandleHolders, List<YoPositionJointHandleHolder> yoPositionJointHandleHolders, List<YoJointStateHandleHolder> yoJointStateHandleHolders,
         List<YoIMUHandleHolder> yoIMUHandleHolders, List<YoForceTorqueSensorHandle> yoForceTorqueSensorHandles, ValkyrieJointMap jointMap, YoVariableRegistry registry)
   {

      this.sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, registry);
      this.timestampProvider = timestampProvider;
      this.yoEffortJointHandleHolders = yoEffortJointHandleHolders;
      this.yoPositionJointHandleHolders = yoPositionJointHandleHolders;
      this.yoJointStateHandleHolders = yoJointStateHandleHolders;
      this.yoIMUHandleHolders = yoIMUHandleHolders;
      this.yoForceTorqueSensorHandles = yoForceTorqueSensorHandles;

      double estimatorDT = sensorProcessingConfiguration.getEstimatorDT();
      lowlLevelController = new ValkyrieRosControlLowLevelController(timestampProvider, estimatorDT, yoEffortJointHandleHolders, yoPositionJointHandleHolders,
                                                                     jointMap, registry);
   }

   @Override
   public void read()
   {
      readSensors();
      writeCommandsToRobot();
   }

   public void writeCommandsToRobot()
   {
      lowlLevelController.doControl();
   }

   public void readSensors()
   {
      for (int i = 0; i < yoEffortJointHandleHolders.size(); i++)
      {
         YoEffortJointHandleHolder yoEffortJointHandleHolder = yoEffortJointHandleHolders.get(i);
         yoEffortJointHandleHolder.update();

         sensorProcessing.setJointPositionSensorValue(yoEffortJointHandleHolder.getOneDoFJoint(), yoEffortJointHandleHolder.getQ());
         sensorProcessing.setJointVelocitySensorValue(yoEffortJointHandleHolder.getOneDoFJoint(), yoEffortJointHandleHolder.getQd());
         sensorProcessing.setJointTauSensorValue(yoEffortJointHandleHolder.getOneDoFJoint(), yoEffortJointHandleHolder.getTauMeasured());
      }

      for (int i = 0; i < yoPositionJointHandleHolders.size(); i++)
      {
         YoPositionJointHandleHolder yoPositionJointHandleHolder = yoPositionJointHandleHolders.get(i);
         yoPositionJointHandleHolder.update();

         sensorProcessing.setJointPositionSensorValue(yoPositionJointHandleHolder.getOneDoFJoint(), yoPositionJointHandleHolder.getQ());
         sensorProcessing.setJointVelocitySensorValue(yoPositionJointHandleHolder.getOneDoFJoint(), yoPositionJointHandleHolder.getQd());
         sensorProcessing.setJointTauSensorValue(yoPositionJointHandleHolder.getOneDoFJoint(),
               0.0); // TODO: Should be NaN eventually as the position control joints won't be able to return a measured torque
      }
      
      for(int i = 0; i < yoJointStateHandleHolders.size(); i++)
      {
         YoJointStateHandleHolder yoJointStateHandleHolder = yoJointStateHandleHolders.get(i);
         yoJointStateHandleHolder.update();

         sensorProcessing.setJointPositionSensorValue(yoJointStateHandleHolder.getOneDoFJoint(), yoJointStateHandleHolder.getQ());
         sensorProcessing.setJointVelocitySensorValue(yoJointStateHandleHolder.getOneDoFJoint(), yoJointStateHandleHolder.getQd());
         sensorProcessing.setJointTauSensorValue(yoJointStateHandleHolder.getOneDoFJoint(), yoJointStateHandleHolder.getTauMeasured());
      }

      for (int i = 0; i < yoIMUHandleHolders.size(); i++)
      {
         YoIMUHandleHolder yoIMUHandleHolder = yoIMUHandleHolders.get(i);
         yoIMUHandleHolder.update();

         yoIMUHandleHolder.packLinearAcceleration(linearAcceleration);
         yoIMUHandleHolder.packAngularVelocity(angularVelocity);
         yoIMUHandleHolder.packOrientation(orientation);

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
   public AtlasAuxiliaryRobotData newAuxiliaryRobotDataInstance()
   {
      return null;
   }

   @Override
   public void subtractTorqueOffset(OneDoFJoint oneDoFJoint, double torqueOffset)
   {
      lowlLevelController.subtractTorqueOffset(oneDoFJoint, torqueOffset);
   }

   public void attachControllerAPI(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager)
   {
      lowlLevelController.attachControllerAPI(commandInputManager, statusOutputManager);
   }

   public void attachJointTorqueOffsetEstimator(JointTorqueOffsetEstimator jointTorqueOffsetEstimator)
   {
      lowlLevelController.attachJointTorqueOffsetEstimator(jointTorqueOffsetEstimator);
   }

   public void setupLowLevelControlWithPacketCommunicator(PacketCommunicator packetCommunicator)
   {
      lowlLevelController.setupLowLevelControlWithPacketCommunicator(packetCommunicator);
   }
}
