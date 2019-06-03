package us.ihmc.valkyrieRosControl;

import java.util.List;
import java.util.stream.Collectors;

import org.ejml.data.DenseMatrix64F;

import controller_msgs.msg.dds.AtlasAuxiliaryRobotData;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.ros2.RealtimeRos2Node;
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
   private final List<YoEffortJointHandleHolder> yoFingerEffortMotorHandleHolders;
   private final List<YoPositionJointHandleHolder> yoPositionJointHandleHolders;
   private final List<YoJointStateHandleHolder> yoJointStateHandleHolders;
   private final List<YoIMUHandleHolder> yoIMUHandleHolders;
   private final List<YoForceTorqueSensorHandle> yoForceTorqueSensorHandles;

   private final Vector3D linearAcceleration = new Vector3D();
   private final Vector3D angularVelocity = new Vector3D();
   private final Quaternion orientation = new Quaternion();

   private final DenseMatrix64F torqueForce = new DenseMatrix64F(6, 1);

   private final ValkyrieRosControlLowLevelController lowlLevelController;

   private final ValkyrieRosControlFingerStateEstimator fingerStateEstimator;

   public ValkyrieRosControlSensorReader(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions,
                                         SensorProcessingConfiguration sensorProcessingConfiguration, TimestampProvider timestampProvider,
                                         List<YoEffortJointHandleHolder> yoEffortJointHandleHolders,
                                         List<YoPositionJointHandleHolder> yoPositionJointHandleHolders,
                                         List<YoJointStateHandleHolder> yoJointStateHandleHolders, List<YoIMUHandleHolder> yoIMUHandleHolders,
                                         List<YoForceTorqueSensorHandle> yoForceTorqueSensorHandles, ValkyrieJointMap jointMap, YoVariableRegistry registry)
   {

      if (ValkyrieRosControlController.ENABLE_FINGER_JOINTS)
      {
         fingerStateEstimator = new ValkyrieRosControlFingerStateEstimator(yoEffortJointHandleHolders, yoPositionJointHandleHolders, yoJointStateHandleHolders,
                                                                           timestampProvider, stateEstimatorSensorDefinitions, sensorProcessingConfiguration,
                                                                           registry);
         this.sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, fingerStateEstimator, registry);
      }
      else
      {
         fingerStateEstimator = null;
         this.sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, registry);
      }
      this.timestampProvider = timestampProvider;
      // Remove the handles that do not have a joint associated. This is useful to remove the finger motors.
      this.yoEffortJointHandleHolders = yoEffortJointHandleHolders.stream().filter(h -> h.getOneDoFJoint() != null).collect(Collectors.toList());
      yoFingerEffortMotorHandleHolders = yoEffortJointHandleHolders.stream().filter(h -> h.getOneDoFJoint() == null).collect(Collectors.toList());
      this.yoPositionJointHandleHolders = yoPositionJointHandleHolders;
      this.yoJointStateHandleHolders = yoJointStateHandleHolders;
      this.yoIMUHandleHolders = yoIMUHandleHolders;
      this.yoForceTorqueSensorHandles = yoForceTorqueSensorHandles;

      double estimatorDT = sensorProcessingConfiguration.getEstimatorDT();
      lowlLevelController = new ValkyrieRosControlLowLevelController(timestampProvider, estimatorDT, fingerStateEstimator, yoEffortJointHandleHolders,
                                                                     yoPositionJointHandleHolders, jointMap, registry);
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
      // Update the finger motor handles separately
      for (int i = 0; i < yoFingerEffortMotorHandleHolders.size(); i++)
         yoFingerEffortMotorHandleHolders.get(i).update();

      for (int i = 0; i < yoEffortJointHandleHolders.size(); i++)
      {
         YoEffortJointHandleHolder yoEffortJointHandleHolder = yoEffortJointHandleHolders.get(i);
         yoEffortJointHandleHolder.update();

         OneDoFJointBasics joint = yoEffortJointHandleHolder.getOneDoFJoint();

         sensorProcessing.setJointPositionSensorValue(joint, yoEffortJointHandleHolder.getQ());
         sensorProcessing.setJointVelocitySensorValue(joint, yoEffortJointHandleHolder.getQd());
         sensorProcessing.setJointTauSensorValue(joint, yoEffortJointHandleHolder.getTauMeasured());
      }

      for (int i = 0; i < yoPositionJointHandleHolders.size(); i++)
      {
         YoPositionJointHandleHolder yoPositionJointHandleHolder = yoPositionJointHandleHolders.get(i);
         yoPositionJointHandleHolder.update();

         OneDoFJointBasics joint = yoPositionJointHandleHolder.getOneDoFJoint();
         sensorProcessing.setJointPositionSensorValue(joint, yoPositionJointHandleHolder.getQ());
         sensorProcessing.setJointVelocitySensorValue(joint, yoPositionJointHandleHolder.getQd());
         sensorProcessing.setJointTauSensorValue(joint, 0.0); // TODO: Should be NaN eventually as the position control joints won't be able to return a measured torque
      }

      for (int i = 0; i < yoJointStateHandleHolders.size(); i++)
      {
         YoJointStateHandleHolder yoJointStateHandleHolder = yoJointStateHandleHolders.get(i);
         yoJointStateHandleHolder.update();

         OneDoFJointBasics joint = yoJointStateHandleHolder.getOneDoFJoint();
         sensorProcessing.setJointPositionSensorValue(joint, yoJointStateHandleHolder.getQ());
         sensorProcessing.setJointVelocitySensorValue(joint, yoJointStateHandleHolder.getQd());
         sensorProcessing.setJointTauSensorValue(joint, yoJointStateHandleHolder.getTauMeasured());
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

      long wallTime = timestampProvider.getTimestamp();
      long monotonicTime = Conversions.secondsToNanoseconds(lowlLevelController.getControllerTime());
      if (ValkyrieRosControlController.ENABLE_FINGER_JOINTS)
         fingerStateEstimator.update();
      sensorProcessing.startComputation(wallTime, monotonicTime, -1);
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
   public void subtractTorqueOffset(OneDoFJointBasics oneDoFJoint, double torqueOffset)
   {
      lowlLevelController.subtractTorqueOffset(oneDoFJoint, torqueOffset);
   }

   public void attachControllerAPI(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager)
   {
      if (ValkyrieRosControlController.ENABLE_FINGER_JOINTS)
         fingerStateEstimator.attachControllerAPI(commandInputManager, statusOutputManager);
      lowlLevelController.attachControllerAPI(commandInputManager, statusOutputManager);
   }

   public void attachJointTorqueOffsetEstimator(JointTorqueOffsetEstimator jointTorqueOffsetEstimator)
   {
      lowlLevelController.attachJointTorqueOffsetEstimator(jointTorqueOffsetEstimator);
   }

   public void setupLowLevelControlCommunication(String robotName, RealtimeRos2Node realtimeRos2Node)
   {
      lowlLevelController.setupLowLevelControlCommunication(robotName, realtimeRos2Node);
   }
}
