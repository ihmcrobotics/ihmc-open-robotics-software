package us.ihmc.darpaRoboticsChallenge;

import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.IMUPacket;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.humanoidRobotics.communication.streamingData.AtomicLastPacketHolder.LastPacket;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.ForceSensorDistalMassCompensator;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.simulationconstructionset.robotController.RawOutputWriter;
import us.ihmc.util.PeriodicThreadScheduler;

// fills a ring buffer with pose and joint data and in a worker thread passes it to the appropriate consumer 
public class DRCPoseCommunicator implements RawOutputWriter
{
   private final int WORKER_SLEEP_TIME_MILLIS = 1;
   private final IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();

//   private final ScheduledExecutorService writeExecutor = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("DRCPoseCommunicator"));
   private final PeriodicThreadScheduler scheduler;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final HumanoidGlobalDataProducer dataProducer;
   private final JointConfigurationGatherer jointConfigurationGathererAndProducer;
   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;
   private final SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly;
   private final SideDependentList<String> wristForceSensorNames;
   private final RobotMotionStatusHolder robotMotionStatusFromController;
   
   // puts the state data into the ring buffer for the output thread
    private final Vector3f[] imuLinearAccelerations;
    private final Vector3f[] rawImuAngularVelocities;
    private final Matrix3d[] imuOrientationsAsMatrix;
    private final Quat4f[] imuOrientations;

   private final SideDependentList<ReferenceFrame> wristForceSensorFrames = new SideDependentList<ReferenceFrame>();
   private final SideDependentList<ForceSensorDistalMassCompensator> wristForceSensorDistalMassCompensators = new SideDependentList<ForceSensorDistalMassCompensator>();

   private final ConcurrentRingBuffer<RobotConfigurationData> robotConfigurationDataRingBuffer;

   public DRCPoseCommunicator(SDFFullRobotModel estimatorModel, JointConfigurationGatherer jointConfigurationGathererAndProducer, SensorReader sensorReader,
         HumanoidGlobalDataProducer dataProducer, SensorOutputMapReadOnly sensorOutputMapReadOnly, SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly, RobotMotionStatusHolder robotMotionStatusFromController, DRCRobotSensorInformation sensorInformation, PeriodicThreadScheduler scheduler)
   {
      this.dataProducer = dataProducer;
      this.jointConfigurationGathererAndProducer = jointConfigurationGathererAndProducer;
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;
      this.sensorRawOutputMapReadOnly = sensorRawOutputMapReadOnly;
      this.robotMotionStatusFromController = robotMotionStatusFromController;
      this.wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      this.scheduler = scheduler;
      
      setupForceSensorMassCompensators(estimatorModel);

      IMUDefinition[] imuDefinitions = estimatorModel.getIMUDefinitions();
      
      int numberOfImuSensors = imuDefinitions.length;
      imuLinearAccelerations = new Vector3f[numberOfImuSensors];
      rawImuAngularVelocities = new Vector3f[numberOfImuSensors];
      imuOrientationsAsMatrix = new Matrix3d[numberOfImuSensors];
      imuOrientations = new Quat4f[numberOfImuSensors];
      
      for (int imuSensorIndex = 0; imuSensorIndex < numberOfImuSensors; imuSensorIndex++)
      {
         imuLinearAccelerations[imuSensorIndex] = new Vector3f();
         rawImuAngularVelocities[imuSensorIndex] = new Vector3f();
         imuOrientationsAsMatrix[imuSensorIndex] = new Matrix3d();
         imuOrientations[imuSensorIndex] = new Quat4f();
      }
      ForceSensorDefinition[] forceSensorDefinitions = jointConfigurationGathererAndProducer.getForceSensorDefinitions();
      OneDoFJoint[] joints = jointConfigurationGathererAndProducer.getJoints();
      robotConfigurationDataRingBuffer = new ConcurrentRingBuffer<RobotConfigurationData>(new RobotConfigurationDataBuilder(joints, forceSensorDefinitions, imuDefinitions, sensorReader), 16);
      startWriterThread();
   }

   private void setupForceSensorMassCompensators(SDFFullRobotModel estimatorModel)
   {
      ForceSensorDefinition[] forceSensorDefinitions = estimatorModel.getForceSensorDefinitions();

      for (int i = 0; i < forceSensorDefinitions.length; i++)
      {
         ForceSensorDefinition sensorDef = forceSensorDefinitions[i];
         String forceSensorName = sensorDef.getSensorName();

         for (RobotSide robotSide : RobotSide.values)
         {
            if (forceSensorName.equals(wristForceSensorNames.get(robotSide)))
            {
               ForceSensorDistalMassCompensator massComp = new ForceSensorDistalMassCompensator(sensorDef, WORKER_SLEEP_TIME_MILLIS, registry);
               wristForceSensorDistalMassCompensators.put(robotSide, massComp);

               ReferenceFrame sensorFrame = sensorDef.getSensorFrame();
               wristForceSensorFrames.put(robotSide, sensorFrame);
            }
         }
      }
   }

   // this thread reads from the stateRingBuffer and pushes the data out to the objectConsumer
   private void startWriterThread()
   {
      scheduler.schedule(new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               if (robotConfigurationDataRingBuffer.poll())
               {
                  RobotConfigurationData robotConfigData;
                  while ((robotConfigData = robotConfigurationDataRingBuffer.read()) != null)
                  {
                     dataProducer.send(robotConfigData);
                  }
                  robotConfigurationDataRingBuffer.flush();
               }
            }
            catch(Throwable throwable)
            {
               throwable.printStackTrace();
            }

         }

      }, WORKER_SLEEP_TIME_MILLIS, TimeUnit.MILLISECONDS);

   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }


   // puts the state data into the ring buffer for the output thread
   @Override
   public void write()
   {

      RobotConfigurationData state = robotConfigurationDataRingBuffer.next();
      if (state == null)
      {
         return;
      }

      long timestamp = sensorOutputMapReadOnly.getVisionSensorTimestamp();
      long pps = sensorOutputMapReadOnly.getSensorHeadPPSTimestamp();
      jointConfigurationGathererAndProducer.packEstimatorJoints(timestamp, pps, state);
      
      List<? extends IMUSensorReadOnly> imuRawOutputs = sensorRawOutputMapReadOnly.getIMURawOutputs();
      for (int sensorNumber = 0; sensorNumber < imuRawOutputs.size(); sensorNumber++)
      {
         IMUSensorReadOnly imuSensor =  imuRawOutputs.get(sensorNumber);
         IMUPacket imuPacketToPack = state.getImuPacketForSensor(sensorNumber);
         
         imuSensor.getLinearAccelerationMeasurement(imuLinearAccelerations[sensorNumber]);
         imuSensor.getOrientationMeasurement(imuOrientationsAsMatrix[sensorNumber]);
         RotationTools.setQuaternionBasedOnMatrix3d(imuOrientations[sensorNumber], imuOrientationsAsMatrix[sensorNumber]);
         imuSensor.getAngularVelocityMeasurement(rawImuAngularVelocities[sensorNumber]);
         
         imuPacketToPack.set(imuLinearAccelerations[sensorNumber], imuOrientations[sensorNumber], rawImuAngularVelocities[sensorNumber]);
      }
      
      state.setRobotMotionStatus(robotMotionStatusFromController.getCurrentRobotMotionStatus());

      state.setAuxiliaryRobotData(sensorRawOutputMapReadOnly.getAuxiliaryRobotData());
      
      LastPacket lastPacket = dataProducer.getLastPacket();
      if(lastPacket != null)
      {
         state.setLastReceivedPacketTypeID(netClassList.getID(lastPacket.getPacket()));
         state.setLastReceivedPacketUniqueId(lastPacket.getUniqueId());
         state.setLastReceivedPacketRobotTimestamp(lastPacket.getReceivedTimestamp());
      }
      else
      {
         state.setLastReceivedPacketTypeID(-1);
         state.setLastReceivedPacketUniqueId(-1);
         state.setLastReceivedPacketRobotTimestamp(-1);
      }
      
      robotConfigurationDataRingBuffer.commit();
   }

   public static class RobotConfigurationDataBuilder implements us.ihmc.concurrent.Builder<RobotConfigurationData>
   {
      private final OneDoFJoint[] joints;
      private final ForceSensorDefinition[] forceSensorDefinitions;
      private final IMUDefinition[] imuDefinitions;
      private final SensorReader sensorReader;

      public RobotConfigurationDataBuilder(OneDoFJoint[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions, SensorReader sensorReader)
      {
         this.joints = joints;
         this.forceSensorDefinitions = forceSensorDefinitions;
         this.imuDefinitions = imuDefinitions;
         this.sensorReader = sensorReader;
      }

      @Override
      public RobotConfigurationData newInstance()
      {
         return new RobotConfigurationData(joints, forceSensorDefinitions, sensorReader.newAuxiliaryRobotDataInstance(), imuDefinitions);
      }

   }
   public void stop()
   {
      scheduler.shutdown();
   }
}