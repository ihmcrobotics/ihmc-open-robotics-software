package us.ihmc.sensorProcessing.communication.producers;

import java.util.List;
import java.util.concurrent.TimeUnit;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packets.IMUPacket;
import us.ihmc.communication.streamingData.AtomicLastPacketHolder.LastPacket;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotController.RawOutputWriter;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AtlasAuxiliaryRobotData;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.ForceSensorDistalMassCompensator;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.simulatedSensors.AuxiliaryRobotDataProvider;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

// fills a ring buffer with pose and joint data and in a worker thread passes it to the appropriate
// consumer
public class DRCPoseCommunicator implements RawOutputWriter
{
   private final int WORKER_SLEEP_TIME_MILLIS = 1;
   private final NetClassList netClassList;

   //   private final ScheduledExecutorService writeExecutor = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("DRCPoseCommunicator"));
   private final PeriodicThreadScheduler scheduler;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final GlobalDataProducer dataProducer;
   private final JointConfigurationGatherer jointConfigurationGathererAndProducer;
   private final SensorTimestampHolder sensorTimestampHolder;
   private final SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly;
   //   private final SideDependentList<String> wristForceSensorNames;
   private final RobotMotionStatusHolder robotMotionStatusFromController;

   // puts the state data into the ring buffer for the output thread
   private final Vector3D32[] imuLinearAccelerations;
   private final Vector3D32[] rawImuAngularVelocities;
   private final RotationMatrix[] imuOrientationsAsMatrix;
   private final Quaternion32[] imuOrientations;

   private final SideDependentList<ReferenceFrame> wristForceSensorFrames = new SideDependentList<ReferenceFrame>();
   private final SideDependentList<ForceSensorDistalMassCompensator> wristForceSensorDistalMassCompensators = new SideDependentList<ForceSensorDistalMassCompensator>();

   private final ConcurrentRingBuffer<RobotConfigurationData> robotConfigurationDataRingBuffer;
   private final ConcurrentRingBuffer<AtlasAuxiliaryRobotData> atlasAuxiliaryRobotDataRingBuffer;

   public DRCPoseCommunicator(FullRobotModel estimatorModel, JointConfigurationGatherer jointConfigurationGathererAndProducer,
                              AuxiliaryRobotDataProvider auxiliaryRobotDataProvider, GlobalDataProducer dataProducer,
                              SensorTimestampHolder sensorTimestampHolder, SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly,
                              RobotMotionStatusHolder robotMotionStatusFromController, DRCRobotSensorInformation sensorInformation,
                              PeriodicThreadScheduler scheduler, NetClassList netClassList)
   {
      this.dataProducer = dataProducer;
      this.jointConfigurationGathererAndProducer = jointConfigurationGathererAndProducer;
      this.sensorTimestampHolder = sensorTimestampHolder;
      this.sensorRawOutputMapReadOnly = sensorRawOutputMapReadOnly;
      this.robotMotionStatusFromController = robotMotionStatusFromController;
      this.scheduler = scheduler;
      this.netClassList = netClassList;

      if (sensorInformation != null)
      {
         SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
         if (wristForceSensorNames != null && !wristForceSensorNames.isEmpty())
         {
            setupForceSensorMassCompensators(estimatorModel, wristForceSensorNames);
         }
      }

      IMUDefinition[] imuDefinitions = estimatorModel.getIMUDefinitions();

      int numberOfImuSensors = imuDefinitions.length;
      imuLinearAccelerations = new Vector3D32[numberOfImuSensors];
      rawImuAngularVelocities = new Vector3D32[numberOfImuSensors];
      imuOrientationsAsMatrix = new RotationMatrix[numberOfImuSensors];
      imuOrientations = new Quaternion32[numberOfImuSensors];

      for (int imuSensorIndex = 0; imuSensorIndex < numberOfImuSensors; imuSensorIndex++)
      {
         imuLinearAccelerations[imuSensorIndex] = new Vector3D32();
         rawImuAngularVelocities[imuSensorIndex] = new Vector3D32();
         imuOrientationsAsMatrix[imuSensorIndex] = new RotationMatrix();
         imuOrientations[imuSensorIndex] = new Quaternion32();
      }
      ForceSensorDefinition[] forceSensorDefinitions = jointConfigurationGathererAndProducer.getForceSensorDefinitions();
      OneDoFJoint[] joints = jointConfigurationGathererAndProducer.getJoints();
      robotConfigurationDataRingBuffer = new ConcurrentRingBuffer<>(new RobotConfigurationDataBuilder(joints, forceSensorDefinitions, imuDefinitions), 16);
      if (auxiliaryRobotDataProvider == null)
         atlasAuxiliaryRobotDataRingBuffer = null;
      else
         atlasAuxiliaryRobotDataRingBuffer = new ConcurrentRingBuffer<>(auxiliaryRobotDataProvider::newAuxiliaryRobotDataInstance, 16);
      startWriterThread();
   }

   private void setupForceSensorMassCompensators(FullRobotModel estimatorModel, SideDependentList<String> wristForceSensorNames)
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
      dataProducer.registerPacketToSkipQueue(RobotConfigurationData.class);
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
                     dataProducer.skipQueueAndSend(robotConfigData);
                  }
                  robotConfigurationDataRingBuffer.flush();
               }

               if (atlasAuxiliaryRobotDataRingBuffer != null && atlasAuxiliaryRobotDataRingBuffer.poll())
               {
                  AtlasAuxiliaryRobotData auxData;
                  while ((auxData = atlasAuxiliaryRobotDataRingBuffer.read()) != null)
                  {
                     dataProducer.skipQueueAndSend(auxData);
                  }
                  atlasAuxiliaryRobotDataRingBuffer.flush();
               }
            }
            catch (Throwable throwable)
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

      RobotConfigurationData configData = robotConfigurationDataRingBuffer.next();
      if (configData != null)
      {
         long timestamp = sensorTimestampHolder.getVisionSensorTimestamp();
         long pps = sensorTimestampHolder.getSensorHeadPPSTimestamp();
         jointConfigurationGathererAndProducer.packEstimatorJoints(timestamp, pps, configData);

         if (sensorRawOutputMapReadOnly != null)
         {
            configData.imuSensorData.clear();

            List<? extends IMUSensorReadOnly> imuRawOutputs = sensorRawOutputMapReadOnly.getIMURawOutputs();
            for (int sensorNumber = 0; sensorNumber < imuRawOutputs.size(); sensorNumber++)
            {
               IMUSensorReadOnly imuSensor = imuRawOutputs.get(sensorNumber);
               IMUPacket imuPacketToPack = configData.imuSensorData.add();

               imuSensor.getLinearAccelerationMeasurement(imuLinearAccelerations[sensorNumber]);
               imuSensor.getOrientationMeasurement(imuOrientationsAsMatrix[sensorNumber]);
               imuOrientations[sensorNumber].set(imuOrientationsAsMatrix[sensorNumber]);
               imuSensor.getAngularVelocityMeasurement(rawImuAngularVelocities[sensorNumber]);

               imuPacketToPack.set(imuLinearAccelerations[sensorNumber], imuOrientations[sensorNumber], rawImuAngularVelocities[sensorNumber]);
            }
         }

         configData.setRobotMotionStatus(robotMotionStatusFromController.getCurrentRobotMotionStatus().toByte());

         LastPacket lastPacket = dataProducer.getLastPacket();
         if (lastPacket != null)
         {
            configData.setLastReceivedPacketTypeID(netClassList.getID(lastPacket.getPacket()));
            configData.setLastReceivedPacketUniqueId(lastPacket.getUniqueId());
            configData.setLastReceivedPacketRobotTimestamp(lastPacket.getReceivedTimestamp());
         }
         else
         {
            configData.setLastReceivedPacketTypeID(-1);
            configData.setLastReceivedPacketUniqueId(-1);
            configData.setLastReceivedPacketRobotTimestamp(-1);
         }

         robotConfigurationDataRingBuffer.commit();
      }

      if (atlasAuxiliaryRobotDataRingBuffer != null)
      {
         AtlasAuxiliaryRobotData auxData = atlasAuxiliaryRobotDataRingBuffer.next();

         if (auxData != null)
         {
            if (sensorRawOutputMapReadOnly != null)
               auxData.set(sensorRawOutputMapReadOnly.getAuxiliaryRobotData());

            robotConfigurationDataRingBuffer.commit();
         }
      }
   }

   public static class RobotConfigurationDataBuilder implements us.ihmc.concurrent.Builder<RobotConfigurationData>
   {
      private final OneDoFJoint[] joints;
      private final ForceSensorDefinition[] forceSensorDefinitions;
      private final IMUDefinition[] imuDefinitions;

      public RobotConfigurationDataBuilder(OneDoFJoint[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions)
      {
         this.joints = joints;
         this.forceSensorDefinitions = forceSensorDefinitions;
         this.imuDefinitions = imuDefinitions;
      }

      @Override
      public RobotConfigurationData newInstance()
      {
         return RobotConfigurationDataFactory.create(joints, forceSensorDefinitions, imuDefinitions);
      }
   }

   public void stop()
   {
      scheduler.shutdown();
   }
}