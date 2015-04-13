package us.ihmc.darpaRoboticsChallenge;

import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.dataobjects.IMUPacket;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.ForceSensorDistalMassCompensator;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.simulationconstructionset.robotController.RawOutputWriter;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.humanoidRobot.model.RobotMotionStatusHolder;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

// fills a ring buffer with pose and joint data and in a worker thread passes it to the appropriate consumer 
public class DRCPoseCommunicator implements RawOutputWriter
{
   private final int WORKER_SLEEP_TIME_MILLIS = 1;

   private final ScheduledExecutorService writeExecutor = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("DRCPoseCommunicator"));

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final GlobalDataProducer dataProducer;
   private final JointConfigurationGatherer jointConfigurationGathererAndProducer;
   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;
   private final SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly;
   private final SideDependentList<String> wristForceSensorNames;
   private final RobotMotionStatusHolder robotMotionStatusFromController;

   private final SideDependentList<ReferenceFrame> wristForceSensorFrames = new SideDependentList<ReferenceFrame>();
   private final SideDependentList<ForceSensorDistalMassCompensator> wristForceSensorDistalMassCompensators = new SideDependentList<ForceSensorDistalMassCompensator>();

   private final ConcurrentRingBuffer<RobotConfigurationData> robotConfigurationDataRingBuffer;

   public DRCPoseCommunicator(SDFFullRobotModel estimatorModel, JointConfigurationGatherer jointConfigurationGathererAndProducer, SensorReader sensorReader,
         GlobalDataProducer dataProducer, SensorOutputMapReadOnly sensorOutputMapReadOnly, SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly, RobotMotionStatusHolder robotMotionStatusFromController, DRCRobotSensorInformation sensorInformation)
   {
      this.dataProducer = dataProducer;
      this.jointConfigurationGathererAndProducer = jointConfigurationGathererAndProducer;
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;
      this.sensorRawOutputMapReadOnly = sensorRawOutputMapReadOnly;
      this.robotMotionStatusFromController = robotMotionStatusFromController;
      this.wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      
      setupForceSensorMassCompensators(estimatorModel);

      IMUDefinition[] imuDefinitions = estimatorModel.getIMUDefinitions();
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
            if (forceSensorName == wristForceSensorNames.get(robotSide))
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
      writeExecutor.scheduleAtFixedRate(new Runnable()
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

      }, 0, WORKER_SLEEP_TIME_MILLIS, TimeUnit.MILLISECONDS);

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
   Vector3d imuLinearAcceleration = new Vector3d();
   Vector3d rawImuAngularVelocityToPack = new Vector3d();
   Matrix3d imuOrientationAsMatrix = new Matrix3d();
   Quat4d imuOrientationAsQuat= new Quat4d();

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
         
         imuSensor.getLinearAccelerationMeasurement(imuLinearAcceleration);
         imuSensor.getOrientationMeasurement(imuOrientationAsMatrix);
         RotationFunctions.setQuaternionBasedOnMatrix3d(imuOrientationAsQuat, imuOrientationAsMatrix);
         imuSensor.getAngularVelocityMeasurement(rawImuAngularVelocityToPack);
         
         imuPacketToPack.set(imuLinearAcceleration, imuOrientationAsQuat, rawImuAngularVelocityToPack);
      }
      
      state.setRobotMotionStatus(robotMotionStatusFromController.getCurrentRobotMotionStatus());

      state.setAuxiliaryRobotData(sensorRawOutputMapReadOnly.getAuxiliaryRobotData());
      
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
      writeExecutor.shutdown();
   }
}