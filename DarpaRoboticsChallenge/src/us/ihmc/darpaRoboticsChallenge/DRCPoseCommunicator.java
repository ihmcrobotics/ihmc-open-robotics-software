package us.ihmc.darpaRoboticsChallenge;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.ForceSensorDistalMassCompensator;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.simulationconstructionset.robotController.RawOutputWriter;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
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
   private final PacketCommunicator networkProcessorCommunicator;
   private final JointConfigurationGatherer jointConfigurationGathererAndProducer;
   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;
   private final SideDependentList<String> wristForceSensorNames;

   private final SideDependentList<ReferenceFrame> wristForceSensorFrames = new SideDependentList<ReferenceFrame>();
   private final SideDependentList<ForceSensorDistalMassCompensator> wristForceSensorDistalMassCompensators = new SideDependentList<ForceSensorDistalMassCompensator>();

   private final ConcurrentRingBuffer<RobotConfigurationData> robotConfigurationDataRingBuffer;

   public DRCPoseCommunicator(SDFFullRobotModel estimatorModel, JointConfigurationGatherer jointConfigurationGathererAndProducer,
         PacketCommunicator networkProcessorCommunicator, SensorOutputMapReadOnly sensorOutputMapReadOnly, DRCRobotSensorInformation sensorInformation)
   {
      this.networkProcessorCommunicator = networkProcessorCommunicator;
      this.jointConfigurationGathererAndProducer = jointConfigurationGathererAndProducer;
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;

      this.wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      setupForceSensorMassCompensators(estimatorModel);

      robotConfigurationDataRingBuffer = new ConcurrentRingBuffer<RobotConfigurationData>(new RobotConfigurationDataBuilder(jointConfigurationGathererAndProducer.getJoints(), jointConfigurationGathererAndProducer.getForceSensorDefinitions()), 16);
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
            if (robotConfigurationDataRingBuffer.poll())
            {
               RobotConfigurationData robotConfigData;
               while ((robotConfigData = robotConfigurationDataRingBuffer.read()) != null)
               {
                  networkProcessorCommunicator.send(robotConfigData);
               }
               robotConfigurationDataRingBuffer.flush();
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
      robotConfigurationDataRingBuffer.commit();
   }

   public static class RobotConfigurationDataBuilder implements us.ihmc.concurrent.Builder<RobotConfigurationData>
   {
      private final OneDoFJoint[] joints;
      private final ForceSensorDefinition[] forceSensorDefinitions;

      public RobotConfigurationDataBuilder(OneDoFJoint[] joints, ForceSensorDefinition[] forceSensorDefinitions)
      {
         this.joints = joints;
         this.forceSensorDefinitions = forceSensorDefinitions;
      }

      @Override
      public RobotConfigurationData newInstance()
      {
         return new RobotConfigurationData(joints, forceSensorDefinitions);
      }

   }
   public void stop()
   {
      writeExecutor.shutdown();
   }
}