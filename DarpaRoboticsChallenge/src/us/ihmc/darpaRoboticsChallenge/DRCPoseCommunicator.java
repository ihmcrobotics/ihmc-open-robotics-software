package us.ihmc.darpaRoboticsChallenge;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.sensing.RobotPoseData;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorParameters;
import us.ihmc.sensorProcessing.sensorData.ForceSensorDistalMassCompensator;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.simulationconstructionset.robotController.RawOutputWriter;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
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
   private final ReferenceFrame rootFrame;
   private ReferenceFrame[] pointCloudFrames;
   private ReferenceFrame[] lidarFrames;
   private ReferenceFrame[] cameraFrames;
   private RigidBodyTransform[] cameraPoses, lidarPoses, pointCloudPoses;

   private final RigidBodyTransform rootTransform = new RigidBodyTransform();

   private final PacketCommunicator networkProcessorCommunicator;
   private final JointConfigurationGatherer jointConfigurationGathererAndProducer;
   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;
   private final SideDependentList<String> wristForceSensorNames;

   private final SideDependentList<ReferenceFrame> wristForceSensorFrames = new SideDependentList<ReferenceFrame>();
   private final SideDependentList<ForceSensorDistalMassCompensator> wristForceSensorDistalMassCompensators = new SideDependentList<ForceSensorDistalMassCompensator>();

   private final ConcurrentRingBuffer<State> stateRingBuffer;

   public DRCPoseCommunicator(SDFFullRobotModel estimatorModel, JointConfigurationGatherer jointConfigurationGathererAndProducer,
         PacketCommunicator networkProcessorCommunicator, SensorOutputMapReadOnly sensorOutputMapReadOnly, DRCRobotSensorInformation sensorInformation)
   {
      this.networkProcessorCommunicator = networkProcessorCommunicator;
      this.jointConfigurationGathererAndProducer = jointConfigurationGathererAndProducer;
      this.sensorOutputMapReadOnly = sensorOutputMapReadOnly;

      this.wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      setupForceSensorMassCompensators(estimatorModel);

      rootFrame = estimatorModel.getRootJoint().getFrameAfterJoint();
      stateRingBuffer = new ConcurrentRingBuffer<State>(new State.Builder(jointConfigurationGathererAndProducer.getJoints(), jointConfigurationGathererAndProducer.getForceSensorDefinitions()), 16);
      setupSensorFrames(sensorInformation, estimatorModel);
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

   private void setupSensorFrames(DRCRobotSensorInformation sensorInformation, SDFFullRobotModel estimatorModel)
   {

      DRCRobotCameraParameters[] cameraParams = sensorInformation.getCameraParameters();
      DRCRobotLidarParameters[] lidarParams = sensorInformation.getLidarParameters();
      DRCRobotPointCloudParameters[] pointCloudParams = sensorInformation.getPointCloudParameters();

      if (cameraParams != null)
      {
         cameraFrames = new ReferenceFrame[cameraParams.length];
         cameraPoses = new RigidBodyTransform[cameraParams.length];
         setupSensorFrames(cameraParams, cameraFrames, cameraPoses, estimatorModel);
      }
      else
      {
         cameraFrames = new ReferenceFrame[0];
         cameraPoses = new RigidBodyTransform[0];
      }

      if (lidarParams != null)
      {
         lidarFrames = new ReferenceFrame[lidarParams.length];
         lidarPoses = new RigidBodyTransform[lidarParams.length];
         setupSensorFrames(lidarParams, lidarFrames, lidarPoses, estimatorModel);
      }
      else
      {
         lidarFrames = new ReferenceFrame[0];
         lidarPoses = new RigidBodyTransform[0];
      }

      if (pointCloudParams != null)
      {
         pointCloudFrames = new ReferenceFrame[pointCloudParams.length];
         pointCloudPoses = new RigidBodyTransform[pointCloudParams.length];
         setupSensorFrames(pointCloudParams, pointCloudFrames, pointCloudPoses, estimatorModel);
      }
      else
      {
         pointCloudFrames = new ReferenceFrame[0];
         pointCloudPoses = new RigidBodyTransform[0];
      }
   }

   private void setupSensorFrames(DRCRobotSensorParameters[] params, ReferenceFrame[] referenceFrames, RigidBodyTransform[] poses,
         SDFFullRobotModel sdfRobotModel)
   {
      for (int i = 0; i < params.length; i++)
      {
         poses[i] = new RigidBodyTransform();
         if (params[i].useRosForTransformFromPoseToSensor())
         {
            referenceFrames[i] = sdfRobotModel.getSensorReferenceFrameByLink(params[i].getPoseFrameForSdf());
         }
         else
         {
            switch (params[i].getSensorType())
            {
            case CAMERA:
               referenceFrames[i] = sdfRobotModel.getCameraFrame(params[i].getSensorNameInSdf());
               if (referenceFrames[i] == null)
               {
                  System.err.println("DrcPoseCommunicator: Camera sensor not found: " + params[i].getSensorNameInSdf());
               }
               break;
            case LIDAR:
               referenceFrames[i] = sdfRobotModel.getLidarBaseFrame(params[i].getSensorNameInSdf());
               if (referenceFrames[i] == null)
               {
                  System.err.println("DrcPoseCommunicator: Lidar sensor not found: " + params[i].getSensorNameInSdf());
               }
               break;
            case POINTCLOUD:
               referenceFrames[i] = sdfRobotModel.getLidarBaseFrame(params[i].getSensorNameInSdf());
               if (referenceFrames[i] == null)
               {
                  System.err.println("DrcPoseCommunicator: PointCloud sensor not found: " + params[i].getSensorNameInSdf());
               }
               break;
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
            if (stateRingBuffer.poll())
            {
               State currentState;
               while ((currentState = stateRingBuffer.read()) != null)
               {
                  RobotPoseData robotPoseData = currentState.poseData;
                  RobotConfigurationData robotConfigData = currentState.jointData;

                  networkProcessorCommunicator.send(robotPoseData);
                  networkProcessorCommunicator.send(robotConfigData);
               }
               stateRingBuffer.flush();
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

   private void upateSensorFrames(ReferenceFrame[] referenceFrames, RigidBodyTransform[] targetTransforms)
   {
      for (int i = 0; i < referenceFrames.length; i++)
      {
         if (referenceFrames[i] != null)
         {
            referenceFrames[i].getTransformToDesiredFrame(targetTransforms[i], ReferenceFrame.getWorldFrame());
         }
      }
   }

   // puts the state data into the ring buffer for the output thread
   @Override
   public void write()
   {
      rootFrame.getTransformToDesiredFrame(rootTransform, ReferenceFrame.getWorldFrame());

      upateSensorFrames(cameraFrames, cameraPoses);
      upateSensorFrames(lidarFrames, lidarPoses);
      upateSensorFrames(pointCloudFrames, pointCloudPoses);

      State state = stateRingBuffer.next();
      if (state == null)
      {
         return;
      }

      long timestamp = sensorOutputMapReadOnly.getVisionSensorTimestamp();
      jointConfigurationGathererAndProducer.packEstimatorJoints(timestamp, state.jointData);
      state.poseData.setAll(timestamp, rootTransform, cameraPoses, lidarPoses, pointCloudPoses);

      stateRingBuffer.commit();
   }

   //this object is just a glorified tuple
   private static class State
   {
      public final RobotPoseData poseData;
      public final RobotConfigurationData jointData;

      public State(OneDoFJoint[] joints, ForceSensorDefinition[] forceSensorDefinitions)
      {
         poseData = new RobotPoseData();
         jointData = new RobotConfigurationData(joints, forceSensorDefinitions);
      }

      public static class Builder implements us.ihmc.concurrent.Builder<State>
      {
         private final OneDoFJoint[] joints;
         private final ForceSensorDefinition[] forceSensorDefinitions;

         public Builder(OneDoFJoint[] joints, ForceSensorDefinition[] forceSensorDefinitions)
         {
            this.joints = joints;
            this.forceSensorDefinitions = forceSensorDefinitions;
         }

         @Override
         public State newInstance()
         {
            return new State(joints, forceSensorDefinitions);
         }

      }
   }

   public void stop()
   {
      writeExecutor.shutdown();
   }
}