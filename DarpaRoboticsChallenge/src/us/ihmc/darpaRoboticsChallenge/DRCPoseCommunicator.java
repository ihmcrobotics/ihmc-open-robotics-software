package us.ihmc.darpaRoboticsChallenge;

import javax.media.j3d.Transform3D;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.sensing.RobotPoseData;
import us.ihmc.communication.producers.DRCJointConfigurationData;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotLidarParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPointCloudParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorParameters;
import us.ihmc.darpaRoboticsChallenge.networking.dataProducers.JointConfigurationGatherer;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.TimestampProvider;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.robotController.RawOutputWriter;

// fills a ring buffer with pose and joint data and in a worker thread passes it to the appropriate consumer 
public class DRCPoseCommunicator implements RawOutputWriter
{
   private final int WORKER_SLEEP_TIME_MILLIS = 1;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame rootFrame;
   private ReferenceFrame[] pointCloudFrames;
   private ReferenceFrame[] lidarFrames;
   private ReferenceFrame[] cameraFrames;
   private Transform3D[] cameraPoses, lidarPoses, pointCloudPoses;

   private final Transform3D rootTransform = new Transform3D();

   private final ObjectCommunicator networkProcessorCommunicator;
   private final JointConfigurationGatherer jointConfigurationGathererAndProducer;
   private final TimestampProvider timeProvider;
   private State currentState;

   private final ConcurrentRingBuffer<State> stateRingBuffer;
   private final int numberOfJoints;

   public DRCPoseCommunicator(SDFFullRobotModel estimatorModel, JointConfigurationGatherer jointConfigurationGathererAndProducer,
         ObjectCommunicator networkProcessorCommunicator, TimestampProvider timestampProvider, DRCRobotModel robotModel)
   {
      DRCRobotJointMap jointMap = robotModel.getJointMap();
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      numberOfJoints = jointMap.getOrderedJointNames().length;
      this.networkProcessorCommunicator = networkProcessorCommunicator;
      this.jointConfigurationGathererAndProducer = jointConfigurationGathererAndProducer;
      this.timeProvider = timestampProvider;

      rootFrame = estimatorModel.getRootJoint().getFrameAfterJoint();
      stateRingBuffer = new ConcurrentRingBuffer<State>(State.builder, 8);
      setupSensorFrames(sensorInformation, estimatorModel);
      startWriterThread();
   }

   private void setupSensorFrames(DRCRobotSensorInformation sensorInformation,SDFFullRobotModel estimatorModel)
   {

      DRCRobotCameraParameters[] cameraParams = sensorInformation.getCameraParameters();
      DRCRobotLidarParameters[] lidarParams = sensorInformation.getLidarParameters();
      DRCRobotPointCloudParameters[] pointCloudParams = sensorInformation.getPointCloudParameters();
      
      if(cameraParams != null)
      {
         cameraFrames = new ReferenceFrame[cameraParams.length];
         cameraPoses = new Transform3D[cameraParams.length];
         setupSensorFrames(cameraParams,cameraFrames,cameraPoses,estimatorModel);
      } else {
         cameraFrames = new ReferenceFrame[0];
         cameraPoses = new Transform3D[0];
      }
      
      if(lidarParams != null)
      {
         lidarFrames = new ReferenceFrame[lidarParams.length];
         lidarPoses = new Transform3D[lidarParams.length];
         setupSensorFrames(lidarParams,lidarFrames,lidarPoses,estimatorModel);
      } else {
         lidarFrames = new ReferenceFrame[0];
         lidarPoses = new Transform3D[0];
      }
      
      if(pointCloudParams != null)
      {
         pointCloudFrames = new ReferenceFrame[pointCloudParams.length];
         pointCloudPoses = new Transform3D[pointCloudParams.length];
         setupSensorFrames(pointCloudParams,pointCloudFrames,pointCloudPoses,estimatorModel);
      } else {
         pointCloudFrames = new ReferenceFrame[0];
         pointCloudPoses = new Transform3D[0];
      }
   }
   
   private void setupSensorFrames(DRCRobotSensorParameters[] params, ReferenceFrame[] referenceFrames, Transform3D[] poses, SDFFullRobotModel sdfRobotModel)
   {
      for(int i = 0; i < params.length; i++)
      {
         poses[i] = new Transform3D();
         if(params[i].useRosForTransformFromPoseToSensor())
         {
            referenceFrames[i] = sdfRobotModel.getSensorReferenceFrameByLink(params[i].getPoseFrameForSdf());
         } else {
            switch(params[i].getSensorType())
            {
            case CAMERA:
               referenceFrames[i] = sdfRobotModel.getCameraFrame(params[i].getSensorNameInSdf());
               if(referenceFrames[i] == null)
               {
                  System.err.println("DrcPoseCommunicator: Camera sensor not found: " + params[i].getSensorNameInSdf());
               }
               break;
            case LIDAR:
               referenceFrames[i] = sdfRobotModel.getLidarBaseFrame(params[i].getSensorNameInSdf());
               if(referenceFrames[i] == null)
               {
                  System.err.println("DrcPoseCommunicator: Lidar sensor not found: " + params[i].getSensorNameInSdf());
               }
               break;
            case POINTCLOUD:
               referenceFrames[i] = sdfRobotModel.getLidarBaseFrame(params[i].getSensorNameInSdf());
               if(referenceFrames[i] == null)
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
      AsyncContinuousExecutor.executeContinuously(new Runnable()
      {
         @Override
         public void run()
         {
            if (stateRingBuffer.poll())
            {
               while ((currentState = stateRingBuffer.read()) != null)
               {
                  if (networkProcessorCommunicator == null)
                  {
                     System.out.println("Net Proc Comm");
                  }
                  if (currentState.poseData == null)
                  {
                     System.out.println("Pose Data");
                  }
                  networkProcessorCommunicator.consumeObject(currentState.poseData);
                  networkProcessorCommunicator.consumeObject(currentState.jointData);
               }
               stateRingBuffer.flush();
            }
         }
      }, WORKER_SLEEP_TIME_MILLIS, "DRC Pose Communicator");
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

   private void upateSensorFrames(ReferenceFrame[] referenceFrames, Transform3D[] targetTransforms)
   {
      for (int i = 0; i < referenceFrames.length; i++)
      {
         if(referenceFrames[i] != null)
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

      long timestamp = timeProvider.getTimestamp();
      jointConfigurationGathererAndProducer.packEstimatorJoints(timestamp, state.jointData, numberOfJoints);
      state.poseData.setAll(timestamp, rootTransform, cameraPoses, lidarPoses, pointCloudPoses);

      stateRingBuffer.commit();
   }

   //this object is just a glorified tuple
   private static class State
   {
      public final RobotPoseData poseData;
      public final DRCJointConfigurationData jointData;

      public static final Builder<State> builder = new Builder<State>()
      {
         @Override
         public State newInstance()
         {
            return new State();
         }
      };

      public State()
      {
         poseData = new RobotPoseData();
         jointData = new DRCJointConfigurationData();
      }
   }
}