package us.ihmc.darpaRoboticsChallenge;

import javax.media.j3d.Transform3D;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.messages.controller.RobotPoseData;
import us.ihmc.darpaRoboticsChallenge.networking.dataProducers.DRCJointConfigurationData;
import us.ihmc.darpaRoboticsChallenge.networking.dataProducers.JointConfigurationGatherer;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.TimestampProvider;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RawOutputWriter;

// fills a ring buffer with pose and joint data and in a worker thread passes it to the appropriate consumer 
public class DRCPoseCommunicator implements RawOutputWriter
{
   private final int WORKER_SLEEP_TIME_MILLIS = 1;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame headFrame;
   private final ReferenceFrame lidarFrame;
   private final ReferenceFrame cameraFrame;
   private final ReferenceFrame rootFrame;

   private final Transform3D rootTransform = new Transform3D();
   private final Transform3D headTransform = new Transform3D();
   private final Transform3D cameraTransform = new Transform3D();
   private final Transform3D lidarTransform = new Transform3D();

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

      headFrame = estimatorModel.getHeadBaseFrame();
      String lidarSpindleJointName = null;
      if(sensorInformation.getPrimaryLidarParameters() != null)
      {
     	 lidarSpindleJointName =sensorInformation.getPrimaryLidarParameters().getSensorNameInSdf();
      }
      
      lidarFrame = estimatorModel.getLidarBaseFrame(lidarSpindleJointName);
      cameraFrame = estimatorModel.getCameraFrame(sensorInformation.getPrimaryCameraParameters().getSensorNameInSdf());
      rootFrame = estimatorModel.getRootJoint().getFrameAfterJoint();

      stateRingBuffer = new ConcurrentRingBuffer<State>(State.builder, 8);

      startWriterThread();
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

   // puts the state data into the ring buffer for the output thread
   @Override
   public void write()
   {
      rootFrame.getTransformToDesiredFrame(rootTransform, ReferenceFrame.getWorldFrame());
      cameraFrame.getTransformToDesiredFrame(cameraTransform, ReferenceFrame.getWorldFrame());
      if(lidarFrame != null)
      {
         lidarFrame.getTransformToDesiredFrame(lidarTransform, ReferenceFrame.getWorldFrame());
      }
      headFrame.getTransformToDesiredFrame(headTransform, ReferenceFrame.getWorldFrame());

      State state = stateRingBuffer.next();
      if (state == null)
      {
         return;
      }
      
      long timestamp = timeProvider.getTimestamp();
      jointConfigurationGathererAndProducer.packEstimatorJoints(timestamp, state.jointData, numberOfJoints);
      state.poseData.setAll(timestamp, rootTransform, cameraTransform, lidarTransform, headTransform);

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