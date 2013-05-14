package us.ihmc.darpaRoboticsChallenge;

import javax.media.j3d.Transform3D;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.messages.controller.RobotPoseData;
import us.ihmc.darpaRoboticsChallenge.networking.dataProducers.JointConfigurationGathererAndProducer;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.TimestampProvider;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RawOutputWriter;

public class DRCPoseCommunicator implements RawOutputWriter
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame lidarFrame;
   private final ReferenceFrame cameraFrame;
   private final ReferenceFrame rootFrame;
   
   private final Transform3D rootTransform = new Transform3D();
   private final Transform3D cameraTransform = new Transform3D();
   private final Transform3D lidarTransform = new Transform3D();
   
   private final ObjectCommunicator networkProcessorCommunicator;
   private final JointConfigurationGathererAndProducer jointConfigurationGathererAndProducer;
   private final TimestampProvider timeProvider;
   
   public DRCPoseCommunicator(SDFFullRobotModel estimatorModel, JointConfigurationGathererAndProducer jointConfigurationGathererAndProducer, DRCRobotJointMap jointMap, ObjectCommunicator networkProcessorCommunicator, TimestampProvider timestampProvider)
   {
      this.networkProcessorCommunicator = networkProcessorCommunicator;
      this.jointConfigurationGathererAndProducer = jointConfigurationGathererAndProducer;
      this.timeProvider = timestampProvider;
      
      lidarFrame = estimatorModel.getLidarBaseFrame(jointMap.getLidarSensorName());
      cameraFrame = estimatorModel.getCameraFrame(jointMap.getLeftCameraName());
      rootFrame = estimatorModel.getRootJoint().getFrameAfterJoint();
   }
   
   
   public void initialize()
   {
      // TODO Auto-generated method stub

   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return getClass().getSimpleName();
   }

   public String getDescription()
   {
      return getName();
   }

   public void write()
   {
      rootFrame.getTransformToDesiredFrame(rootTransform, ReferenceFrame.getWorldFrame());
      cameraFrame.getTransformToDesiredFrame(cameraTransform, ReferenceFrame.getWorldFrame());
      lidarFrame.getTransformToDesiredFrame(lidarTransform, ReferenceFrame.getWorldFrame());
    
      long timestamp = timeProvider.getTimestamp();
      RobotPoseData robotPoseData = new RobotPoseData(timestamp,  rootTransform, cameraTransform, lidarTransform);
      networkProcessorCommunicator.consumeObject(robotPoseData);
      
      jointConfigurationGathererAndProducer.updateEstimatorJoints(timestamp);
   }

}
