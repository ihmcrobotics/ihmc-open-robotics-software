package us.ihmc.sensorProcessing.sensorData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.SpatialVectorMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialAccelerationBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

public class JointConfigurationGatherer
{
   private final List<OneDoFJointBasics> joints = new ArrayList<>();

   private final FloatingJointBasics rootJoint;
   private final List<ForceSensorDefinition> forceSensorDefinitions = new ArrayList<>();
   private final List<ForceSensorDataReadOnly> forceSensorDataList = new ArrayList<>();

   /**
    * The estimated state of the whole robot is packed and sent to the GUI using the
    * DRCJointConfigurationData packet. Hackish shit going on around here: the lidar and finger joints
    * are not packed in the packet.
    * 
    * @param estimatorModel
    * @param forceSensorDataHolderForEstimator
    */
   public JointConfigurationGatherer(FullHumanoidRobotModel estimatorModel, ForceSensorDataHolderReadOnly forceSensorDataHolderForEstimator)
   {
      this.rootJoint = estimatorModel.getRootJoint();

      FullRobotModelUtils.getAllJointsExcludingHands(joints, estimatorModel);

      forceSensorDefinitions.addAll(forceSensorDataHolderForEstimator.getForceSensorDefinitions());

      for (ForceSensorDefinition definition : forceSensorDefinitions)
      {
         ForceSensorDataReadOnly forceSensorData = forceSensorDataHolderForEstimator.get(definition);
         forceSensorDataList.add(forceSensorData);
      }
   }

   public JointConfigurationGatherer(FullRobotModel estimatorModel)
   {
      this.rootJoint = estimatorModel.getRootJoint();
      this.joints.addAll(Arrays.asList(estimatorModel.getOneDoFJoints()));
   }

   public int getNumberOfJoints()
   {
      return joints.size();
   }

   public int getNumberOfForceSensors()
   {
      return forceSensorDataList.size();
   }

   // fills a DRCJointConfigurationData object on the ConcurrentRingBuffer
   public void packEstimatorJoints(long wallTime, long monotonicTime, long syncTimestamp, RobotConfigurationData jointConfigurationData)
   {
      if (jointConfigurationData == null)
      {
         return;
      }

      Pose3DBasics rootJointPose = rootJoint.getJointPose();
      FixedFrameTwistBasics rootJointTwist = rootJoint.getJointTwist();
      FixedFrameSpatialAccelerationBasics rootJointAcceleration = rootJoint.getJointAcceleration();

      jointConfigurationData.getRootOrientation().set(rootJointPose.getOrientation());
      jointConfigurationData.getRootTranslation().set(rootJointPose.getPosition());
      jointConfigurationData.getPelvisAngularVelocity().set(rootJointTwist.getAngularPart());
      jointConfigurationData.getPelvisLinearVelocity().set(rootJointTwist.getLinearPart());
      jointConfigurationData.getPelvisLinearAcceleration().set(rootJointAcceleration.getLinearPart());
      RobotConfigurationDataFactory.packJointState(jointConfigurationData, joints);
      jointConfigurationData.setWallTime(wallTime);
      jointConfigurationData.setMonotonicTime(monotonicTime);
      jointConfigurationData.setSyncTimestamp(syncTimestamp);

      RecyclingArrayList<SpatialVectorMessage> momentAndForceDataAllForceSensors = jointConfigurationData.getForceSensorData();
      momentAndForceDataAllForceSensors.clear();

      for (int sensorNumber = 0; sensorNumber < getNumberOfForceSensors(); sensorNumber++)
      {
         SpatialVectorMessage wrench = momentAndForceDataAllForceSensors.add();
         forceSensorDataList.get(sensorNumber).getWrench(wrench.getAngularPart(), wrench.getLinearPart());
      }
   }

   public OneDoFJointBasics[] getJoints()
   {
      return joints.toArray(new OneDoFJointBasics[joints.size()]);
   }

   public ForceSensorDefinition[] getForceSensorDefinitions()
   {
      return forceSensorDefinitions.toArray(new ForceSensorDefinition[forceSensorDefinitions.size()]);
   }
}
