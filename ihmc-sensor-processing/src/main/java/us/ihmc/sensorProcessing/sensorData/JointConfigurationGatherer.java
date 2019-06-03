package us.ihmc.sensorProcessing.sensorData;

import java.util.ArrayList;
import java.util.Arrays;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.SpatialVectorMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

public class JointConfigurationGatherer
{
   private final ArrayList<OneDoFJointBasics> joints = new ArrayList<>();

   private final FloatingJointBasics rootJoint;
   private final Vector3D rootTranslation = new Vector3D();
   private final Quaternion rootOrientation = new Quaternion();

   private final Vector3D rootLinearVelocity = new Vector3D();
   private final Vector3D rootAngularVelocity = new Vector3D();
   private final Vector3D rootLinearAcceleration = new Vector3D();

   private final ForceSensorDefinition[] forceSensorDefinitions;
   private final ArrayList<String> forceSensorNameList = new ArrayList<String>();
   private final ArrayList<ForceSensorDataReadOnly> forceSensorDataList = new ArrayList<>();

   /**
    * The estimated state of the whole robot is packed and sent to the GUI using the DRCJointConfigurationData packet.
    * Hackish shit going on around here: the lidar and finger joints are not packed in the packet.
    * @param estimatorModel
    * @param forceSensorDataHolderForEstimator 
    */
   public JointConfigurationGatherer(FullHumanoidRobotModel estimatorModel, ForceSensorDataHolderReadOnly forceSensorDataHolderForEstimator)
   {
      this.rootJoint = estimatorModel.getRootJoint();

      FullRobotModelUtils.getAllJointsExcludingHands(joints, estimatorModel);

      forceSensorDefinitions = forceSensorDataHolderForEstimator.getForceSensorDefinitions().toArray(new ForceSensorDefinition[forceSensorDataHolderForEstimator.getForceSensorDefinitions().size()]);
      for (ForceSensorDefinition definition : forceSensorDefinitions)
      {
         String sensorName = definition.getSensorName();
         forceSensorNameList.add(sensorName);

         ForceSensorDataReadOnly forceSensorData = forceSensorDataHolderForEstimator.get(definition);
         forceSensorDataList.add(forceSensorData);
      }
   }
   
   public JointConfigurationGatherer(FullRobotModel estimatorModel)
   {
      this.rootJoint = estimatorModel.getRootJoint();
      this.joints.addAll(Arrays.asList(estimatorModel.getOneDoFJoints()));
      this.forceSensorDefinitions = new ForceSensorDefinition[0];
   }

   public int getNumberOfJoints()
   {
      return joints.size();
   }

   public int getNumberOfForceSensors()
   {
      return forceSensorDataList.size();
   }

   public String getForceSensorName(int sensorNumber)
   {
      return forceSensorNameList.get(sensorNumber);
   }

   // fills a DRCJointConfigurationData object on the ConcurrentRingBuffer
   public void packEstimatorJoints(long wallTime, long monotonicTime, long syncTimestamp, RobotConfigurationData jointConfigurationData)
   {
      if (jointConfigurationData == null)
      {
         return;
      }

      rootTranslation.set(rootJoint.getJointPose().getPosition());
      rootOrientation.set(rootJoint.getJointPose().getOrientation());
      rootAngularVelocity.set(rootJoint.getJointTwist().getAngularPart());
      rootLinearVelocity.set(rootJoint.getJointTwist().getLinearPart());
      rootLinearAcceleration.set(rootJoint.getJointAcceleration().getLinearPart());

      jointConfigurationData.getPelvisAngularVelocity().set(rootAngularVelocity);
      jointConfigurationData.getPelvisLinearVelocity().set(rootLinearVelocity);
      jointConfigurationData.getPelvisLinearAcceleration().set(rootLinearAcceleration);
      jointConfigurationData.getRootTranslation().set(rootTranslation);
      jointConfigurationData.getRootOrientation().set(rootOrientation);
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
      return forceSensorDefinitions;
   }
}
