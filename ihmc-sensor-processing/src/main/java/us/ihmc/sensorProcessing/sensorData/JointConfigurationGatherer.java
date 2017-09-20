package us.ihmc.sensorProcessing.sensorData;

import java.util.ArrayList;
import java.util.Arrays;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

public class JointConfigurationGatherer
{
   private final ArrayList<OneDoFJoint> joints = new ArrayList<>();

   private final FloatingInverseDynamicsJoint rootJoint;
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
   public void packEstimatorJoints(long timestamp, long sensorHeadPPSTimestamp, RobotConfigurationData jointConfigurationData)
   {
      if (jointConfigurationData == null)
      {
         return;
      }

      rootJoint.getTranslation(rootTranslation);
      rootJoint.getRotation(rootOrientation);
      rootJoint.getAngularVelocity(rootAngularVelocity);
      rootJoint.getLinearVelocity(rootLinearVelocity);
      rootJoint.getLinearAcceleration(rootLinearAcceleration);

      jointConfigurationData.setPelvisAngularVelocity(rootAngularVelocity);
      jointConfigurationData.setPelvisLinearVelocity(rootLinearVelocity);
      jointConfigurationData.setPelvisLinearAcceleration(rootLinearAcceleration);
      jointConfigurationData.setRootTranslation(rootTranslation);
      jointConfigurationData.setRootOrientation(rootOrientation);
      jointConfigurationData.setJointState(joints);
      jointConfigurationData.setTimestamp(timestamp);
      jointConfigurationData.setSensorHeadPPSTimestamp(sensorHeadPPSTimestamp);

      for (int sensorNumber = 0; sensorNumber < getNumberOfForceSensors(); sensorNumber++)
      {
         float[] forceAndMomentVector = jointConfigurationData.getMomentAndForceVectorForSensor(sensorNumber);
         forceSensorDataList.get(sensorNumber).getWrench(forceAndMomentVector);
      }
   }

   public OneDoFJoint[] getJoints()
   {
      return joints.toArray(new OneDoFJoint[joints.size()]);
   }

   public ForceSensorDefinition[] getForceSensorDefinitions()
   {
      return forceSensorDefinitions;
   }
}
