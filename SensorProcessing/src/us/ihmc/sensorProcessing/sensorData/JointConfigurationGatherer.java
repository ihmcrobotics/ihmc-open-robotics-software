package us.ihmc.sensorProcessing.sensorData;

import java.util.ArrayList;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorData;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModelUtils;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

public class JointConfigurationGatherer
{
   private final ArrayList<OneDoFJoint> joints = new ArrayList<>();

   private final SixDoFJoint rootJoint;
   private final Vector3d rootTranslation = new Vector3d();
   private final Quat4d rootOrientation = new Quat4d();

   private final ForceSensorDefinition[] forceSensorDefinitions;
   private final ArrayList<String> forceSensorNameList = new ArrayList<String>();
   private final ArrayList<ForceSensorData> forceSensorDataList = new ArrayList<>();

   /**
    * The estimated state of the whole robot is packed and sent to the GUI using the DRCJointConfigurationData packet.
    * Hackish shit going on around here: the lidar and finger joints are not packed in the packet.
    * @param estimatorModel
    * @param forceSensorDataHolderForEstimator 
    */
   public JointConfigurationGatherer(FullRobotModel estimatorModel, ForceSensorDataHolder forceSensorDataHolderForEstimator)
   {
      this.rootJoint = estimatorModel.getRootJoint();

      FullRobotModelUtils.getAllJointsExcludingHands(joints, estimatorModel);

      forceSensorDefinitions = forceSensorDataHolderForEstimator.getForceSensorDefinitions().toArray(new ForceSensorDefinition[forceSensorDataHolderForEstimator.getForceSensorDefinitions().size()]);
      for (ForceSensorDefinition definition : forceSensorDefinitions)
      {
         String sensorName = definition.getSensorName();
         forceSensorNameList.add(sensorName);

         ForceSensorData forceSensorData = forceSensorDataHolderForEstimator.get(definition);
         forceSensorDataList.add(forceSensorData);
      }
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

      rootJoint.packTranslation(rootTranslation);
      rootJoint.packRotation(rootOrientation);

      jointConfigurationData.setRootTranslation(rootTranslation);
      jointConfigurationData.setRootOrientation(rootOrientation);
      jointConfigurationData.setJointState(joints);
      jointConfigurationData.setTimestamp(timestamp);
      jointConfigurationData.setSensorHeadPPSTimestamp(sensorHeadPPSTimestamp);

      for (int sensorNumber = 0; sensorNumber < getNumberOfForceSensors(); sensorNumber++)
      {
         DenseMatrix64F forceAndMomentVector = jointConfigurationData.getMomentAndForceVectorForSensor(sensorNumber);
         forceSensorDataList.get(sensorNumber).packWrench(forceAndMomentVector);
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
