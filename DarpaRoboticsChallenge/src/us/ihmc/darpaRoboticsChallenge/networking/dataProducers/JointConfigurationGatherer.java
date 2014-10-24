package us.ihmc.darpaRoboticsChallenge.networking.dataProducers;

import java.util.ArrayList;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorData;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

public class JointConfigurationGatherer
{
   private final ArrayList<OneDoFJoint> joints = new ArrayList<>();

   private final SixDoFJoint rootJoint;
   private final Vector3d rootTranslation = new Vector3d();
   private final Quat4d rootOrientation = new Quat4d();
   
   private final ArrayList<ForceSensorData> forceSensorData = new ArrayList<>();

   /**
    * The estimated state of the whole robot is packed and sent to the GUI using the DRCJointConfigurationData packet.
    * Hackish shit going on around here: the lidar and finger joints are not packed in the packet.
    * @param estimatorModel
    * @param forceSensorDataHolderForEstimator 
    */
   public JointConfigurationGatherer(FullRobotModel estimatorModel, ForceSensorDataHolder forceSensorDataHolderForEstimator)
   {
      this.rootJoint = estimatorModel.getRootJoint();

      estimatorModel.getOneDoFJoints(joints);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = estimatorModel.getHand(robotSide);
         if (hand != null)
         {
            OneDoFJoint[] fingerJoints = ScrewTools.filterJoints(ScrewTools.computeSubtreeJoints(hand), OneDoFJoint.class);
            for (OneDoFJoint fingerJoint : fingerJoints)
            {
               joints.remove(fingerJoint);
            }
         }
      }
      
      for(ForceSensorDefinition definition : forceSensorDataHolderForEstimator.getForceSensorDefinitions())
      {
         forceSensorData.add(forceSensorDataHolderForEstimator.get(definition));
      }
   }
   
   public int getNumberOfJoints()
   {
      return joints.size();
   }
   
   public int getNumberOfForceSensors()
   {
      return forceSensorData.size();
   }

   // fills a DRCJointConfigurationData object on the ConcurrentRingBuffer
   public void packEstimatorJoints(long timestamp, RobotConfigurationData jointConfigurationData)
   {
      if (jointConfigurationData == null)
      {
         return;
      }
      
      rootJoint.packTranslation(rootTranslation);
      rootJoint.packRotation(rootOrientation);

      jointConfigurationData.setRootTranslation(rootTranslation);
      jointConfigurationData.setRootOrientation(rootOrientation);
      jointConfigurationData.setJointAngles(joints);
      jointConfigurationData.setSimTime(timestamp);
      
      for(int i = 0; i < forceSensorData.size(); i++)
      {
         forceSensorData.get(i).packWrench(jointConfigurationData.getForceSensorData(i));
      }
   }
}
