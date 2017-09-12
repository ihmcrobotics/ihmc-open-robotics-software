package us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class RobotKinematicsConfiguration
{
   private int jointConfigurationHash;
   public double[] jointConfiguration;

   public Vector3D rootTranslation = new Vector3D();
   public Quaternion rootOrientation = new Quaternion();

   public RobotKinematicsConfiguration()
   {
   }

   public RobotKinematicsConfiguration(RobotKinematicsConfiguration other)
   {
      setRobotConfigurationData(other);
   }

   public RobotKinematicsConfiguration(FullHumanoidRobotModel fullRobotModel)
   {
      setRobotConfigurationData(fullRobotModel);
   }

   public void setRobotConfigurationData(RobotKinematicsConfiguration other)
   {
      jointConfigurationHash = other.jointConfigurationHash;
      jointConfiguration = new double[other.jointConfiguration.length];
      System.arraycopy(other.jointConfiguration, 0, jointConfiguration, 0, jointConfiguration.length);

      this.rootTranslation.set(other.rootTranslation);
      this.rootOrientation.set(other.rootOrientation);
   }

   public void setRobotConfigurationData(FullHumanoidRobotModel fullRobotModel)
   {
      OneDoFJoint[] oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      
      jointConfigurationHash = (int) NameBasedHashCodeTools.computeArrayHashCode(oneDoFJoints);

      jointConfiguration = new double[oneDoFJoints.length];

      for (int i = 0; i < oneDoFJoints.length; i++)
         jointConfiguration[i] = oneDoFJoints[i].getQ();

      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      
      this.rootTranslation.set(rootJoint.getTranslationForReading());
      this.rootOrientation.set(rootJoint.getRotationForReading());
   }

   public void getRobotConfiguration(FullHumanoidRobotModel fullRobotModelToUpdate)
   {
      OneDoFJoint[] oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModelToUpdate);
      getOneDoFJoints(oneDoFJoints);

      FloatingInverseDynamicsJoint rootJoint = fullRobotModelToUpdate.getRootJoint();
      rootJoint.setPosition(rootTranslation);
      rootJoint.setRotation(rootOrientation);
   }

   private void getOneDoFJoints(OneDoFJoint[] oneDoFJoints)
   {
      int jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(oneDoFJoints);
      
      if (jointNameHash != jointConfigurationHash)
         throw new RuntimeException("The robots are different.");
      
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         oneDoFJoints[i].setQ(jointConfiguration[i]);
         oneDoFJoints[i].setQd(0.0);
         oneDoFJoints[i].setqDesired(jointConfiguration[i]);
      }
   }
}
