package us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning;

import java.util.HashMap;
import java.util.Set;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

public class AtlasKinematicsConfiguration
{
   /**
    * see OneDoFJoint names without fingers.
    */
   public final HashMap<String, Double> jointConfiguration;

   //   public final HashMap<String, Double> rootConfiguration;

   public Vector3D rootTranslation;
   public Quaternion rootOrientation;

   public AtlasKinematicsConfiguration()
   {
      jointConfiguration = new HashMap<String, Double>();

      jointConfiguration.put("back_bkz", 0.0);
      jointConfiguration.put("back_bky", 0.0);
      jointConfiguration.put("back_bkx", 0.0);

      jointConfiguration.put("l_arm_shz", 0.0);
      jointConfiguration.put("l_arm_shx", 0.0);
      jointConfiguration.put("l_arm_ely", 0.0);
      jointConfiguration.put("l_arm_elx", 0.0);
      jointConfiguration.put("l_arm_wry", 0.0);
      jointConfiguration.put("l_arm_wrx", 0.0);
      jointConfiguration.put("l_arm_wry2", 0.0);

      jointConfiguration.put("neck_ry", 0.0);
      jointConfiguration.put("hokuyo_joint", 0.0);

      jointConfiguration.put("r_arm_shz", 0.0);
      jointConfiguration.put("r_arm_shx", 0.0);
      jointConfiguration.put("r_arm_ely", 0.0);
      jointConfiguration.put("r_arm_elx", 0.0);
      jointConfiguration.put("r_arm_wry", 0.0);
      jointConfiguration.put("r_arm_wrx", 0.0);
      jointConfiguration.put("r_arm_wry2", 0.0);

      jointConfiguration.put("l_leg_hpz", 0.0);
      jointConfiguration.put("l_leg_hpx", 0.0);
      jointConfiguration.put("l_leg_hpy", 0.0);
      jointConfiguration.put("l_leg_kny", 0.0);
      jointConfiguration.put("l_leg_aky", 0.0);
      jointConfiguration.put("l_leg_akx", 0.0);

      jointConfiguration.put("r_leg_hpz", 0.0);
      jointConfiguration.put("r_leg_hpx", 0.0);
      jointConfiguration.put("r_leg_hpy", 0.0);
      jointConfiguration.put("r_leg_kny", 0.0);
      jointConfiguration.put("r_leg_aky", 0.0);
      jointConfiguration.put("r_leg_akx", 0.0);

      rootTranslation = new Vector3D();
      rootOrientation = new Quaternion();
   }

   public void putAtlasConfigurationData(AtlasKinematicsConfiguration other)
   {
      Set<String> jointKeySet = other.jointConfiguration.keySet();
      for (String jointName : jointKeySet)
         jointConfiguration.put(jointName, other.jointConfiguration.get(jointName));

      this.rootTranslation.set(other.rootTranslation);
      this.rootOrientation.set(other.rootOrientation);
   }
   
   public void putAtlasConfigurationData(FullHumanoidRobotModel fullRobotModel)
   {
      putJointConfiguration(fullRobotModel.getOneDoFJoints());
      putRootTranslation(fullRobotModel.getRootJoint().getTranslationForReading());
      putRootOrientation(fullRobotModel.getRootJoint().getRotationForReading());
   }

   public void putAtlasConfigurationData(RobotConfigurationData robotConfigurationData)
   {

   }

   public void putJointConfiguration(OneDoFJoint[] oneDoFJoints)
   {
      for(int i=0;i<oneDoFJoints.length;i++)
         jointConfiguration.put(oneDoFJoints[i].getName(), oneDoFJoints[i].getQ());
   }

   public void putRootTranslation(Tuple3DReadOnly rootTranslation)
   {
      this.rootTranslation.set(rootTranslation);
   }

   public void putRootOrientation(Tuple4DReadOnly rootOrientation)
   {
      this.rootOrientation.set(rootOrientation);
   }
   
   public void getOneDoFJoints(OneDoFJoint[] oneDoFJoints)
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         oneDoFJoints[i].setQ(jointConfiguration.get(oneDoFJoints[i].getName()));
         oneDoFJoints[i].setQd(0.0);
         oneDoFJoints[i].setqDesired(jointConfiguration.get(oneDoFJoints[i].getName()));
      }
   }

   public void print()
   {
      Set<String> jointKeySet = this.jointConfiguration.keySet();
      for (String jointName : jointKeySet)
         System.out.println(jointName + " " + jointConfiguration.get(jointName));

      System.out.println(rootTranslation);
      System.out.println(rootOrientation);
   }
}
