package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.HashMap;
import java.util.Set;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

public class AtlasKinematicsConfiguration
{
   /**
    * see OneDoFJoint names without fingers.
    */
   public final HashMap<String, Double> jointConfiguration;
   
   /**
    * translation (3) - Point3D values
    * orientation (4) - Quaternion values
    */
   public final HashMap<String, Double> rootConfiguration;
      
   public AtlasKinematicsConfiguration()
   {
      jointConfiguration = new HashMap<String, Double>();
      rootConfiguration = new HashMap<String, Double>();
      
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
      
      rootConfiguration.put("translation_x", 0.0);
      rootConfiguration.put("translation_y", 0.0);
      rootConfiguration.put("translation_z", 0.0);
      
      rootConfiguration.put("orientation_x", 0.0);
      rootConfiguration.put("orientation_y", 0.0);
      rootConfiguration.put("orientation_z", 0.0);
      rootConfiguration.put("orientation_s", 0.0);
   }
   
   public AtlasKinematicsConfiguration(AtlasKinematicsConfiguration other)
   {      
      jointConfiguration = new HashMap<String, Double>();
      rootConfiguration = new HashMap<String, Double>();
      
      Set<String> jointKeySet = other.jointConfiguration.keySet();
      for(String jointName : jointKeySet)
         jointConfiguration.put(jointName, other.jointConfiguration.get(jointName));
      
      Set<String> rootKeySet = other.rootConfiguration.keySet();
      for(String rootName : rootKeySet)
         rootConfiguration.put(rootName, other.rootConfiguration.get(rootName));
   }
   
   public void putOtherAtlasKinematicsConfiguration(AtlasKinematicsConfiguration other)
   {
      Set<String> jointKeySet = other.jointConfiguration.keySet();
      for(String jointName : jointKeySet)
         jointConfiguration.put(jointName, other.jointConfiguration.get(jointName));
      
      Set<String> rootKeySet = other.rootConfiguration.keySet();
      for(String rootName : rootKeySet)
         rootConfiguration.put(rootName, other.rootConfiguration.get(rootName));
   }
   
   public void putRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      
   }
   
   public void putRootTranslation()
   {
      
   }
   
   public void putRootOrientation()
   {
      
   }
   
   public void getRootTranslation()
   {
      
   }
   
   public void getRootOrientation()
   {
      
   }
}
