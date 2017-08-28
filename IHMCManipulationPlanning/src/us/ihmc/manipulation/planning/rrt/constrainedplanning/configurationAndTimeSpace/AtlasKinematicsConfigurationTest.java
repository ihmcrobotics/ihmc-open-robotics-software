package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.AtlasKinematicsConfiguration;

public class AtlasKinematicsConfigurationTest
{
   public static void main(String[] args)
   {
      System.out.println("HI!");
      
      AtlasKinematicsConfiguration configurationOne = new AtlasKinematicsConfiguration();
      
      System.out.println(configurationOne.jointConfiguration.get("l_arm_shz"));
      
      System.out.println(configurationOne.jointConfiguration.values().size());
      
      configurationOne.jointConfiguration.put("l_arm_shz", 1.1);
      
      System.out.println(configurationOne.jointConfiguration.get("l_arm_shz"));
      
      AtlasKinematicsConfiguration configurationTwo = new AtlasKinematicsConfiguration();
      
      configurationTwo.putAtlasConfigurationData(configurationOne);
      
      System.out.println(configurationTwo.jointConfiguration.get("l_arm_shz"));
      
      AtlasKinematicsConfiguration configurationThree = new AtlasKinematicsConfiguration();
      
      configurationThree.putAtlasConfigurationData(configurationTwo);
      
      System.out.println(configurationThree.jointConfiguration.get("l_arm_shz"));
      
      
      
      configurationThree.print();
   }
}
