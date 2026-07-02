package us.ihmc.zulu;

import us.ihmc.avatar.drcRobot.RobotVersion;
import us.ihmc.handsros2.HandType;
import us.ihmc.zulu.parameters.model.ZuluPhysicalProperties;
import us.ihmc.zulu.parameters.model.ZULUURDFParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Collection;
import java.util.List;

public enum ZuluVersion implements RobotVersion
{
   V1_FULL_ROBOT(ZuluVersion.V1_RESOURCE_DIRECTORY, List.of(ZULUURDFParameters.URDF_FULL_BODY), null);

   private static final String V1_RESOURCE_DIRECTORY = "us/ihmc/zulu/";

   private final String robotModelResourceDirectory;

   private final Collection<String> xmlResources;

   public static final boolean SHORT_NUBS = false;

   private final Collection<String> urdfResources;

   private ZuluJointMap jointMap;
   private ZuluPhysicalProperties physicalProperties;
   private ZuluSensorInformation sensorInformation;
   private ZULUURDFParameters urdfParameters;

   ZuluVersion(String robotModelResourceDirectory, Collection<String> urdfResources, Collection<String> xmlResources)
   {
      this.robotModelResourceDirectory = robotModelResourceDirectory;
      this.urdfResources = urdfResources;
      this.xmlResources = xmlResources;
   }

   public String getRobotModelResourceDirectory()
   {
      return robotModelResourceDirectory;
   }

   public Collection<String> getURDFDescriptionResources()
   {
      return urdfResources;
   }

   public Collection<String> getXMLDescriptionResources()
   {
      return xmlResources;
   }

   public boolean hasArms(RobotSide side)
   {
      switch (this)
      {
         case V1_FULL_ROBOT:
            return true;
         default:
            return false;
      }
   }

   public boolean hasCycloidForearm()
   {
      switch (this)
      {
         case V1_FULL_ROBOT:
            return true;
         default:
            return false;
      }
   }

   public boolean hasCycloidForearm(RobotSide side)
   {
      switch (this)
      {
         case V1_FULL_ROBOT:
            return true;
         default:
            return false;
      }
   }

   public boolean armsNeedCalibration()
   {
      return false;
   }

   public ZuluJointMap getJointMap()
   {
      if (jointMap != null)
      {
         return jointMap;
      }
      switch (this)
      {
         case V1_FULL_ROBOT:
            jointMap = new ZuluJointMap(getPhysicalProperties(),
                                        new SideDependentList<>(ZuluArmConfiguration.FOREARM, ZuluArmConfiguration.FOREARM),
                                        true,
                                        true);
            break;
//         case V1_NUB_FOREARMS:
//            jointMap = new ZuluJointMap(getPhysicalProperties(),
//                                        new SideDependentList<>(ZuluArmConfiguration.NUB, ZuluArmConfiguration.NUB),
//                                        true,
//                                        true);
//            break;
//         case V1_LEGS_ROBOT:
//            jointMap = new ZuluJointMap(getPhysicalProperties(),
//                                        new SideDependentList<>(ZuluArmConfiguration.NONE, ZuluArmConfiguration.NONE),
//                                        false,
//                                        false);
      }
      return jointMap;
   }

   @Override
   public boolean hasArm(RobotSide robotSide)
   {
      return switch (this)
      {
         case V1_FULL_ROBOT -> true;
//         case V1_NUB_FOREARMS -> true;
         default -> false;
      };
   }

   public boolean hasHandWithFingers(RobotSide side)
   {
      return this == V1_FULL_ROBOT;
   }

   @Override
   public HandType getHandType(RobotSide side)
   {
      if (this == V1_FULL_ROBOT)
         return HandType.EZ_GRIPPER;

      return null;
   }

   @Override
   public boolean hasSakeGripperJoints(RobotSide side)
   {
      switch (this)
      {
         case V1_FULL_ROBOT ->
         {
            return true;
         }
         default ->
         {
            return false;
         }
      }
   }

   public boolean hasNubForearms(RobotSide side)
   {
      switch (this)
      {
         case V1_FULL_ROBOT:
            return true;
         default:
            return false;
      }
   }

   public ZuluSensorInformation getSensorInformation()
   {
      if (sensorInformation != null)
      {
         return sensorInformation;
      }

      switch (this)
      {
         case V1_FULL_ROBOT:
            sensorInformation = new ZuluSensorInformation(this);
            break;
         default:
            break;
      }
      // If this point is reached it means that sensorInformation is null
      return sensorInformation;
   }

   public ZuluPhysicalProperties getPhysicalProperties()
   {
      if (physicalProperties != null)
      {
         return physicalProperties;
      }

      switch (this)
      {
         case V1_FULL_ROBOT:
            physicalProperties = new ZuluPhysicalProperties();
            break;
         default:
            break;
      }
      return physicalProperties;
   }

   public ZULUURDFParameters getURDFParameters()
   {
      if (urdfParameters != null)
         return urdfParameters;
      urdfParameters = new ZULUURDFParameters(this);
      return urdfParameters;
   }
}
