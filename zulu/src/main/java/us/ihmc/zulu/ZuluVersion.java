package us.ihmc.zulu;

import us.ihmc.handsros2.HandType;
import us.ihmc.zulu.parameters.model.ZuluPhysicalProperties;
import us.ihmc.zulu.parameters.model.ZuluPhysicalPropertiesV0;
import us.ihmc.zulu.parameters.model.ZULUURDFParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Arrays;
import java.util.Collection;

public enum ZuluVersion implements ZuluVersionInterface
{
   V1_FULL_ROBOT(ZuluVersion.V1_RESOURCE_DIRECTORY, Arrays.asList(ZULUURDFParameters.URDF_FULL_BODY), null);
//   V1_NUB_FOREARMS(ZuluVersion.V1_RESOURCE_DIRECTORY,
//                   Arrays.asList(ZULUURDFParameters.URDF_LOWER_BODY,
//                                 ZULUURDFParameters.URDF_LEFT_ARM_NUB_FOREARM,
//                                 ZULUURDFParameters.URDF_HEAD,
//                                 ZULUURDFParameters.URDF_RIGHT_ARM_NUB_FOREARM),
//                   null),
//   V1_LEGS_ROBOT(ZuluVersion.V1_RESOURCE_DIRECTORY, Arrays.asList(ZULUURDFParameters.URDF_LOWER_BODY_ONLY), null);

   private static final String V1_RESOURCE_DIRECTORY = "us/ihmc/zulu/";

   private final String robotModelResourceDirectory;

   private final SideDependentList<RigidBodyTransform> offsetHandFromAttachmentPlate = new SideDependentList<RigidBodyTransform>();
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

   @Override
   public String getRobotModelResourceDirectory()
   {
      return robotModelResourceDirectory;
   }

   @Override
   public Collection<String> getURDFDescriptionResources()
   {
      return urdfResources;
   }

   @Override
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

   @Override
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

   @Override
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

   @Override
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

   @Override
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

   @Override
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

   @Override
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

   @Override
   public ZuluPhysicalProperties getPhysicalProperties()
   {
      if (physicalProperties != null)
      {
         return physicalProperties;
      }

      switch (this)
      {
         case V1_FULL_ROBOT:
            physicalProperties = new ZuluPhysicalPropertiesV0();
            break;
         default:
            break;
      }
      return physicalProperties;
   }

   @Override
   public ZULUURDFParameters getURDFParameters()
   {
      if (urdfParameters != null)
         return urdfParameters;
      urdfParameters = new ZULUURDFParameters(this);
      return urdfParameters;
   }
}
