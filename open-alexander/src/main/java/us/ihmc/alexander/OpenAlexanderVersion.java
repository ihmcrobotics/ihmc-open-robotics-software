package us.ihmc.alexander;

import us.ihmc.alexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.alexander.parameters.model.AlexanderPhysicalPropertiesV0;
import us.ihmc.alexander.parameters.model.OpenAlexanderURDFParameters;
import us.ihmc.alexander.parameters.model.HumanoidURDFParameterInterface;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Arrays;
import java.util.Collection;

public enum OpenAlexanderVersion implements AlexanderVersionInterface
{
   V0_FULL_ROBOT(Arrays.asList(OpenAlexanderURDFParameters.URDF_LOWER_BODY,
                               OpenAlexanderURDFParameters.URDF_LEFT_ARM,
                               OpenAlexanderURDFParameters.URDF_RIGHT_ARM), null),
   V0_NUB_FOREARMS(Arrays.asList(OpenAlexanderURDFParameters.URDF_LOWER_BODY,
                                 OpenAlexanderURDFParameters.URDF_LEFT_ARM_NUB_FOREARM,
                                 OpenAlexanderURDFParameters.URDF_RIGHT_ARM_NUB_FOREARM), null),
   V0_LEGS_ROBOT(Arrays.asList(OpenAlexanderURDFParameters.URDF_ONLY_LEGS), null);

   private static String[] resourceDirectories;
   private final SideDependentList<RigidBodyTransform> offsetHandFromAttachmentPlate = new SideDependentList<RigidBodyTransform>();
   private final Collection<String> hardwareMapResources;

   public static final boolean SHORT_NUBS = false;

   private final Collection<String> urdfModelPath;

   private AlexanderJointMap jointMap;
   private AlexanderPhysicalProperties physicalProperties;
   private AlexanderSensorInformation sensorInformation;
   private OpenAlexanderURDFParameters urdfParameters;

   OpenAlexanderVersion(Collection<String> urdfModelPath, Collection<String> hardwareMapResources)
   {
      this.urdfModelPath = urdfModelPath;
      this.hardwareMapResources = hardwareMapResources;
   }

   @Override
   public Collection<String> getModelPath()
   {
      return urdfModelPath;
   }

   @Override
   public Collection<String> getHardwareMapResources()
   {
      return hardwareMapResources;
   }

   public boolean hasArms(RobotSide side)
   {
      switch (this)
      {
         case V0_FULL_ROBOT, V0_NUB_FOREARMS:
            return true;
         default:
            return false;
      }
   }

   @Override
   public boolean hasCycloidForearms()
   {
      switch (this)
      {
         case V0_FULL_ROBOT:
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
   public AlexanderJointMap getJointMap()
   {
      if (jointMap != null)
      {
         return jointMap;
      }
      switch (this)
      {
         case V0_FULL_ROBOT:
            jointMap = new AlexanderJointMap(getPhysicalProperties(),
                                             new SideDependentList<>(AlexanderArmConfiguration.FOREARM, AlexanderArmConfiguration.FOREARM),
                                             true,
                                             true);
            break;
         case V0_NUB_FOREARMS:
            jointMap = new AlexanderJointMap(getPhysicalProperties(),
                                             new SideDependentList<>(AlexanderArmConfiguration.NUB, AlexanderArmConfiguration.NUB),
                                             true,
                                             true);
            break;
         case V0_LEGS_ROBOT:
            jointMap = new AlexanderJointMap(getPhysicalProperties(),
                                             new SideDependentList<>(AlexanderArmConfiguration.NONE, AlexanderArmConfiguration.NONE),
                                             false,
                                             false);
      }
      return jointMap;
   }

   @Override
   public boolean hasArm(RobotSide robotSide)
   {
      return switch (this)
      {
         case V0_FULL_ROBOT -> true;
         case V0_NUB_FOREARMS -> true;
         default -> false;
      };
   }

   @Override
   public boolean hasSakeGripperJoints(RobotSide side)
   {
      switch (this)
      {
         case V0_FULL_ROBOT ->
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
   public boolean hasNubHands(RobotSide side)
   {
      switch (this)
      {
         case V0_FULL_ROBOT, V0_NUB_FOREARMS:
            return true;
         default:
            return false;
      }
   }

   @Override
   public AlexanderSensorInformation getSensorInformation()
   {
      if (sensorInformation != null)
      {
         return sensorInformation;
      }

      switch (this)
      {
         case V0_FULL_ROBOT, V0_NUB_FOREARMS, V0_LEGS_ROBOT:
            sensorInformation = new AlexanderSensorInformation(this);
            break;
         default:
            break;
      }
      // If this point is reached it means that sensorInformation is null
      return sensorInformation;
   }

   @Override
   public AlexanderPhysicalProperties getPhysicalProperties()
   {
      if (physicalProperties != null)
      {
         return physicalProperties;
      }

      switch (this)
      {
         case V0_FULL_ROBOT, V0_NUB_FOREARMS:
            physicalProperties = new AlexanderPhysicalPropertiesV0();
            break;
         case V0_LEGS_ROBOT:
            physicalProperties = new AlexanderPhysicalPropertiesV0(false);
         default:
            break;
      }
      return physicalProperties;
   }

   @Override
   public HumanoidURDFParameterInterface getURDFParameters()
   {
      if (urdfParameters != null)
         return urdfParameters;
      urdfParameters = new OpenAlexanderURDFParameters(this);
      return urdfParameters;
   }
}
