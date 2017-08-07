package us.ihmc.atlas.initialSetup;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

import org.junit.Test;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasDrivingInitialSetupTest
{
   private static final AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
   
	@ContinuousIntegrationTest(estimatedDuration = 1.2)
   @Test(timeout = 30000)
   public void testLoadFile()
   {
	   AtlasPhysicalProperties atlasPhysicalProperties = new AtlasPhysicalProperties(1);
      AtlasRobotModel robotModel = new AtlasRobotModel(version, DRCRobotModel.RobotTarget.SCS, false);
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      DRCRobotJointMap jointMap = new AtlasJointMap(version, atlasPhysicalProperties);
      
      AtlasInitialSetupFromFile initialSetup = new AtlasInitialSetupFromFile("initialDrivingSetup");
      initialSetup.initializeRobot(robot, jointMap);
   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFileContainsAllJoints()
   {
      boolean containsAllJoints = true;
      AtlasPhysicalProperties atlasPhysicalProperties = new AtlasPhysicalProperties(1);
      DRCRobotJointMap jointMap = new AtlasJointMap(version, atlasPhysicalProperties);
      try
      {
         Properties properties = new Properties();
         InputStream stream = getClass().getClassLoader().getResourceAsStream("initialDrivingSetup");
         properties.load(stream);
         
         for (RobotSide robotSide : RobotSide.values())
         {
            for (LegJointName jointName : LegJointName.values)
            {
               String key = jointMap.getLegJointName(robotSide, jointName);
               if (key == null)
               {
                  continue;
               }
               if (!properties.containsKey(key))
               {
                  System.out.println("File did not contain joint " + key);
                  containsAllJoints = false;
               }
            }
            
            for (ArmJointName jointName : ArmJointName.values())
            {
               String key = jointMap.getArmJointName(robotSide, jointName);
               if (key == null)
               {
                  continue;
               }
               if (!properties.containsKey(key))
               {
                  System.out.println("File did not contain joint " + key);
                  containsAllJoints = false;
               }
            }
         }
         
         for (SpineJointName jointName : SpineJointName.values)
         {
            String key = jointMap.getSpineJointName(jointName);
            if (key == null)
            {
               continue;
            }
            if (!properties.containsKey(key))
            {
               System.out.println("File did not contain joint " + key);
               containsAllJoints = false;
            }
         }
         
         stream.close();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Atlas joint parameter file cannot be loaded. ", e);
      }
      
      assertTrue(containsAllJoints);
   }
}
