package us.ihmc.atlas.initialSetup;

import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

import org.junit.Test;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasDrivingInitialSetupTest
{
   private static final AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
   
   @EstimatedDuration(duration = 1.0)
   @Test(timeout = 3000)
   public void testLoadFile()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(version, DRCRobotModel.RobotTarget.SCS, false);
      SDFHumanoidRobot robot = robotModel.createSdfRobot(false);
      DRCRobotJointMap jointMap = new AtlasJointMap(version);
      
      AtlasInitialSetupFromFile initialSetup = new AtlasInitialSetupFromFile("initialDrivingSetup");
      initialSetup.initializeRobot(robot, jointMap);
   }
   
   @EstimatedDuration(duration = 1.0)
   @Test(timeout = 3000)
   public void testFileContainsAllJoints()
   {
      boolean containsAllJoints = true;
      
      DRCRobotJointMap jointMap = new AtlasJointMap(version);
      File file = new File("initialDrivingSetup");
      try
      {
         Properties properties = new Properties();
         FileInputStream stream = new FileInputStream(file);
         properties.load(stream);
         
         for (RobotSide robotSide : RobotSide.values())
         {
            for (LegJointName jointName : LegJointName.values())
            {
               String key = jointMap.getLegJointName(robotSide, jointName);
               if (key == null)
               {
                  continue;
               }
               if (!properties.containsKey(key))
               {
                  System.out.println(file.getAbsolutePath() + " did not contain joint " + key);
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
                  System.out.println(file.getAbsolutePath() + " did not contain joint " + key);
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
               System.out.println(file.getAbsolutePath() + " did not contain joint " + key);
               containsAllJoints = false;
            }
         }
         
         stream.close();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Atlas joint parameter file " + file.getAbsolutePath() + " cannot be loaded. ", e);
      }
      
      assertTrue(containsAllJoints);
   }
}
