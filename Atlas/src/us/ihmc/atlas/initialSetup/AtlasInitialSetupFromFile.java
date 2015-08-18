package us.ihmc.atlas.initialSetup;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFBaseRobot;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.humanoidRobotics.partNames.SpineJointName;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasInitialSetupFromFile implements DRCRobotInitialSetup<SDFHumanoidRobot>
{
   private String initialConditionsFileName;
   
   private static final String POSITION_KEY = "pelvisPos", ORIENTATION_KEY = "pelvisRot";
   
   private final RigidBodyTransform pelvisPoseInWorld = new RigidBodyTransform();
   private boolean robotInitialized = false;

   public AtlasInitialSetupFromFile(String initialConditionsFile)
   {
      this.initialConditionsFileName = initialConditionsFile;
   }

   @Override
   public void initializeRobot(SDFHumanoidRobot robot, DRCRobotJointMap jointMap)
   {
      if (robotInitialized) 
         return;
      
      File file = new File(initialConditionsFileName);
      PrintTools.info("Loading initial joint configuration for driving simulation from " + file.getAbsolutePath());
      
      if (file.exists() && file.isFile())
      {
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
                  setRobotAngle(key, properties, robot);
               }
               
               for (ArmJointName jointName : ArmJointName.values())
               {
                  String key = jointMap.getArmJointName(robotSide, jointName);
                  setRobotAngle(key, properties, robot);
               }
           }
            
            for (SpineJointName jointName : SpineJointName.values)
            {
               String key = jointMap.getSpineJointName(jointName);
               setRobotAngle(key, properties, robot);
            }
            
            if(properties.containsKey(POSITION_KEY))
            {
               String position[] = properties.getProperty(POSITION_KEY).split(" ");
               pelvisPoseInWorld.setTranslation(new Vector3d(
                     Double.parseDouble(position[0]), 
                     Double.parseDouble(position[1]), 
                     Double.parseDouble(position[2])));
            }
            
            if(properties.containsKey(ORIENTATION_KEY))
            {
               String quat[] = properties.getProperty(ORIENTATION_KEY).split(" ");
               pelvisPoseInWorld.setRotation(new Quat4d(
                     Double.parseDouble(quat[0]), 
                     Double.parseDouble(quat[1]), 
                     Double.parseDouble(quat[2]),
                     1.0));
            }
            
            stream.close();
         }
         catch (IOException e)
         {
            throw new RuntimeException("Atlas joint parameter file " + file.getAbsolutePath() + " cannot be loaded. ", e);
         }
         catch (NumberFormatException e)
         {
            throw new RuntimeException("Make sure all fields are doubles in " + file.getAbsolutePath(), e);
         }
      }
      else
      {
         PrintTools.info("File not found or invalid.");
      }
      
      robot.getRootJoint().setRotationAndTranslation(pelvisPoseInWorld);
      robot.update();
      
      robotInitialized = true;
   }
   
   private void setRobotAngle(String jointName, Properties properties, SDFBaseRobot robot)
   {
      if(jointName == null) return;
      
      if (properties.containsKey(jointName))
      {
         String jointAngle = properties.getProperty(jointName);
         robot.getOneDegreeOfFreedomJoint(jointName).setQ(Double.parseDouble(jointAngle) / 100.0);
      }
      else
      {
         PrintTools.info("Did not find initial angle for " + jointName);
      }
   }

   @Override
   public void getOffset(Vector3d offsetToPack)
   {
      pelvisPoseInWorld.getTranslation(offsetToPack);
   }

   @Override
   public void setOffset(Vector3d offset)
   {
      pelvisPoseInWorld.setTranslation(offset);
   }

   @Override
   public void setInitialYaw(double yaw)
   {
      PrintTools.info("not implemented");
   }

   @Override
   public void setInitialGroundHeight(double groundHeight)
   {
      PrintTools.info("not implemented");
   }

   @Override
   public double getInitialYaw()
   {
      PrintTools.info("not implemented");
      return 0.0;
   }

   @Override
   public double getInitialGroundHeight()
   {
      PrintTools.info("not implemented");
      return 0.0;
   }
   
   public String getFileName()
   {
      return initialConditionsFileName;
   }
}
