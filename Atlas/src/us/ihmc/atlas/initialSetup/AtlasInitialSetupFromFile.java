package us.ihmc.atlas.initialSetup;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasInitialSetupFromFile implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
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
   public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      if (robotInitialized)
         return;

      PrintTools.info("Loading initial joint configuration for driving simulation from " + initialConditionsFileName);

      try
      {
         Properties properties = new Properties();
         InputStream stream = getClass().getClassLoader().getResourceAsStream(initialConditionsFileName);
         properties.load(stream);

         for (RobotSide robotSide : RobotSide.values())
         {
            for (LegJointName jointName : LegJointName.values)
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

         if (properties.containsKey(POSITION_KEY))
         {
            String position[] = properties.getProperty(POSITION_KEY).split(" ");
            pelvisPoseInWorld.setTranslation(new Vector3D(Double.parseDouble(position[0]), Double.parseDouble(position[1]), Double.parseDouble(position[2])));
         }

         if (properties.containsKey(ORIENTATION_KEY))
         {
            String quat[] = properties.getProperty(ORIENTATION_KEY).split(" ");
            pelvisPoseInWorld.setRotation(new Quaternion(Double.parseDouble(quat[0]), Double.parseDouble(quat[1]), Double.parseDouble(quat[2]), 1.0));
         }

         stream.close();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Atlas joint parameter file  cannot be loaded. ", e);
      }
      catch (NumberFormatException e)
      {
         throw new RuntimeException("Make sure all fields are doubles in File", e);
      }

      robot.getRootJoint().setRotationAndTranslation(pelvisPoseInWorld);
      robot.update();

      robotInitialized = true;
   }

   private void setRobotAngle(String jointName, Properties properties, FloatingRootJointRobot robot)
   {
      if (jointName == null)
         return;

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
   public void getOffset(Vector3D offsetToPack)
   {
      pelvisPoseInWorld.getTranslation(offsetToPack);
   }

   @Override
   public void setOffset(Vector3D offset)
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
