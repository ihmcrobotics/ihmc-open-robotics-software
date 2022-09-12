package us.ihmc.avatar.networkProcessor.directionalControlToolboxModule;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.zip.CRC32;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

/**
 * Class to update a robot model based on RobotConfigurationData messages
 * 
 * @author Mark Paterson (adapted from JavaFXRobotVisualizer, without the JFX dependencies)
 */
public class RobotModelUpdater
{
   private final OneDoFJointBasics[] allJoints;
   private final int jointNameHash;
   private final AtomicReference<RigidBodyTransform> newRootJointPoseReference = new AtomicReference<>(null);
   private final AtomicReference<float[]> newJointConfigurationReference = new AtomicReference<>(null);

   private FullHumanoidRobotModel fullRobotModel;
   private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(4);
   private final int CONFIG_UPDATE_RATE_MS = 10; // How often to update the configuration state information in milliseconds 

   public RobotModelUpdater(FullHumanoidRobotModel robotModel)
   {
      fullRobotModel = robotModel;
      allJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);

      /* Used to ensure our model is compatible with the source model */
      jointNameHash = calculateJointNameHash(allJoints, fullRobotModel.getForceSensorDefinitions(), fullRobotModel.getIMUDefinitions());

      /* Task to update model joint poses and frames */
      final Runnable task = new Runnable()
      {
         @Override
         public void run()
         {
            /* Get new root joint pose (if one is available) and set to null, awaiting the next update */
            RigidBodyTransform newRootJointPose = newRootJointPoseReference.getAndSet(null);
            if (newRootJointPose != null)
            {
               fullRobotModel.getRootJoint().setJointConfiguration(newRootJointPose);
            }

            /* Get new joint configuration (if one is available) and set to null, awaiting the next update */
            float[] newJointConfiguration = newJointConfigurationReference.getAndSet(null);
            if (newJointConfiguration != null)
            {
               for (int i = 0; i < allJoints.length; i++)
                  allJoints[i].setQ(newJointConfiguration[i]);
            }

            /* Update frames, based on the new joint and position data */
            fullRobotModel.getElevator().updateFramesRecursively();
         }
      };

      /* run the update task */
      scheduler.scheduleAtFixedRate(task, 0, CONFIG_UPDATE_RATE_MS, TimeUnit.MILLISECONDS);
   }

   /**
    * Update the robot model based on the new configuration data. This function does not do the actual
    * updating, but rather sets up the information to be processed by the scheduled task defined in the
    * constructor. This function relies on being call with update, either directly, or as the callback
    * function to a RobotConfigurationData topic.
    * 
    * @param robotConfigurationData -- the incoming new configuration
    */
   public void updateConfiguration(RobotConfigurationData robotConfigurationData)
   {
      if (robotConfigurationData.getJointNameHash() != jointNameHash)
         throw new RuntimeException("Joint names do not match for RobotConfigurationData");

      newRootJointPoseReference.set(new RigidBodyTransform(robotConfigurationData.getRootOrientation(), robotConfigurationData.getRootPosition()));
      newJointConfigurationReference.set(robotConfigurationData.getJointAngles().toArray());
   }

   public static int calculateJointNameHash(OneDoFJointBasics[] joints, ForceSensorDefinition[] forceSensorDefinitions, IMUDefinition[] imuDefinitions)
   {
      CRC32 crc = new CRC32();
      for (OneDoFJointBasics joint : joints)
      {
         crc.update(joint.getName().getBytes());
      }

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         crc.update(forceSensorDefinition.getSensorName().getBytes());
      }

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         crc.update(imuDefinition.getName().getBytes());
      }

      return (int) crc.getValue();
   }

   public FullHumanoidRobotModel getRobot()
   {
      return fullRobotModel;
   }
}
