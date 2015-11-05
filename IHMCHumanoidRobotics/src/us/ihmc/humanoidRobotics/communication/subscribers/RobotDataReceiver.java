package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.tools.gui.GraphicsUpdatable;

public class RobotDataReceiver
{

   private final AtomicReference<RobotConfigurationData> packet = new AtomicReference<RobotConfigurationData>(null);
   private final Object lock = new Object();
   private final ArrayList<GraphicsUpdatable> graphicsToUpdate = new ArrayList<GraphicsUpdatable>();
   private final AtomicLong simTime = new AtomicLong(-1);
   protected final SixDoFJoint rootJoint;
   private boolean frameshaveBeenSetUp = false;
   private final ForceSensorDataHolder forceSensorDataHolder;
   private final OneDoFJoint[] allJoints;
   private final int jointNameHash;

   public RobotDataReceiver(FullRobotModel fullRobotModel, ForceSensorDataHolder forceSensorDataHolder)
   {
      this(fullRobotModel, fullRobotModel.getOneDoFJoints(), forceSensorDataHolder);
   }
   
   protected RobotDataReceiver(FullRobotModel fullRobotModel, OneDoFJoint[] allJoints, ForceSensorDataHolder forceSensorDataHolder)
   {
      this.allJoints = allJoints; 
      jointNameHash = RobotConfigurationData.calculateJointNameHash(allJoints, fullRobotModel.getForceSensorDefinitions(), fullRobotModel.getIMUDefinitions());

      rootJoint = fullRobotModel.getRootJoint();
      this.forceSensorDataHolder = forceSensorDataHolder;
   }



   public ForceSensorDataHolder getForceSensorDataHolder()
   {
      return forceSensorDataHolder;
   }

   public void addGraphicsUpdateable(GraphicsUpdatable updateable)
   {
      graphicsToUpdate.add(updateable);
   }

   public void receivedPacket(RobotConfigurationData object)
   {
      packet.set(object);
      simTime.set(object.getTimestamp());
   }

   public void updateRobotModel()
   {
      RobotConfigurationData robotConfigurationData = packet.getAndSet(null);

      if (robotConfigurationData == null)
         return;

      synchronized (lock)
      {
         if (robotConfigurationData.jointNameHash != jointNameHash)
         {
            throw new RuntimeException("Joint names do not match for RobotConfigurationData");
         }

         float[] newJointAngles = robotConfigurationData.getJointAngles();
         for (int i = 0; i < newJointAngles.length; i++)
         {
            allJoints[i].setQ(newJointAngles[i]);
         }

         Vector3f translation = robotConfigurationData.getPelvisTranslation();
         rootJoint.setPosition(translation.x, translation.y, translation.z);
         Quat4f orientation = robotConfigurationData.getPelvisOrientation();
         rootJoint.setRotation(orientation.x, orientation.y, orientation.z, orientation.w);
         rootJoint.getPredecessor().updateFramesRecursively();
         
         updateFrames();
         

         if (forceSensorDataHolder != null)
         {
            for (int i = 0; i < forceSensorDataHolder.getForceSensorDefinitions().size(); i++)
            {
               forceSensorDataHolder.get(forceSensorDataHolder.getForceSensorDefinitions().get(i))
                     .setWrench(robotConfigurationData.getMomentAndForceVectorForSensor(i));
            }
         }
         for (GraphicsUpdatable graphicsUpdatable : graphicsToUpdate)
         {
            if (graphicsUpdatable != null)
               graphicsUpdatable.update();
         }

         frameshaveBeenSetUp = true;

      }
   }

   protected void updateFrames()
   {
      
   }



   public long getSimTimestamp()
   {
      return simTime.getAndSet(-1);
   }

   public boolean framesHaveBeenSetUp()
   {
      return frameshaveBeenSetUp;
   }

}