package us.ihmc.sensorProcessing.communication.subscribers;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.tools.gui.GraphicsUpdatable;

public class RobotDataReceiver implements PacketConsumer<RobotConfigurationData>
{

   private final AtomicReference<RobotConfigurationData> packet = new AtomicReference<RobotConfigurationData>(null);
   private final Object lock = new Object();
   private final ArrayList<GraphicsUpdatable> graphicsToUpdate = new ArrayList<GraphicsUpdatable>();
   private final AtomicLong simTime = new AtomicLong(-1);
   protected final FloatingInverseDynamicsJoint rootJoint;
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

         Vector3D32 translation = robotConfigurationData.getPelvisTranslation();
         rootJoint.setPosition(translation.getX(), translation.getY(), translation.getZ());
         Quaternion32 orientation = robotConfigurationData.getPelvisOrientation();
         rootJoint.setRotation(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
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