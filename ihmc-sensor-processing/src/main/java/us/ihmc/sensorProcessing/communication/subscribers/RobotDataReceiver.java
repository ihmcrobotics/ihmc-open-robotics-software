package us.ihmc.sensorProcessing.communication.subscribers;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.SpatialVectorMessage;
import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.GraphicsUpdatable;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

public class RobotDataReceiver implements PacketConsumer<RobotConfigurationData>
{

   private final AtomicReference<RobotConfigurationData> packet = new AtomicReference<RobotConfigurationData>(null);
   private final Object lock = new Object();
   private final ArrayList<GraphicsUpdatable> graphicsToUpdate = new ArrayList<GraphicsUpdatable>();
   private final AtomicLong simTime = new AtomicLong(-1);
   protected final FloatingJointBasics rootJoint;
   private boolean frameshaveBeenSetUp = false;
   private final ForceSensorDataHolder forceSensorDataHolder;
   private final OneDoFJointBasics[] allJoints;
   private final int jointNameHash;

   public RobotDataReceiver(FullRobotModel fullRobotModel, ForceSensorDataHolder forceSensorDataHolder)
   {
      this(fullRobotModel, fullRobotModel.getOneDoFJoints(), forceSensorDataHolder);
   }
   
   protected RobotDataReceiver(FullRobotModel fullRobotModel, OneDoFJointBasics[] allJoints, ForceSensorDataHolder forceSensorDataHolder)
   {
      this.allJoints = allJoints; 
      jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(allJoints, fullRobotModel.getForceSensorDefinitions(), fullRobotModel.getIMUDefinitions());

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
      simTime.set(object.getMonotonicTime());
   }

   public void updateRobotModel()
   {
      RobotConfigurationData robotConfigurationData = packet.getAndSet(null);

      if (robotConfigurationData == null)
         return;

      synchronized (lock)
      {
         if (robotConfigurationData.getJointNameHash() != jointNameHash)
         {
            throw new RuntimeException("Joint names do not match for RobotConfigurationData");
         }

         TFloatArrayList newJointAngles = robotConfigurationData.getJointAngles();
         for (int i = 0; i < newJointAngles.size(); i++)
         {
            allJoints[i].setQ(newJointAngles.get(i));
         }

         Vector3D translation = robotConfigurationData.getRootTranslation();
         rootJoint.getJointPose().setPosition(translation.getX(), translation.getY(), translation.getZ());
         Quaternion orientation = robotConfigurationData.getRootOrientation();
         rootJoint.getJointPose().getOrientation().setQuaternion(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
         rootJoint.getPredecessor().updateFramesRecursively();
         
         updateFrames();
         

         if (forceSensorDataHolder != null)
         {
            for (int i = 0; i < forceSensorDataHolder.getForceSensorDefinitions().size(); i++)
            {
               SpatialVectorMessage momentAndForceVectorForSensor = robotConfigurationData.getForceSensorData().get(i);
               forceSensorDataHolder.get(forceSensorDataHolder.getForceSensorDefinitions().get(i)).setWrench(momentAndForceVectorForSensor.getAngularPart(),
                                                                                                             momentAndForceVectorForSensor.getLinearPart());
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