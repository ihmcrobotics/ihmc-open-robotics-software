package us.ihmc.sensorProcessing.communication.subscribers;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.RobotConfigurationData;
import controller_msgs.SpatialVectorMessage;
import us.ihmc.fastddsjava.cdr.idl.IDLFloatSequence;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.GraphicsUpdatable;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

public class RobotDataReceiver
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
      jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(allJoints,
                                                                           fullRobotModel.getForceSensorDefinitions(),
                                                                           fullRobotModel.getIMUDefinitions());

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

         IDLFloatSequence newJointAngles = robotConfigurationData.getJointAngles();
         for (int i = 0; i < newJointAngles.size(); i++)
         {
            allJoints[i].setQ(newJointAngles.get(i));
         }

         Point3D translation = robotConfigurationData.getRootPosition().getPoint();
         rootJoint.getJointPose().getPosition().set(translation.getX(), translation.getY(), translation.getZ());
         Quaternion orientation = robotConfigurationData.getRootOrientation().getQuaternion();
         rootJoint.getJointPose().getOrientation().setQuaternion(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
         rootJoint.getPredecessor().updateFramesRecursively();

         updateFrames();

         if (forceSensorDataHolder != null)
         {
            for (int i = 0; i < forceSensorDataHolder.getForceSensorDefinitions().size(); i++)
            {
               SpatialVectorMessage momentAndForceVectorForSensor = robotConfigurationData.getForceSensorData().get(i);
               forceSensorDataHolder.getData(forceSensorDataHolder.getForceSensorDefinitions().get(i))
                                    .setWrench(momentAndForceVectorForSensor.getAngularPart().getVector(), momentAndForceVectorForSensor.getLinearPart().getVector());
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