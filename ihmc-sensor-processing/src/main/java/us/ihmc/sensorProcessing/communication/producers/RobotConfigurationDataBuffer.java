package us.ihmc.sensorProcessing.communication.producers;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.SpatialVectorMessage;
import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;

/**
 * Buffer for RobotConfigurationData. Allows updating a fullrobotmodel based on timestamps. Make
 * sure not to share fullrobotmodels between thread
 *
 * @author jesper
 *
 */
public class RobotConfigurationDataBuffer implements PacketConsumer<RobotConfigurationData>
{
   private static final boolean DEBUG = false;
   final static int BUFFER_SIZE = 1000;

   private final RobotConfigurationData[] configurationBuffer = new RobotConfigurationData[BUFFER_SIZE];

   private final AtomicInteger currentIndex = new AtomicInteger();
   private final ReentrantLock updateLock = new ReentrantLock();
   private final Condition timestampCondition = updateLock.newCondition();

   private final ThreadLocal<HashMap<FullRobotModel, FullRobotModelCache>> fullRobotModelsCache = new ThreadLocal<HashMap<FullRobotModel, FullRobotModelCache>>()
   {
      @Override
      protected HashMap<FullRobotModel, FullRobotModelCache> initialValue()
      {
         return new HashMap<>();
      }
   };

   public void update(RobotConfigurationData data)
   {
      updateLock.lock();

      int index = currentIndex.get();
      index++;
      if (index >= BUFFER_SIZE)
      {
         index = 0;
      }
      configurationBuffer[index] = data;

      currentIndex.set(index);

      timestampCondition.signalAll();
      updateLock.unlock();
   }

   void waitForTimestamp(long timestamp)
   {
      updateLock.lock();
      long currentTimestamp;
      while ((currentTimestamp = getNewestTimestamp()) < timestamp)
      {
         if (DEBUG)
         {
            System.out.println("Current timestamp: " + currentTimestamp + ", waiting for " + timestamp);
         }

         try
         {
            timestampCondition.await();
         }
         catch (InterruptedException e)
         {
         }
      }
      updateLock.unlock();
   }

   public long getNewestTimestamp()
   {
      RobotConfigurationData newestConfigurationData = configurationBuffer[currentIndex.get()];
      if (newestConfigurationData == null)
      {
         return -1;
      }
      else
      {
         return newestConfigurationData.getMonotonicTime();
      }
   }

   private RobotConfigurationData floorIndex(long key)
   {
      int currentIndex = this.currentIndex.get();
      for (int i = currentIndex; i >= -BUFFER_SIZE + currentIndex; i--)
      {
         int index = (i < 0) ? BUFFER_SIZE + i : i;

         RobotConfigurationData data = configurationBuffer[index];
         if (data == null)
         {
            break;
         }

         if (data.getMonotonicTime() <= key)
         {
            return data;
         }
      }
      return null;
   }

   /**
    * Update a full robot model with data from timestamp. Optionally update force sensors
    *
    * @param waitForTimestamp Will block if no timestamp is not received yet
    * @param timestamp Timestamp to get. Will return the data for the last received that is smaller
    *           or equal to timestamp.
    * @param model Model to update. Will call updateFramesRecursively()
    * @param forceSensorDataHolder. Optional, update force sensor data holders
    *
    * @return true if model is updated
    */
   public long updateFullRobotModel(boolean waitForTimestamp, long timestamp, FullRobotModel model, ForceSensorDataHolder forceSensorDataHolder)
   {
      if (waitForTimestamp)
      {
         waitForTimestamp(timestamp);
      }

      RobotConfigurationData robotConfigurationData = floorIndex(timestamp);
      if (robotConfigurationData == null)
      {
         return -1;
      }
      updateFullRobotModel(robotConfigurationData, model, forceSensorDataHolder);
      return robotConfigurationData.getMonotonicTime();
   }

   public boolean updateFullRobotModelWithNewestData(FullRobotModel fullRobotModel, ForceSensorDataHolder forceSensorDataHolder)
   {
      int currentIndex = this.currentIndex.get() - 1;
      if (currentIndex < 0)
      {
         currentIndex = 0;

      }
      RobotConfigurationData robotConfigurationData = configurationBuffer[currentIndex];

      if (robotConfigurationData == null)
      {
         return false;
      }

      updateFullRobotModel(robotConfigurationData, fullRobotModel, forceSensorDataHolder);
      return true;
   }

   private void updateFullRobotModel(RobotConfigurationData robotConfigurationData, FullRobotModel model, ForceSensorDataHolder forceSensorDataHolder)
   {
      FullRobotModelCache fullRobotModelCache = getFullRobotModelCache(model);

      FloatingJointBasics rootJoint = model.getRootJoint();
      if (robotConfigurationData.getJointNameHash() != fullRobotModelCache.jointNameHash)
      {
         System.out.println(robotConfigurationData.getJointNameHash());
         System.out.println(fullRobotModelCache.jointNameHash);
         throw new RuntimeException("Joint names do not match for RobotConfigurationData");
      }

      TFloatArrayList newJointAngles = robotConfigurationData.getJointAngles();
      TFloatArrayList newJointVelocities = robotConfigurationData.getJointVelocities();

      for (int i = 0; i < newJointAngles.size(); i++)
      {
         fullRobotModelCache.allJoints[i].setQ(newJointAngles.get(i));
         fullRobotModelCache.allJoints[i].setQd(newJointVelocities.get(i));
      }

      Vector3D translation = robotConfigurationData.getRootTranslation();
      rootJoint.getJointPose().setPosition(translation.getX(), translation.getY(), translation.getZ());
      Quaternion orientation = robotConfigurationData.getRootOrientation();
      rootJoint.getJointPose().getOrientation().setQuaternion(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());

      Twist rootJointTwist = new Twist();
      rootJointTwist.setIncludingFrame(rootJoint.getJointTwist());
      Vector3D pelvisAngularVelocity = robotConfigurationData.getPelvisAngularVelocity();
      Vector3D pelvisLinearVelocity = robotConfigurationData.getPelvisLinearVelocity();
      rootJointTwist.getAngularPart().set(pelvisAngularVelocity);
      rootJointTwist.getLinearPart().set(pelvisLinearVelocity);
      rootJoint.setJointTwist(rootJointTwist);

      rootJoint.getPredecessor().updateFramesRecursively();

      if (forceSensorDataHolder != null)
      {
         for (int i = 0; i < forceSensorDataHolder.getForceSensorDefinitions().size(); i++)
         {
            SpatialVectorMessage momentAndForceVectorForSensor = robotConfigurationData.getForceSensorData().get(i);
            forceSensorDataHolder.get(forceSensorDataHolder.getForceSensorDefinitions().get(i)).setWrench(momentAndForceVectorForSensor.getAngularPart(),
                                                                                                          momentAndForceVectorForSensor.getLinearPart());
         }
      }
   }

   private FullRobotModelCache getFullRobotModelCache(FullRobotModel fullRobotModel)
   {
      HashMap<FullRobotModel, FullRobotModelCache> cache = fullRobotModelsCache.get();
      FullRobotModelCache fullRobotModelCache = cache.get(fullRobotModel);
      if (fullRobotModelCache == null)
      {
         fullRobotModelCache = new FullRobotModelCache(fullRobotModel);
         cache.put(fullRobotModel, fullRobotModelCache);
      }

      return fullRobotModelCache;

   }

   private static class FullRobotModelCache
   {
      private final OneDoFJointBasics[] allJoints;
      private final long jointNameHash;

      private FullRobotModelCache(FullRobotModel fullRobotModel)
      {
         if (fullRobotModel instanceof FullHumanoidRobotModel)
            allJoints = FullRobotModelUtils.getAllJointsExcludingHands((FullHumanoidRobotModel) fullRobotModel);
         else
            allJoints = fullRobotModel.getOneDoFJoints();
         jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(allJoints, fullRobotModel.getForceSensorDefinitions(),
                                                                       fullRobotModel.getIMUDefinitions());
      }
   }

   @Override
   public void receivedPacket(RobotConfigurationData packet)
   {
      update(packet);
   }

}
