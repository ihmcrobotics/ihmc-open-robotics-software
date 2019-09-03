package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import gnu.trove.set.TLongSet;
import gnu.trove.set.hash.TLongHashSet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.sensorProcessing.outputData.LowLevelState;

public class SensorDataContext implements Settable<SensorDataContext>
{
   private final List<String> jointNames = new ArrayList<>();
   private final List<LowLevelState> jointMeasurements = new ArrayList<>();
   private final transient TLongSet jointSet = new TLongHashSet();
   private final transient Map<String, LowLevelState> jointMeasurementMap = new HashMap<>();

   private final List<String> imuNames = new ArrayList<>();
   private final List<ImuData> imuMeasurements = new ArrayList<>();
   private final transient TLongSet imuSet = new TLongHashSet();
   private final transient Map<String, ImuData> imuMeasurementMap = new HashMap<>();

   private final List<String> forceSensorNames = new ArrayList<>();
   private final List<DenseMatrix64F> forceSensorMeasurements = new ArrayList<>();
   private final transient TLongSet forceSensorSet = new TLongHashSet();
   private final transient Map<String, DenseMatrix64F> forceSensorMeasurementMap = new HashMap<>();

   public SensorDataContext()
   {
   }

   public SensorDataContext(FullHumanoidRobotModel fullRobotModel)
   {
      Arrays.asList(fullRobotModel.getOneDoFJoints()).forEach(joint -> registerJoint(joint.getName()));
      Arrays.asList(fullRobotModel.getIMUDefinitions()).forEach(imu -> registerImu(imu.getName()));
      Arrays.asList(fullRobotModel.getForceSensorDefinitions()).forEach(forceSensor -> registerForceSensor(forceSensor.getSensorName()));
   }

   public LowLevelState registerJoint(String jointName)
   {
      // Check if a joint with the same name is already registered.
      // It is very unlikely but it could happen that two joint names produce the same hash code.
      if (isJointRegistered(jointName))
      {
         throw new RuntimeException("Joint with same name hash as " + jointName + " is already registered.");
      }

      // If the joint was ever registered in the past reuse the existing data structure.
      LowLevelState lowLevelState = jointMeasurementMap.get(jointName);
      if (lowLevelState == null)
      {
         lowLevelState = new LowLevelState();
         jointMeasurementMap.put(jointName, lowLevelState);
      }

      jointNames.add(jointName);
      jointMeasurements.add(lowLevelState);
      jointSet.add(jointName.hashCode());

      return lowLevelState;
   }

   public ImuData registerImu(String imuName)
   {
      // Check if an IMU with the same name is already registered.
      // It is very unlikely but it could happen that two IMU names produce the same hash code.
      if (isImuRegistered(imuName))
      {
         throw new RuntimeException("IMU with same name hash as " + imuName + " is already registered.");
      }

      // If the IMU was ever registered in the past reuse the existing data structure.
      ImuData imuData = imuMeasurementMap.get(imuName);
      if (imuData == null)
      {
         imuData = new ImuData();
         imuMeasurementMap.put(imuName, imuData);
      }

      imuNames.add(imuName);
      imuMeasurements.add(imuData);
      imuSet.add(imuName.hashCode());

      return imuData;
   }

   public DenseMatrix64F registerForceSensor(String forceSensorName)
   {
      // Check if an force sensor with the same name is already registered.
      // It is very unlikely but it could happen that two force sensor names produce the same hash code.
      if (isForceSensorRegistered(forceSensorName))
      {
         throw new RuntimeException("Force Sensor with same name hash as " + forceSensorName + " is already registered.");
      }

      // If the force sensor was ever registered in the past reuse the existing data structure.
      DenseMatrix64F forceSensorData = forceSensorMeasurementMap.get(forceSensorName);
      if (forceSensorData == null)
      {
         forceSensorData = new DenseMatrix64F(6, 1);
         forceSensorMeasurementMap.put(forceSensorName, forceSensorData);
      }

      forceSensorNames.add(forceSensorName);
      forceSensorMeasurements.add(forceSensorData);
      forceSensorSet.add(forceSensorName.hashCode());

      return forceSensorData;
   }

   public LowLevelState getMeasuredJointState(String jointName)
   {
      if (!isJointRegistered(jointName))
      {
         throw new RuntimeException("Joint " + jointName + " was not registered in this data holder.");
      }
      return jointMeasurementMap.get(jointName);
   }

   public ImuData getImuMeasurement(String imuName)
   {
      if (!isImuRegistered(imuName))
      {
         throw new RuntimeException("IMU " + imuName + " was not registered in this data holder.");
      }
      return imuMeasurementMap.get(imuName);
   }

   public DenseMatrix64F getForceSensorMeasurement(String forceSensorName)
   {
      if (!isForceSensorRegistered(forceSensorName))
      {
         throw new RuntimeException("Force Sensor " + forceSensorName + " was not registered in this data holder.");
      }
      return forceSensorMeasurementMap.get(forceSensorName);
   }

   public boolean isJointRegistered(String jointName)
   {
      return jointSet.contains(jointName.hashCode());
   }

   public boolean isImuRegistered(String imuName)
   {
      return imuSet.contains(imuName.hashCode());
   }

   public boolean isForceSensorRegistered(String forceSensorName)
   {
      return forceSensorSet.contains(forceSensorName.hashCode());
   }

   public void clear()
   {
      jointNames.clear();
      jointMeasurements.clear();
      jointSet.clear();

      imuNames.clear();
      imuMeasurements.clear();
      imuSet.clear();

      forceSensorNames.clear();
      forceSensorMeasurements.clear();
      forceSensorSet.clear();
   }

   @Override
   public void set(SensorDataContext other)
   {
      clear();
      for (int i = 0; i < other.jointNames.size(); i++)
      {
         registerJoint(other.jointNames.get(i)).set(other.jointMeasurements.get(i));
      }
      for (int i = 0; i < other.imuNames.size(); i++)
      {
         registerImu(other.imuNames.get(i)).set(other.imuMeasurements.get(i));
      }
      for (int i = 0; i < other.forceSensorNames.size(); i++)
      {
         registerForceSensor(other.forceSensorNames.get(i)).set(other.forceSensorMeasurements.get(i));
      }
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof SensorDataContext)
      {
         SensorDataContext other = (SensorDataContext) obj;

         if (jointNames.size() != other.jointNames.size())
            return false;
         for (int i = 0; i < jointNames.size(); i++)
         {
            String jointName = jointNames.get(i);
            if (!other.isJointRegistered(jointName))
               return false;
            if (!jointMeasurements.get(i).equals(other.getMeasuredJointState(jointName)))
               return false;
         }

         if (imuNames.size() != other.imuNames.size())
            return false;
         for (int i = 0; i < imuNames.size(); i++)
         {
            String imuName = imuNames.get(i);
            if (!other.isImuRegistered(imuName))
               return false;
            if (!imuMeasurements.get(i).equals(other.getImuMeasurement(imuName)))
               return false;
         }

         if (forceSensorNames.size() != other.forceSensorNames.size())
            return false;
         for (int i = 0; i < forceSensorNames.size(); i++)
         {
            String forceSensorName = forceSensorNames.get(i);
            if (!other.isForceSensorRegistered(forceSensorName))
               return false;
            if (!MatrixTools.equals(forceSensorMeasurements.get(i), other.getForceSensorMeasurement(forceSensorName)))
               return false;
         }

         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      String ret = "";
      for (int i = 0; i < jointNames.size(); i++)
         ret += "Joint " + jointNames.get(i) + ": " + jointMeasurements.get(i) + "\n";
      for (int i = 0; i < imuNames.size(); i++)
         ret += "IMU " + imuNames.get(i) + ": " + imuMeasurements.get(i) + "\n";
      for (int i = 0; i < forceSensorNames.size(); i++)
         ret += "Force Sensor " + forceSensorNames.get(i) + ": " + forceSensorMeasurements.get(i) + "\n";
      return ret;
   }
}
