package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;

import gnu.trove.set.TLongSet;
import gnu.trove.set.hash.TLongHashSet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.sensorProcessing.outputData.LowLevelState;

/**
 * A class for managing sensor data from joints, IMUs, and force sensors in a robotic system.
 * It provides methods to register sensors, retrieve their measurements, and manage the data context.
 * Implements {@link Settable} to allow copying data from another {@code SensorDataContext}.
 */
public class SensorDataContext implements Settable<SensorDataContext>
{
   /** List of registered joint names. */
   private final List<String> jointNames = new ArrayList<>();
   /** List of joint measurements. */
   private final List<LowLevelState> jointMeasurements = new ArrayList<>();
   /** Set of joint name hash codes for quick lookup. */
   private final transient TLongSet jointSet = new TLongHashSet();
   /** Map of joint names to their measurements. */
   private final transient Map<String, LowLevelState> jointMeasurementMap = new HashMap<>();

   /** List of registered IMU names. */
   private final List<String> imuNames = new ArrayList<>();
   /** List of IMU measurements. */
   private final List<ImuData> imuMeasurements = new ArrayList<>();
   /** Set of IMU name hash codes for quick lookup. */
   private final transient TLongSet imuSet = new TLongHashSet();
   /** Map of IMU names to their measurements. */
   private final transient Map<String, ImuData> imuMeasurementMap = new HashMap<>();

   /** List of registered force sensor names. */
   private final List<String> forceSensorNames = new ArrayList<>();
   /** List of force sensor measurements. */
   private final List<DMatrixRMaj> forceSensorMeasurements = new ArrayList<>();
   /** Set of force sensor name hash codes for quick lookup. */
   private final transient TLongSet forceSensorSet = new TLongHashSet();
   /** Map of force sensor names to their measurements. */
   private final transient Map<String, DMatrixRMaj> forceSensorMeasurementMap = new HashMap<>();

   /**
    * Constructs an empty {@code SensorDataContext}.
    */
   public SensorDataContext()
   {
   }

   /**
    * Constructs a {@code SensorDataContext} and registers all joints, IMUs, and force sensors from the provided robot model.
    *
    * @param fullRobotModel The robot model containing joint, IMU, and force sensor definitions.
    */
   public SensorDataContext(FullRobotModel fullRobotModel)
   {
      Arrays.asList(fullRobotModel.getOneDoFJoints()).forEach(joint -> registerJoint(joint.getName()));
      Arrays.asList(fullRobotModel.getIMUDefinitions()).forEach(imu -> registerImu(imu.getName()));
      Arrays.asList(fullRobotModel.getForceSensorDefinitions()).forEach(sensor -> registerForceSensor(sensor.getSensorName()));
   }

   /**
    * Constructs a {@code SensorDataContext} and registers the provided joints.
    *
    * @param joints The list of joints to register.
    */
   public SensorDataContext(List<OneDoFJointBasics> joints)
   {
      joints.forEach(joint -> registerJoint(joint.getName()));
   }

   /**
    * Registers a joint with the given name and returns its associated measurement object.
    * Reuses existing measurement data if the joint was previously registered.
    *
    * @param jointName The name of the joint to register.
    * @return The {@link LowLevelState} object for the joint's measurements.
    * @throws RuntimeException If a joint with the same name hash is already registered.
    */
   public LowLevelState registerJoint(String jointName)
   {
      if (isJointRegistered(jointName))
      {
         throw new RuntimeException("Joint with same name hash as " + jointName + " is already registered.");
      }

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

   /**
    * Registers an IMU with the given name and returns its associated measurement object.
    * Reuses existing measurement data if the IMU was previously registered.
    *
    * @param imuName The name of the IMU to register.
    * @return The {@link ImuData} object for the IMU's measurements.
    * @throws RuntimeException If an IMU with the same name hash is already registered.
    */
   public ImuData registerImu(String imuName)
   {
      if (isImuRegistered(imuName))
      {
         throw new RuntimeException("IMU with same name hash as " + imuName + " is already registered.");
      }

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

   /**
    * Registers a force sensor with the given name and returns its associated measurement matrix.
    * Reuses existing measurement data if the force sensor was previously registered.
    *
    * @param forceSensorName The name of the force sensor to register.
    * @return The {@link DMatrixRMaj} object for the force sensor's measurements (6x1 matrix).
    * @throws RuntimeException If a force sensor with the same name hash is already registered.
    */
   public DMatrixRMaj registerForceSensor(String forceSensorName)
   {
      if (isForceSensorRegistered(forceSensorName))
      {
         throw new RuntimeException("Force Sensor with same name hash as " + forceSensorName + " is already registered.");
      }

      DMatrixRMaj forceSensorData = forceSensorMeasurementMap.get(forceSensorName);
      if (forceSensorData == null)
      {
         forceSensorData = new DMatrixRMaj(6, 1);
         forceSensorMeasurementMap.put(forceSensorName, forceSensorData);
      }

      forceSensorNames.add(forceSensorName);
      forceSensorMeasurements.add(forceSensorData);
      forceSensorSet.add(forceSensorName.hashCode());

      return forceSensorData;
   }

   /**
    * Retrieves the measurement data for the specified joint.
    *
    * @param jointName The name of the joint.
    * @return The {@link LowLevelState} object containing the joint's measurements.
    * @throws RuntimeException If the joint is not registered.
    */
   public LowLevelState getMeasuredJointState(String jointName)
   {
      if (!isJointRegistered(jointName))
      {
         throw new RuntimeException("Joint " + jointName + " was not registered in this data holder.");
      }
      return jointMeasurementMap.get(jointName);
   }

   /**
    * Retrieves the measurement data for the specified IMU.
    *
    * @param imuName The name of the IMU.
    * @return The {@link ImuData} object containing the IMU's measurements.
    * @throws RuntimeException If the IMU is not registered.
    */
   public ImuData getImuMeasurement(String imuName)
   {
      if (!isImuRegistered(imuName))
      {
         throw new RuntimeException("IMU " + imuName + " was not registered in this data holder.");
      }
      return imuMeasurementMap.get(imuName);
   }

   /**
    * Retrieves the measurement data for the specified force sensor.
    *
    * @param forceSensorName The name of the force sensor.
    * @return The {@link DMatrixRMaj} object containing the force sensor's measurements.
    * @throws RuntimeException If the force sensor is not registered.
    */
   public DMatrixRMaj getForceSensorMeasurement(String forceSensorName)
   {
      if (!isForceSensorRegistered(forceSensorName))
      {
         throw new RuntimeException("Force Sensor " + forceSensorName + " was not registered in this data holder.");
      }
      return forceSensorMeasurementMap.get(forceSensorName);
   }

   /**
    * Checks if a joint with the given name is registered.
    *
    * @param jointName The name of the joint.
    * @return {@code true} if the joint is registered, {@code false} otherwise.
    */
   public boolean isJointRegistered(String jointName)
   {
      return jointSet.contains(jointName.hashCode());
   }

   /**
    * Checks if an IMU with the given name is registered.
    *
    * @param imuName The name of the IMU.
    * @return {@code true} if the IMU is registered, {@code false} otherwise.
    */
   public boolean isImuRegistered(String imuName)
   {
      return imuSet.contains(imuName.hashCode());
   }

   /**
    * Checks if a force sensor with the given name is registered.
    *
    * @param forceSensorName The name of the force sensor.
    * @return {@code true} if the force sensor is registered, {@code false} otherwise.
    */
   public boolean isForceSensorRegistered(String forceSensorName)
   {
      return forceSensorSet.contains(forceSensorName.hashCode());
   }

   /**
    * Clears all registered sensors and their measurements.
    */
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

   /**
    * Copies the data from another {@code SensorDataContext} into this instance.
    * Clears existing data and registers all sensors from the other context.
    *
    * @param other The {@code SensorDataContext} to copy from.
    */
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

   /**
    * Compares this {@code SensorDataContext} with another object for equality.
    * Two {@code SensorDataContext} instances are equal if they have the same registered sensors
    * with equal measurements.
    *
    * @param obj The object to compare with.
    * @return {@code true} if the objects are equal, {@code false} otherwise.
    */
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

   /**
    * Returns a string representation of this {@code SensorDataContext}, listing all registered
    * sensors and their measurements.
    *
    * @return A string describing the sensor data.
    */
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
