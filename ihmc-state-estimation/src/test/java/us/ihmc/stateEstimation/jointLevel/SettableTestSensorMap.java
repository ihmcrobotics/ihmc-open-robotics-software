package us.ihmc.stateEstimation.jointLevel;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

/**
 * Public, settable {@link SensorOutputMapReadOnly} for estimator tests: serves a fixed list of IMUs plus a
 * settable encoder state (position/velocity/effort) for every supplied joint. Reusable across test packages
 * (the {@code JointLevelKFTestFixture} equivalent is package-private and joint-chain specific).
 */
public class SettableTestSensorMap implements SensorOutputMapReadOnly
{
   private final List<IMUSensorReadOnly> imus;
   private final Map<OneDoFJointBasics, JointState> states = new LinkedHashMap<>();
   private final List<OneDoFJointStateReadOnly> stateList = new ArrayList<>();

   public SettableTestSensorMap(List<? extends IMUSensorReadOnly> imus, Iterable<? extends OneDoFJointBasics> joints)
   {
      this.imus = new ArrayList<>(imus);
      for (OneDoFJointBasics joint : joints)
      {
         JointState state = new JointState(joint.getName());
         states.put(joint, state);
         stateList.add(state);
      }
   }

   public void setPosition(OneDoFJointBasics joint, double q)
   {
      states.get(joint).position = q;
   }

   public void setVelocity(OneDoFJointBasics joint, double qd)
   {
      states.get(joint).velocity = qd;
   }

   @Override
   public OneDoFJointStateReadOnly getOneDoFJointOutput(OneDoFJointBasics oneDoFJoint)
   {
      return states.get(oneDoFJoint);
   }

   @Override
   public List<? extends OneDoFJointStateReadOnly> getOneDoFJointOutputs()
   {
      return stateList;
   }

   @Override
   public List<? extends IMUSensorReadOnly> getIMUOutputs()
   {
      return imus;
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorOutputs()
   {
      return null;
   }

   @Override
   public long getWallTime()
   {
      return 0L;
   }

   @Override
   public long getMonotonicTime()
   {
      return 0L;
   }

   @Override
   public long getSyncTimestamp()
   {
      return 0L;
   }

   private static final class JointState implements OneDoFJointStateReadOnly
   {
      private final String name;
      private double position;
      private double velocity;
      private double effort;

      private JointState(String name)
      {
         this.name = name;
      }

      @Override
      public String getJointName()
      {
         return name;
      }

      @Override
      public double getPosition()
      {
         return position;
      }

      @Override
      public double getVelocity()
      {
         return velocity;
      }

      @Override
      public double getAcceleration()
      {
         return 0.0;
      }

      @Override
      public double getEffort()
      {
         return effort;
      }

      @Override
      public boolean isJointEnabled()
      {
         return true;
      }
   }
}
