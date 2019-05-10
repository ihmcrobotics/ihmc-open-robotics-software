package us.ihmc.sensorProcessing.sensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;

// TODO: move this to atlas specific code once switch to barrier scheduler is complete.
public class RawJointSensorDataHolderMap implements Settable<RawJointSensorDataHolderMap>
{
   private final List<OneDoFJointBasics> joints = new ArrayList<>();
   private final RecyclingArrayList<RawJointSensorDataHolder> rawJointSensorDataHolders = new RecyclingArrayList<>(RawJointSensorDataHolder.class);

   private final transient Map<String, RawJointSensorDataHolder> rawJointSensorDataHolderMap = new HashMap<>();

   public RawJointSensorDataHolderMap()
   {
   }

   public RawJointSensorDataHolderMap(FullRobotModel fullRobotModel)
   {
      OneDoFJointBasics[] joints = fullRobotModel.getOneDoFJoints();
      for (OneDoFJointBasics joint : joints)
      {
         registerJoint(joint);
      }
   }

   public void registerJoint(OneDoFJointBasics joint)
   {
      joints.add(joint);
      RawJointSensorDataHolder rawData = rawJointSensorDataHolders.add();
      if (rawJointSensorDataHolderMap.put(joint.getName(), rawData) != null)
      {
         throw new RuntimeException("Already have joint " + joint.getName());
      }
   }

   public void registerJoint(OneDoFJointBasics joint, RawJointSensorDataHolder rawJointSensorDataHolder)
   {
      joints.add(joint);
      RawJointSensorDataHolder rawData = rawJointSensorDataHolders.add();
      rawData.set(rawJointSensorDataHolder);
      if (rawJointSensorDataHolderMap.put(joint.getName(), rawData) != null)
      {
         throw new RuntimeException("Already have joint " + joint.getName());
      }
   }

   public void clear()
   {
      joints.clear();
      rawJointSensorDataHolders.clear();
      rawJointSensorDataHolderMap.clear();
   }

   @Override
   public void set(RawJointSensorDataHolderMap other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         registerJoint(other.getJoint(i), other.get(i));
      }
   }

   public int getNumberOfJoints()
   {
      return joints.size();
   }

   public OneDoFJointBasics getJoint(int jointIndex)
   {
      return joints.get(jointIndex);
   }

   public RawJointSensorDataHolder get(String jointName)
   {
      return rawJointSensorDataHolderMap.get(jointName);
   }

   public RawJointSensorDataHolder get(int jointIndex)
   {
      return rawJointSensorDataHolders.get(jointIndex);
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof RawJointSensorDataHolderMap)
      {
         RawJointSensorDataHolderMap other = (RawJointSensorDataHolderMap) obj;
         if (getNumberOfJoints() != other.getNumberOfJoints())
            return false;
         for (int i = 0; i < getNumberOfJoints(); i++)
         {
            OneDoFJointBasics joint = getJoint(i);
            if (!get(i).equals(other.get(joint.getName())))
               return false;
         }
         return true;
      }
      else
      {
         return false;
      }
   }
}
