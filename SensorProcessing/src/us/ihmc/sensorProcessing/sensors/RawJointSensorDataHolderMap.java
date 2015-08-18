package us.ihmc.sensorProcessing.sensors;

import java.util.LinkedHashMap;

import us.ihmc.humanoidRobotics.model.BaseFullRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class RawJointSensorDataHolderMap extends LinkedHashMap<OneDoFJoint, RawJointSensorDataHolder>
{
   private static final long serialVersionUID = 743946164652993907L;

   public RawJointSensorDataHolderMap(BaseFullRobotModel fullRobotModel)
   {
      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();
      
      for(OneDoFJoint joint : joints)
      {
         RawJointSensorDataHolder dataHolder = new RawJointSensorDataHolder(joint.getName());
         put(joint, dataHolder);
      }
   }
}
