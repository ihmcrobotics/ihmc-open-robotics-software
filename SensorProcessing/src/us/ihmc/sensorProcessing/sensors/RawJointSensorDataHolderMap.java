package us.ihmc.sensorProcessing.sensors;

import java.util.LinkedHashMap;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class RawJointSensorDataHolderMap extends LinkedHashMap<OneDoFJoint, RawJointSensorDataHolder>
{
   private static final long serialVersionUID = 743946164652993907L;

   public RawJointSensorDataHolderMap(FullRobotModel fullRobotModel)
   {
      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();
      
      for(OneDoFJoint joint : joints)
      {
         RawJointSensorDataHolder dataHolder = new RawJointSensorDataHolder(joint.getName());
         put(joint, dataHolder);
      }
   }
}
