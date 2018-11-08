package us.ihmc.sensorProcessing.sensors;

import java.util.LinkedHashMap;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;

public class RawJointSensorDataHolderMap extends LinkedHashMap<OneDoFJointBasics, RawJointSensorDataHolder>
{
   private static final long serialVersionUID = 743946164652993907L;

   public RawJointSensorDataHolderMap(FullRobotModel fullRobotModel)
   {
      OneDoFJointBasics[] joints = fullRobotModel.getOneDoFJoints();
      
      for(OneDoFJointBasics joint : joints)
      {
         RawJointSensorDataHolder dataHolder = new RawJointSensorDataHolder(joint.getName());
         put(joint, dataHolder);
      }
   }
}
