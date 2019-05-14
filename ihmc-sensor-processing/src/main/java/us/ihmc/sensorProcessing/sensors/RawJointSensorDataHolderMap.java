package us.ihmc.sensorProcessing.sensors;

import java.util.LinkedHashMap;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;

// TODO: move this to atlas specific code one switch to barrier scheduler is complete.
public class RawJointSensorDataHolderMap extends LinkedHashMap<String, RawJointSensorDataHolder>
{
   private static final long serialVersionUID = 743946164652993907L;

   public RawJointSensorDataHolderMap(FullRobotModel fullRobotModel)
   {
      OneDoFJointBasics[] joints = fullRobotModel.getOneDoFJoints();
      
      for(OneDoFJointBasics joint : joints)
      {
         RawJointSensorDataHolder dataHolder = new RawJointSensorDataHolder(joint.getName());
         if (put(joint.getName(), dataHolder) != null)
         {
            throw new RuntimeException("Have duplicate joint " + joint.getName());
         }
      }
   }
}
