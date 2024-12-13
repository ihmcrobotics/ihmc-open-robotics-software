package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;

import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
@SuppressWarnings("serial")
public class HumanoidRobotMPCContextData extends HumanoidRobotContextData
{
   private boolean wholeBodyControllerCoreRan = false;

   public HumanoidRobotMPCContextData()
   {
      super();
   }

   public HumanoidRobotMPCContextData(HumanoidRobotContextJointData processedJointData,
                                      ForceSensorDataHolder forceSensorDataHolder,
                                      CenterOfMassDataHolder centerOfMassDataHolder,
                                      CenterOfPressureDataHolder centerOfPressureDataHolder,
                                      RobotMotionStatusHolder robotMotionStatusHolder,
                                      LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList,
                                      SensorDataContext sensorDataContext)
   {
      super(processedJointData,
            forceSensorDataHolder,
            centerOfMassDataHolder,
            centerOfPressureDataHolder,
            robotMotionStatusHolder,
            jointDesiredOutputList,
            sensorDataContext);
   }

   public HumanoidRobotMPCContextData(FullHumanoidRobotModel fullRobotModel)
   {
      super(fullRobotModel);
   }

   public HumanoidRobotMPCContextData(List<OneDoFJointBasics> joints)
   {
      super(joints);
   }

   public void setWholeBodyControllerCoreRan(boolean wholeBodyControllerCoreRan)
   {
      this.wholeBodyControllerCoreRan = wholeBodyControllerCoreRan;
   }

   public void set(HumanoidRobotMPCContextData other)
   {
      copyFrom(other);
   }

   public void copyFrom(HumanoidRobotMPCContextData src)
   {
      super.copyFrom(src);
   }

   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof HumanoidRobotMPCContextData other)
      {
         return super.equals(other);
      }
      else
      {
         return false;
      }
   }
}
