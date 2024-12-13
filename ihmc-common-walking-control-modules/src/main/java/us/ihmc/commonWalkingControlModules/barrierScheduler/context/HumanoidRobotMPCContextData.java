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
   private final LowLevelOneDoFJointDesiredDataHolder mpcControllerDesiredOutputList;

   public HumanoidRobotMPCContextData()
   {
      super();
      mpcControllerDesiredOutputList = new LowLevelOneDoFJointDesiredDataHolder();
   }

   public HumanoidRobotMPCContextData(HumanoidRobotContextJointData processedJointData,
                                      ForceSensorDataHolder forceSensorDataHolder,
                                      CenterOfMassDataHolder centerOfMassDataHolder,
                                      CenterOfPressureDataHolder centerOfPressureDataHolder,
                                      RobotMotionStatusHolder robotMotionStatusHolder,
                                      LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList,
                                      SensorDataContext sensorDataContext,
                                      LowLevelOneDoFJointDesiredDataHolder mpcControllerDesiredOutputList)
   {
      super(processedJointData,
            forceSensorDataHolder,
            centerOfMassDataHolder,
            centerOfPressureDataHolder,
            robotMotionStatusHolder,
            jointDesiredOutputList,
            sensorDataContext);
      this.mpcControllerDesiredOutputList = mpcControllerDesiredOutputList;
   }

   public HumanoidRobotMPCContextData(FullHumanoidRobotModel fullRobotModel)
   {
      super(fullRobotModel);
      mpcControllerDesiredOutputList = new LowLevelOneDoFJointDesiredDataHolder(fullRobotModel.getControllableOneDoFJoints());
   }

   public HumanoidRobotMPCContextData(List<OneDoFJointBasics> joints)
   {
      super(joints);
      mpcControllerDesiredOutputList = new LowLevelOneDoFJointDesiredDataHolder(joints.toArray(new OneDoFJointBasics[0]));
   }

   public void setWholeBodyControllerCoreRan(boolean wholeBodyControllerCoreRan)
   {
      this.wholeBodyControllerCoreRan = wholeBodyControllerCoreRan;
   }

   public boolean getWholeBodyControllerCoreRan()
   {
      return wholeBodyControllerCoreRan;
   }

   public void set(HumanoidRobotMPCContextData other)
   {
      copyFrom(other);
   }

   public void copyFrom(HumanoidRobotMPCContextData src)
   {
      super.copyFrom(src);
      mpcControllerDesiredOutputList.set(src.mpcControllerDesiredOutputList);
   }

   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof HumanoidRobotMPCContextData other)
      {
         if (!mpcControllerDesiredOutputList.equals(other.mpcControllerDesiredOutputList))
            return false;
         return super.equals(other);
      }
      else
      {
         return false;
      }
   }
}
