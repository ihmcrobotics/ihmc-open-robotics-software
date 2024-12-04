package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.ControllerCoreOutputDataHolder;
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
   /**
    * Serves to inform the controller and estimator that the wholeBodyControllerCore ran and populated the desired values in
    * this context. Set by the wholeBodyControllerCore
    */
   private boolean wholeBodyControllerCoreRan = false;

   /**
    * The output of the wholebodyControllerCore. Set by the wholebodyControllerCore thread.
    */
   private final ControllerCoreOutputDataHolder controllerCoreOutPutDataHolder;

   /**
    * The output joint data from the WBCC. set by the WholeBodyController.
    * TODO This will be deleted after finishing moving WBCC from controllerThread to WBCCThread.
    */
   private final LowLevelOneDoFJointDesiredDataHolder wholeBodyControllerCoreDesiredOutPutList;

   /**
    * TODO: doc what this is
    */
   private final ControllerCoreCommandDataHolder controllerCoreCommandDataHolder;

   public HumanoidRobotMPCContextData()
   {
      super();
      wholeBodyControllerCoreDesiredOutPutList = new LowLevelOneDoFJointDesiredDataHolder();
      controllerCoreOutPutDataHolder = new ControllerCoreOutputDataHolder(null);
      controllerCoreCommandDataHolder = new ControllerCoreCommandDataHolder();
   }

   public HumanoidRobotMPCContextData(HumanoidRobotContextJointData processedJointData,
                                      ForceSensorDataHolder forceSensorDataHolder,
                                      CenterOfMassDataHolder centerOfMassDataHolder,
                                      CenterOfPressureDataHolder centerOfPressureDataHolder,
                                      RobotMotionStatusHolder robotMotionStatusHolder,
                                      LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList,
                                      SensorDataContext sensorDataContext,
                                      LowLevelOneDoFJointDesiredDataHolder wbccJointDesiredOutputList,
                                      ControllerCoreCommandDataHolder controllerCoreCommandDataHolder,
                                      ControllerCoreOutputDataHolder controllerCoreOutPutDataHolder)
   {
      super(processedJointData,
            forceSensorDataHolder,
            centerOfMassDataHolder,
            centerOfPressureDataHolder,
            robotMotionStatusHolder,
            jointDesiredOutputList,
            sensorDataContext);
      this.wholeBodyControllerCoreDesiredOutPutList = wbccJointDesiredOutputList;
      this.controllerCoreOutPutDataHolder = controllerCoreOutPutDataHolder;
      this.controllerCoreCommandDataHolder = controllerCoreCommandDataHolder;
   }

   public HumanoidRobotMPCContextData(FullHumanoidRobotModel fullRobotModel)
   {
      super(fullRobotModel);
      wholeBodyControllerCoreDesiredOutPutList = new LowLevelOneDoFJointDesiredDataHolder(fullRobotModel.getControllableOneDoFJoints());
      controllerCoreOutPutDataHolder = new ControllerCoreOutputDataHolder(fullRobotModel.getControllableOneDoFJoints());
      controllerCoreCommandDataHolder = new ControllerCoreCommandDataHolder();
   }

   public HumanoidRobotMPCContextData(List<OneDoFJointBasics> joints)
   {
      super(joints);
      wholeBodyControllerCoreDesiredOutPutList = new LowLevelOneDoFJointDesiredDataHolder(joints.toArray(new OneDoFJointBasics[0]));
      controllerCoreOutPutDataHolder = new ControllerCoreOutputDataHolder(joints.toArray(new OneDoFJointBasics[0]));
      controllerCoreCommandDataHolder = new ControllerCoreCommandDataHolder();
   }

   public LowLevelOneDoFJointDesiredDataHolder getWholeBodyControllerCoreDesiredOutPutList()
   {
      return wholeBodyControllerCoreDesiredOutPutList;
   }

   public ControllerCoreOutputDataHolder getControllerCoreOutPutDataHolder()
   {
      return controllerCoreOutPutDataHolder;
   }

   public ControllerCoreCommandDataHolder getControllerCoreCommandDataHolder()
   {
      return controllerCoreCommandDataHolder;
   }

   @Override
   public void set(HumanoidRobotContextData other)
   {
      copyFrom(other);
   }

   @Override
   public void copyFrom(HumanoidRobotContextData src)
   {
      super.copyFrom(src);
      if (src instanceof HumanoidRobotMPCContextData srcMPC)
      {
         wholeBodyControllerCoreRan = srcMPC.wholeBodyControllerCoreRan;
         wholeBodyControllerCoreDesiredOutPutList.set(srcMPC.wholeBodyControllerCoreDesiredOutPutList);
         controllerCoreOutPutDataHolder.set(srcMPC.controllerCoreOutPutDataHolder);
         controllerCoreCommandDataHolder.setControllerCoreMode(srcMPC.controllerCoreCommandDataHolder.getControllerCoreMode());
         controllerCoreCommandDataHolder.set(srcMPC.controllerCoreCommandDataHolder);
      }
   }

   public boolean getWholeBodyControllerCoreRan()
   {
      return wholeBodyControllerCoreRan;
   }

   public void setWholeBodyControllerCoreRan(boolean wholeBodyControllerCoreRan)
   {
      this.wholeBodyControllerCoreRan = wholeBodyControllerCoreRan;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof HumanoidRobotMPCContextData)
      {
         HumanoidRobotMPCContextData other = (HumanoidRobotMPCContextData) obj;
         if (wholeBodyControllerCoreRan != other.wholeBodyControllerCoreRan)
            return false;
         if (!wholeBodyControllerCoreDesiredOutPutList.equals(other.wholeBodyControllerCoreDesiredOutPutList))
            return false;
         if (!controllerCoreOutPutDataHolder.equals(other.controllerCoreOutPutDataHolder))
            return false;
         if (!controllerCoreCommandDataHolder.equals(other.controllerCoreCommandDataHolder))
            return false;
         return super.equals(other);
      }
      else
      {
         return false;
      }
   }
}