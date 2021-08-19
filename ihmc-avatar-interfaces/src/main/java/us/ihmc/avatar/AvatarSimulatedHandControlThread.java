package us.ihmc.avatar;

import java.util.List;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.outputData.LowLevelState;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface AvatarSimulatedHandControlThread
{
   void run();

   YoRegistry getYoVariableRegistry();

   FullHumanoidRobotModel getFullRobotModel();

   HumanoidRobotContextData getHumanoidRobotContextData();

   List<OneDoFJointBasics> getControlledOneDoFJoints();

   boolean hasControllerRan();

   void cleanup();

   default void updateFullRobotModel()
   {
      List<OneDoFJointBasics> controlledOneDoFJoints = getControlledOneDoFJoints();
      for (int i = 0; i < controlledOneDoFJoints.size(); i++)
      {
         OneDoFJointBasics joint = controlledOneDoFJoints.get(i);
         LowLevelState measuredJointState = getHumanoidRobotContextData().getSensorDataContext().getMeasuredJointState(joint.getName());

         if (measuredJointState.isPositionValid())
            joint.setQ(measuredJointState.getPosition());
         if (measuredJointState.isVelocityValid())
            joint.setQd(measuredJointState.getVelocity());
         if (measuredJointState.isAccelerationValid())
            joint.setQdd(measuredJointState.getAcceleration());
         if (measuredJointState.isEffortValid())
            joint.setTau(measuredJointState.getEffort());
      }
   }
}
