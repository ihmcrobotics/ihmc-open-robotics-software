package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class KSTLowPassFilteredOutputProcessor implements KSTOutputProcessor
{
   private final YoKinematicsToolboxOutputStatus filteredRobotState;
   private final DoubleProvider breakFrequency;
   private final double updateDT;

   private boolean resetFilter = false;

   public KSTLowPassFilteredOutputProcessor(KSTTools tools, DoubleProvider breakFrequency, YoRegistry registry)
   {
      this.breakFrequency = breakFrequency;
      updateDT = tools.getToolboxControllerPeriod();

      FullHumanoidRobotModel desiredFullRobotModel = tools.getDesiredFullRobotModel();
      FloatingJointBasics rootJoint = desiredFullRobotModel.getRootJoint();
      OneDoFJointBasics[] oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(desiredFullRobotModel);
      filteredRobotState = new YoKinematicsToolboxOutputStatus("filtered", rootJoint, oneDoFJoints, registry);
   }

   @Override
   public void initialize()
   {
      resetFilter = true;
      filteredRobotState.setToNaN();
   }

   @Override
   public void update(double time, boolean wasStreaming, boolean isStreaming, KinematicsToolboxOutputStatus latestOutput)
   {
      if (resetFilter)
      {
         filteredRobotState.set(latestOutput);
         resetFilter = false;
      }
      else
      {
         double alphaFilter = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency.getValue(), updateDT);
         filteredRobotState.interpolate(latestOutput, filteredRobotState.getStatus(), alphaFilter);
      }
   }

   @Override
   public KinematicsToolboxOutputStatus getProcessedOutput()
   {
      return filteredRobotState.getStatus();
   }
}
