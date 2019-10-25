package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.parameters.FootstepPostProcessingParametersReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class StepSplitFractionPostProcessingElement implements FootstepPlanPostProcessingElement
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble stepHeightForLargeStepDown = new YoDouble("stepHeightForLargeStepDown", registry);

   private final FootstepPostProcessingParametersReadOnly parameters;

   public StepSplitFractionPostProcessingElement(FootstepPostProcessingParametersReadOnly parameters, YoVariableRegistry parentRegistry)
   {
      this.parameters = parameters;

      stepHeightForLargeStepDown.set(0.15);

      parentRegistry.addChild(registry);
   }

   /** {@inheritDoc} **/
   @Override
   public boolean isActive()
   {
      return parameters.splitFractionProcessingEnabled();
   }

   /** {@inheritDoc} **/
   @Override
   public FootstepPlanningToolboxOutputStatus postProcessFootstepPlan(FootstepPlanningRequestPacket request, FootstepPlanningToolboxOutputStatus outputStatus)
   {
      FootstepPlanningToolboxOutputStatus processedOutput = new FootstepPlanningToolboxOutputStatus(outputStatus);

      FramePose3D stanceFootPose = new FramePose3D();
      stanceFootPose.setPosition(request.getStanceFootPositionInWorld());
      stanceFootPose.setOrientation(request.getStanceFootOrientationInWorld());

      FramePose3D nextFootPose = new FramePose3D();

      List<FootstepDataMessage> footstepDataMessageList = processedOutput.getFootstepDataList().getFootstepDataList();
      for (int stepNumber = 0; stepNumber < footstepDataMessageList.size(); stepNumber++)
      {
         if (stepNumber > 0)
         {
            stanceFootPose.setPosition(footstepDataMessageList.get(stepNumber - 1).getLocation());
            stanceFootPose.setOrientation(footstepDataMessageList.get(stepNumber - 1).getOrientation());
         }

         nextFootPose.setPosition(footstepDataMessageList.get(stepNumber).getLocation());
         nextFootPose.setOrientation(footstepDataMessageList.get(stepNumber).getOrientation());

         // TODO scale the split fraction with the distance down.
         // This step is a big step down.
         if (nextFootPose.getZ() - stanceFootPose.getZ() < -stepHeightForLargeStepDown.getDoubleValue())
         {
            if (stepNumber == footstepDataMessageList.size() - 1)
            { // this is the last step
               processedOutput.getFootstepDataList().setFinalTransferSplitFraction(0.01);
               processedOutput.getFootstepDataList().setFinalTransferWeightDistribution(0.7);
            }
            else
            {
               footstepDataMessageList.get(stepNumber + 1).setTransferSplitFraction(0.01);
               footstepDataMessageList.get(stepNumber + 1).setTransferWeightDistribution(0.7);
            }
         }

      }

      return processedOutput;
   }

   /** {@inheritDoc} **/
   @Override
   public PostProcessingEnum getElementName()
   {
      return PostProcessingEnum.STEP_SPLIT_FRACTIONS;
   }
}
