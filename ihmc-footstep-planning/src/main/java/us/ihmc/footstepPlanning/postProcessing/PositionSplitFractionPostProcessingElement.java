package us.ihmc.footstepPlanning.postProcessing;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.FootstepPostProcessingPacket;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

public class PositionSplitFractionPostProcessingElement implements FootstepPlanPostProcessingElement
{
   private final FootstepPostProcessingParametersReadOnly parameters;
   private final ICPPlannerParameters walkingControllerParameters;

   public PositionSplitFractionPostProcessingElement(FootstepPostProcessingParametersReadOnly parameters,
                                                     ICPPlannerParameters walkingControllerParameters)
   {
      this.parameters = parameters;
      this.walkingControllerParameters = walkingControllerParameters;
   }

   /** {@inheritDoc} **/
   @Override
   public boolean isActive()
   {
      return parameters.positionSplitFractionProcessingEnabled();
   }

   /** {@inheritDoc} **/
   @Override
   public FootstepPostProcessingPacket postProcessFootstepPlan(FootstepPostProcessingPacket outputPlan)
   {
      FootstepPostProcessingPacket processedPlan = new FootstepPostProcessingPacket(outputPlan);

      FramePose3D stanceFootPose = new FramePose3D();
      RobotSide initialStanceSide = RobotSide.fromByte(outputPlan.getFootstepDataList().getFootstepDataList().get(0).getRobotSide()).getOppositeSide();
      if (initialStanceSide == RobotSide.LEFT)
      {
         stanceFootPose.setPosition(outputPlan.getLeftFootPositionInWorld());
         stanceFootPose.setOrientation(outputPlan.getLeftFootOrientationInWorld());
      }
      else
      {
         stanceFootPose.setPosition(outputPlan.getRightFootPositionInWorld());
         stanceFootPose.setOrientation(outputPlan.getRightFootOrientationInWorld());
      }

      FramePose3D nextFootPose = new FramePose3D();

      double defaultTransferSplitFraction = walkingControllerParameters.getTransferSplitFraction();
      double defaultWeightDistribution = 0.5;

      List<FootstepDataMessage> footstepDataMessageList = processedPlan.getFootstepDataList().getFootstepDataList();
      for (int stepNumber = 0; stepNumber < footstepDataMessageList.size(); stepNumber++)
      {
         if (stepNumber > 0)
         {
            stanceFootPose.setPosition(footstepDataMessageList.get(stepNumber - 1).getLocation());
            stanceFootPose.setOrientation(footstepDataMessageList.get(stepNumber - 1).getOrientation());
         }

         nextFootPose.setPosition(footstepDataMessageList.get(stepNumber).getLocation());
         nextFootPose.setOrientation(footstepDataMessageList.get(stepNumber).getOrientation());

         // This step is a big step down.
         double stepDownHeight = nextFootPose.getZ() - stanceFootPose.getZ();

         if (stepDownHeight < -parameters.getStepHeightForLargeStepDown())
         {
            double alpha = Math.min(1.0, (Math.abs(stepDownHeight) - parameters.getStepHeightForLargeStepDown()) / (parameters.getLargestStepDownHeight() - parameters.getStepHeightForLargeStepDown()));
            double transferSplitFraction = InterpolationTools.linearInterpolate(defaultTransferSplitFraction,
                                                                                parameters.getTransferSplitFractionAtFullDepth(), alpha);
            double transferWeightDistribution = InterpolationTools.linearInterpolate(defaultWeightDistribution,
                                                                                     parameters.getTransferWeightDistributionAtFullDepth(), alpha);

            if (stepNumber == footstepDataMessageList.size() - 1)
            { // this is the last step
               double currentSplitFraction = processedPlan.getFootstepDataList().getFinalTransferSplitFraction();
               double currentWeightDistribution = processedPlan.getFootstepDataList().getFinalTransferWeightDistribution();

               double splitFractionToSet = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultTransferSplitFraction);
               double weightDistributionToSet = SplitFractionTools.appendWeightDistribution(transferWeightDistribution, currentWeightDistribution, defaultWeightDistribution);

               processedPlan.getFootstepDataList().setFinalTransferSplitFraction(splitFractionToSet);
               processedPlan.getFootstepDataList().setFinalTransferWeightDistribution(weightDistributionToSet);
            }
            else
            {
               double currentSplitFraction = footstepDataMessageList.get(stepNumber + 1).getTransferSplitFraction();
               double currentWeightDistribution = footstepDataMessageList.get(stepNumber + 1).getTransferWeightDistribution();

               double splitFractionToSet = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultTransferSplitFraction);
               double weightDistributionToSet = SplitFractionTools.appendWeightDistribution(transferWeightDistribution, currentWeightDistribution, defaultWeightDistribution);

               footstepDataMessageList.get(stepNumber + 1).setTransferSplitFraction(splitFractionToSet);
               footstepDataMessageList.get(stepNumber + 1).setTransferWeightDistribution(weightDistributionToSet);
            }
         }


      }

      return processedPlan;
   }

   /** {@inheritDoc} **/
   @Override
   public PostProcessingEnum getElementName()
   {
      return PostProcessingEnum.STEP_SPLIT_FRACTIONS;
   }
}
