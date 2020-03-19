package us.ihmc.footstepPlanning.postProcessing;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPostProcessingPacket;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

/**
 * The purpose of this class is to modify elements of the dynamic trajectory planner based on the step position.
 * Currently, the transfer phase is split into two segments. One where the CoP goes from the trailing foot to some midpoint, then the second where it goes
 * from the midpoint to the leading foot. The midpoint is determined by interpolating between the trailing foot and the leading foot by some fraction set by
 * the desired weight distribution {@link FootstepDataMessage#transfer_weight_distribution_}. The time spent shifting from the trailing foot to the midpoint
 * is some fraction of the transfer duration determined by the transfer split fraction {@link FootstepDataMessage#transfer_split_fraction_}.
 *
 * <p>
 *    This module defines several parameters that allow the weight distribution and split fraction to be modified based on the step position, specifically
 *    changes in step height. This is done so that the robot "commits" to large step downs. That is to say, when the robot is stepping down, there is a minimum
 *    step down height that causes it to start to shift its upcoming CoP midpoint and time to cause it to rapidly shift the weight on the next step.
 * </p>
 * <p>
 *    This is done by defining the desired weight distribution and split fraction if the upcoming foothold is below a certain threshold, defined by some maximum
 *    step down distance. Then, once the step is below another threshold, it starts linearly interpolating to these values, until it has reached its largest
 *    step down distance.
 * </p>
 */
public class PositionSplitFractionPostProcessingElement implements FootstepPlanPostProcessingElement
{
   private final FootstepPostProcessingParametersReadOnly parameters;
   private final ICPPlannerParameters icpPlannerParameters;

   public PositionSplitFractionPostProcessingElement(FootstepPostProcessingParametersReadOnly parameters, ICPPlannerParameters icpPlannerParameters)
   {
      this.parameters = parameters;
      this.icpPlannerParameters = icpPlannerParameters;
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

      double defaultTransferSplitFraction = icpPlannerParameters.getTransferSplitFraction();
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
