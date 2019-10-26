package us.ihmc.footstepPlanning.postProcessing;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.List;

public class AreaSplitFractionPostProcessingElement implements FootstepPlanPostProcessingElement
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FootstepPostProcessingParametersReadOnly parameters;
   private final ICPPlannerParameters walkingControllerParameters;

   public AreaSplitFractionPostProcessingElement(FootstepPostProcessingParametersReadOnly parameters, ICPPlannerParameters walkingControllerParameters)
   {
      this.parameters = parameters;
      this.walkingControllerParameters = walkingControllerParameters;
   }

   /** {@inheritDoc} **/
   @Override
   public boolean isActive()
   {
      return parameters.areaSplitFractionProcessingEnabled();
   }

   /** {@inheritDoc} **/
   @Override
   public FootstepPlanningToolboxOutputStatus postProcessFootstepPlan(FootstepPlanningRequestPacket request, FootstepPlanningToolboxOutputStatus outputStatus)
   {
      FootstepPlanningToolboxOutputStatus processedOutput = new FootstepPlanningToolboxOutputStatus(outputStatus);

      ConvexPolygon2D previousPolygon = new ConvexPolygon2D();
      ConvexPolygon2D currentPolygon = new ConvexPolygon2D();

      PoseReferenceFrame previousFrame = new PoseReferenceFrame("previousFrame", worldFrame);
      PoseReferenceFrame currentFrame = new PoseReferenceFrame("nextFrame", worldFrame);

      double defaultTransferSplitFraction = walkingControllerParameters.getTransferSplitFraction();
      double defaultWeightDistribution = 0.5;

      List<FootstepDataMessage> footstepDataMessageList = processedOutput.getFootstepDataList().getFootstepDataList();
      for (int stepNumber = 1; stepNumber < footstepDataMessageList.size(); stepNumber++)
      {
         FootstepDataMessage previousStep = footstepDataMessageList.get(stepNumber - 1);
         FootstepDataMessage currentStep = footstepDataMessageList.get(stepNumber);

         previousFrame.setPositionAndUpdate(new FramePoint3D(worldFrame, previousStep.getLocation()));
         previousFrame.setOrientationAndUpdate(previousStep.getOrientation());

         currentFrame.setPositionAndUpdate(new FramePoint3D(worldFrame, currentStep.getLocation()));
         currentFrame.setOrientationAndUpdate(currentStep.getOrientation());

         for (Point3DReadOnly vertex : previousStep.getPredictedContactPoints2d())
         {
            FramePoint3D vertexInSoleFrame = new FramePoint3D(worldFrame, vertex);
            vertexInSoleFrame.changeFrame(previousFrame);

            previousPolygon.addVertex(vertexInSoleFrame);
         }

         for (Point3DReadOnly vertex : currentStep.getPredictedContactPoints2d())
         {
            FramePoint3D vertexInSoleFrame = new FramePoint3D(worldFrame, vertex);
            vertexInSoleFrame.changeFrame(currentFrame);

            currentPolygon.addVertex(vertexInSoleFrame);
         }

         double currentArea = currentPolygon.getArea();
         double previousArea = previousPolygon.getArea();

         double percentAreaOnCurrentFoot = currentArea / (previousArea + currentArea);
         double transferWeightDistribution = percentAreaOnCurrentFoot * parameters.getFractionLoadIfFootHasFullSupport();

         // lower means it spends more time shifting to the center, higher means it spends less time shifting to the center
         // e.g., if we set the fraction to 0 and the trailing foot has no area, the split fraction should be 1 because we spend no time on the first segment
         double transferSplitFraction =  percentAreaOnCurrentFoot * (1.0 - parameters.getFractionTimeOnFootIfFootHasFullSupport());

         transferWeightDistribution = MathTools.clamp(transferWeightDistribution, 0.01, 0.99);
         transferSplitFraction = MathTools.clamp(transferSplitFraction, 0.01, 0.99);

         if (stepNumber == footstepDataMessageList.size() - 1)
         { // this is the last step
            double currentSplitFraction = processedOutput.getFootstepDataList().getFinalTransferSplitFraction();
            double currentWeightDistribution = processedOutput.getFootstepDataList().getFinalTransferWeightDistribution();

            double splitFractionToSet, weightDistributionToSet;

            if (currentSplitFraction == -1.0)
               splitFractionToSet = transferSplitFraction;
            else
               splitFractionToSet = transferSplitFraction * currentSplitFraction / defaultTransferSplitFraction;

            if (currentWeightDistribution == -1.0)
               weightDistributionToSet = transferWeightDistribution;
            else
               weightDistributionToSet = transferWeightDistribution * currentWeightDistribution / defaultWeightDistribution;

            processedOutput.getFootstepDataList().setFinalTransferSplitFraction(splitFractionToSet);
            processedOutput.getFootstepDataList().setFinalTransferWeightDistribution(weightDistributionToSet);
         }
         else
         {
            double currentSplitFraction = footstepDataMessageList.get(stepNumber + 1).getTransferSplitFraction();
            double currentWeightDistribution = footstepDataMessageList.get(stepNumber + 1).getTransferWeightDistribution();

            double splitFractionToSet, weightDistributionToSet;

            if (currentSplitFraction == -1.0)
               splitFractionToSet = transferSplitFraction;
            else
               splitFractionToSet = transferSplitFraction * currentSplitFraction / defaultTransferSplitFraction;

            if (currentWeightDistribution == -1.0)
               weightDistributionToSet = transferWeightDistribution;
            else
               weightDistributionToSet = transferWeightDistribution * currentWeightDistribution / defaultWeightDistribution;

            footstepDataMessageList.get(stepNumber + 1).setTransferSplitFraction(splitFractionToSet);
            footstepDataMessageList.get(stepNumber + 1).setTransferWeightDistribution(weightDistributionToSet);
         }
      }

      return processedOutput;
   }

   /** {@inheritDoc} **/
   @Override
   public PostProcessingEnum getElementName()
   {
      return PostProcessingEnum.AREA_SPLIT_FRACTIONS;
   }
}
