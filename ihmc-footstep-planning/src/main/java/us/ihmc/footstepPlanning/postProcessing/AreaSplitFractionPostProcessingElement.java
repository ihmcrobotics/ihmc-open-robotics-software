package us.ihmc.footstepPlanning.postProcessing;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPostProcessingPacket;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

/**
 * The purpose of this class is to modify elements of the dynamic trajectory planner based on the comparative quality between footholds.
 * Currently, the transfer phase is split into two segments. One where the CoP goes from the trailing foot to some midpoint, then the second where it goes
 * from the midpoint to the leading foot. The midpoint is determined by interpolating between the trailing foot and the leading foot by some fraction set by
 * the desired weight distribution {@link FootstepDataMessage#transfer_weight_distribution_}. The time spent shifting from the trailing foot to the midpoint
 * is some fraction of the transfer duration determined by the transfer split fraction {@link FootstepDataMessage#transfer_split_fraction_}.
 *
 * <p>
 *    This module defines several parameters that allow the weight distribution and split fraction to be modified based on the comparative area and width of the
 *    leading and trailing footholds. That is, we can say that if the upcoming foothold has more area, we should shift our weight to be more on it, and do so
 *    quickly. The same is said for the width - if it has more width, we should shift out weight more and quickly.
 * </p>
 * <p>
 *    This is done by defining the desired weight distribution and split fraction if the upcoming foothold has ALL the area and width. That is, if the trailing
 *    foot has no support area, where the midpoint CoP should be located, and if the trailing foot is purely a line, where the midpoint CoP should be.
 * </p>
 */
public class AreaSplitFractionPostProcessingElement implements FootstepPlanPostProcessingElement
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FootstepPostProcessingParametersReadOnly parameters;
   private final ICPPlannerParameters icpPlannerParameters;
   private final SideDependentList<ConvexPolygon2D> defaultPolygons = new SideDependentList<>();

   public AreaSplitFractionPostProcessingElement(FootstepPostProcessingParametersReadOnly parameters, ICPPlannerParameters icpPlannerParameters,
                                                 SegmentDependentList<RobotSide, ArrayList<Point2D>> defaultContactPointParameters)
   {
      this.parameters = parameters;
      this.icpPlannerParameters = icpPlannerParameters;
      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultPolygon = new ConvexPolygon2D();
         if (defaultContactPointParameters != null)
         {
            for (Point2DReadOnly point : defaultContactPointParameters.get(robotSide))
               defaultPolygon.addVertex(point);
         }
         defaultPolygon.update();
         defaultPolygons.put(robotSide, defaultPolygon);
      }
   }

   /** {@inheritDoc} **/
   @Override
   public boolean isActive()
   {
      return parameters.areaSplitFractionProcessingEnabled();
   }

   /** {@inheritDoc} **/
   @Override
   public FootstepPostProcessingPacket postProcessFootstepPlan(FootstepPostProcessingPacket inputPlan)
   {
      FootstepPostProcessingPacket processedPlan = new FootstepPostProcessingPacket(inputPlan);

      ConvexPolygon2D previousPolygon = new ConvexPolygon2D();
      ConvexPolygon2D currentPolygon = new ConvexPolygon2D();

      PoseReferenceFrame previousFrame = new PoseReferenceFrame("previousFrame", worldFrame);
      PoseReferenceFrame currentFrame = new PoseReferenceFrame("nextFrame", worldFrame);

      List<FootstepDataMessage> footstepDataMessageList = processedPlan.getFootstepDataList().getFootstepDataList();
      if (RobotSide.fromByte(footstepDataMessageList.get(0).getRobotSide()) == RobotSide.LEFT)
      {
         previousFrame.setPositionAndUpdate(new FramePoint3D(worldFrame, inputPlan.getRightFootPositionInWorld()));
         previousFrame.setOrientationAndUpdate(inputPlan.getRightFootOrientationInWorld());

         for (Point3DReadOnly vertex : inputPlan.getRightFootContactPoints2d())
         {
            FramePoint3D vertexInSoleFrame = new FramePoint3D(worldFrame, vertex);
            vertexInSoleFrame.changeFrame(previousFrame);

            previousPolygon.addVertex(vertexInSoleFrame);
         }
         previousPolygon.update();
      }
      else
      {
         previousFrame.setPositionAndUpdate(new FramePoint3D(worldFrame, inputPlan.getLeftFootPositionInWorld()));
         previousFrame.setOrientationAndUpdate(inputPlan.getLeftFootOrientationInWorld());

         for (Point3DReadOnly vertex : inputPlan.getLeftFootContactPoints2d())
         {
            FramePoint3D vertexInSoleFrame = new FramePoint3D(worldFrame, vertex);
            vertexInSoleFrame.changeFrame(previousFrame);

            previousPolygon.addVertex(vertexInSoleFrame);
         }
         previousPolygon.update();
      }

      double defaultTransferSplitFraction = icpPlannerParameters.getTransferSplitFraction();
      double defaultWeightDistribution = 0.5;

      for (int stepNumber = 0; stepNumber < footstepDataMessageList.size(); stepNumber++)
      {
         if (stepNumber > 0)
         {
            previousPolygon.clear();
            FootstepDataMessage previousStep = footstepDataMessageList.get(stepNumber - 1);

            previousFrame.setPositionAndUpdate(new FramePoint3D(worldFrame, previousStep.getLocation()));
            previousFrame.setOrientationAndUpdate(previousStep.getOrientation());

            for (Point3DReadOnly vertex : previousStep.getPredictedContactPoints2d())
            {
               FramePoint3D vertexInSoleFrame = new FramePoint3D(worldFrame, vertex);
               vertexInSoleFrame.changeFrame(previousFrame);

               previousPolygon.addVertex(vertexInSoleFrame);
            }
            previousPolygon.update();
         }

         FootstepDataMessage currentStep = footstepDataMessageList.get(stepNumber);

         currentFrame.setPositionAndUpdate(new FramePoint3D(worldFrame, currentStep.getLocation()));
         currentFrame.setOrientationAndUpdate(currentStep.getOrientation());

         currentPolygon.clear();
         if (currentStep.getPredictedContactPoints2d().size() > 0)
         {
            for (Point3DReadOnly vertex : currentStep.getPredictedContactPoints2d())
            {
               FramePoint3D vertexInSoleFrame = new FramePoint3D(worldFrame, vertex);
               vertexInSoleFrame.changeFrame(currentFrame);

               currentPolygon.addVertex(vertexInSoleFrame);
            }
         }
         else
         {
            currentPolygon.addVertices(defaultPolygons.get(RobotSide.fromByte(currentStep.getRobotSide())));
         }
         currentPolygon.update();

         double currentArea = currentPolygon.getArea();
         double previousArea = previousPolygon.getArea();

         double totalArea = currentArea + previousArea;

         double currentWidth = currentPolygon.getBoundingBoxRangeY();
         double previousWidth = previousPolygon.getBoundingBoxRangeY();

         double totalWidth = currentWidth + previousWidth;

         double percentAreaOnCurrentFoot = totalArea > 0.0 ? currentArea / totalArea : 0.5;
         double percentWidthOnCurrentFoot = totalWidth > 0.0 ? currentWidth / totalWidth : 0.5;

         if (MathTools.epsilonEquals(percentAreaOnCurrentFoot, 0.5, 1.0e-2) && MathTools.epsilonEquals(percentWidthOnCurrentFoot, 0.5, 2.0e-2))
            continue;

         double transferWeightDistributionFromArea = InterpolationTools.linearInterpolate(defaultWeightDistribution, parameters.getFractionLoadIfFootHasFullSupport(),
                                                                                          2.0 * percentAreaOnCurrentFoot - 1.0);
         double transferWeightDistributionFromWidth = InterpolationTools.linearInterpolate(defaultWeightDistribution, parameters.getFractionLoadIfOtherFootHasNoWidth(),
                                                                                           2.0 * percentWidthOnCurrentFoot - 1.0);

         // lower means it spends more time shifting to the center, higher means it spends less time shifting to the center
         // e.g., if we set the fraction to 0 and the trailing foot has no area, the split fraction should be 1 because we spend no time on the first segment
         double transferSplitFractionFromArea = InterpolationTools.linearInterpolate(defaultTransferSplitFraction, 1.0 - parameters.getFractionTimeOnFootIfFootHasFullSupport(),
                                                                                     2.0 * percentAreaOnCurrentFoot - 1.0);
         double transferSplitFractionFromWidth = InterpolationTools.linearInterpolate(defaultTransferSplitFraction, 1.0 - parameters.getFractionTimeOnFootIfOtherFootHasNoWidth(),
                                                                                      2.0 * percentWidthOnCurrentFoot - 1.0);

         double transferWeightDistribution = 0.5 * (transferWeightDistributionFromArea + transferWeightDistributionFromWidth);
         double transferSplitFraction = 0.5 * (transferSplitFractionFromArea + transferSplitFractionFromWidth);

         transferWeightDistribution = MathTools.clamp(transferWeightDistribution, 0.01, 0.99);
         transferSplitFraction = MathTools.clamp(transferSplitFraction, 0.01, 0.99);

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

      return processedPlan;
   }

   /** {@inheritDoc} **/
   @Override
   public PostProcessingEnum getElementName()
   {
      return PostProcessingEnum.AREA_SPLIT_FRACTIONS;
   }
}
