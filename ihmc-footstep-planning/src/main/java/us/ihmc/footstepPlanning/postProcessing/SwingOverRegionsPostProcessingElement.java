package us.ihmc.footstepPlanning.postProcessing;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.FootstepPostProcessingPacket;
import us.ihmc.commons.Conversions;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersReadOnly;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.List;

/**
 * This class is a post-processing elements that bolts on top of the {@link SwingOverPlanarRegionsTrajectoryExpander} to allow it to take in planar regions,
 * and then modify the two midpoints so that the swing trajectory is collision free.
 */
public class SwingOverRegionsPostProcessingElement implements FootstepPlanPostProcessingElement
{
   private final FootstepPostProcessingParametersReadOnly parameters;
   private final SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;

   public SwingOverRegionsPostProcessingElement(FootstepPostProcessingParametersReadOnly parameters, WalkingControllerParameters walkingControllerParameters,
                                                YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.parameters = parameters;

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      swingOverPlanarRegionsTrajectoryExpander = new SwingOverPlanarRegionsTrajectoryExpander(walkingControllerParameters, registry, yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   /** {@inheritDoc} **/
   @Override
   public boolean isActive()
   {
      return parameters.swingOverRegionsProcessingEnabled();
   }

   /** {@inheritDoc} **/
   @Override
   public FootstepPostProcessingPacket postProcessFootstepPlan(FootstepPostProcessingPacket outputPlan)
   {
      swingOverPlanarRegionsTrajectoryExpander.setDoInitialFastApproximation(parameters.getDoInitialFastApproximation());
      swingOverPlanarRegionsTrajectoryExpander.setNumberOfCheckpoints(parameters.getNumberOfChecksPerSwing());
      swingOverPlanarRegionsTrajectoryExpander.setMaximumNumberOfTries(parameters.getMaximumNumberOfAdjustmentAttempts());
      swingOverPlanarRegionsTrajectoryExpander.setMinimumSwingFootClearance(parameters.getMinimumSwingFootClearance());
      swingOverPlanarRegionsTrajectoryExpander.setIncrementalAdjustmentDistance(parameters.getIncrementalWaypointAdjustmentDistance());
      swingOverPlanarRegionsTrajectoryExpander.setMaximumAdjustmentDistance(parameters.getMaximumWaypointAdjustmentDistance());
      swingOverPlanarRegionsTrajectoryExpander.setMinimumHeightAboveFloorForCollision(parameters.getMinimumHeightAboveFloorForCollision());

      FootstepPostProcessingPacket processedPlan = new FootstepPostProcessingPacket(outputPlan);

      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(outputPlan.getPlanarRegionsList());

      RobotSide stanceSide = RobotSide.fromByte(outputPlan.getFootstepDataList().getFootstepDataList().get(0).getRobotSide()).getOppositeSide();

      FramePose3D leftFootPose = new FramePose3D();
      leftFootPose.setPosition(outputPlan.getLeftFootPositionInWorld());
      leftFootPose.setOrientation(outputPlan.getLeftFootOrientationInWorld());

      FramePose3D rightFootPose = new FramePose3D();
      rightFootPose.setPosition(outputPlan.getRightFootPositionInWorld());
      rightFootPose.setOrientation(outputPlan.getRightFootOrientationInWorld());

      SideDependentList<FramePose3D> footPoses = new SideDependentList<>();
      footPoses.put(RobotSide.LEFT, leftFootPose);
      footPoses.put(RobotSide.RIGHT, rightFootPose);

      FramePose3D stanceFootPose = new FramePose3D(footPoses.get(stanceSide));

      List<FootstepDataMessage> footstepDataMessageList = processedPlan.getFootstepDataList().getFootstepDataList();
      for (int stepNumber = 0; stepNumber < footstepDataMessageList.size(); stepNumber++)
      {
         FramePose3D nextFootPose = new FramePose3D();

         if (stepNumber > 0)
         {
            stanceFootPose.setPosition(footstepDataMessageList.get(stepNumber - 1).getLocation());
            stanceFootPose.setOrientation(footstepDataMessageList.get(stepNumber - 1).getOrientation());
         }

         nextFootPose.setPosition(footstepDataMessageList.get(stepNumber).getLocation());
         nextFootPose.setOrientation(footstepDataMessageList.get(stepNumber).getOrientation());
         RobotSide side = RobotSide.fromByte(footstepDataMessageList.get(stepNumber).getRobotSide());

         double maxSpeedDimensionless = swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose, footPoses.get(side),
                                                                                                                   nextFootPose, planarRegionsList);
         if (swingOverPlanarRegionsTrajectoryExpander.wereWaypointsAdjusted())
         {
            FootstepDataMessage footstepData = footstepDataMessageList.get(stepNumber);

            footstepData.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
            Point3D waypointOne = new Point3D(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(0));
            Point3D waypointTwo = new Point3D(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(1));
            MessageTools.copyData(new Point3D[] {waypointOne, waypointTwo}, footstepData.getCustomPositionWaypoints());
         }

         footPoses.put(side, nextFootPose);
      }

      return processedPlan;
   }

   /** {@inheritDoc} **/
   @Override
   public PostProcessingEnum getElementName()
   {
      return PostProcessingEnum.SWING_OVER_REGIONS;
   }
}
