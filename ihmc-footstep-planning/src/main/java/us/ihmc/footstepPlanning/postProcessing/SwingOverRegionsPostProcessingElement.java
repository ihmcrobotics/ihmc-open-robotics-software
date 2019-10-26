package us.ihmc.footstepPlanning.postProcessing;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersReadOnly;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.List;

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
   public FootstepPlanningToolboxOutputStatus postProcessFootstepPlan(FootstepPlanningRequestPacket request, FootstepPlanningToolboxOutputStatus outputStatus)
   {
      swingOverPlanarRegionsTrajectoryExpander.setNumberOfCheckpoints(parameters.getNumberOfChecksPerSwing());
      swingOverPlanarRegionsTrajectoryExpander.setMaximumNumberOfTries(parameters.getMaximumNumberOfAdjustmentAttempts());
      swingOverPlanarRegionsTrajectoryExpander.setMinimumSwingFootClearance(parameters.getMinimumSwingFootClearance());
      swingOverPlanarRegionsTrajectoryExpander.setIncrementalAdjustmentDistance(parameters.getIncrementalWaypointAdjustmentDistance());
      swingOverPlanarRegionsTrajectoryExpander.setMaximumAdjustmentDistance(parameters.getMaximumWaypointAdjustmentDistance());

      FootstepPlanningToolboxOutputStatus processedOutput = new FootstepPlanningToolboxOutputStatus(outputStatus);

      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(outputStatus.getPlanarRegionsList());

      RobotSide stanceSide = RobotSide.fromByte(request.getInitialStanceRobotSide());
      FramePose3D initialStanceFootPose = new FramePose3D();
      initialStanceFootPose.setPosition(request.getStanceFootPositionInWorld());
      initialStanceFootPose.setOrientation(request.getStanceFootOrientationInWorld());

      PoseReferenceFrame stanceReferenceFrame = new PoseReferenceFrame("stanceReferenceFrame", ReferenceFrame.getWorldFrame());
      stanceReferenceFrame.setPoseAndUpdate(initialStanceFootPose);

      FramePose3D otherFootPose = new FramePose3D(stanceReferenceFrame);
      otherFootPose.setY(stanceSide.negateIfLeftSide(0.25));
      otherFootPose.changeFrame(ReferenceFrame.getWorldFrame());


      SideDependentList<FramePose3D> footPoses = new SideDependentList<>();
      footPoses.put(stanceSide, initialStanceFootPose);
      footPoses.put(stanceSide.getOppositeSide(), otherFootPose);

      FramePose3D stanceFootPose = new FramePose3D(initialStanceFootPose);

      List<FootstepDataMessage> footstepDataMessageList = processedOutput.getFootstepDataList().getFootstepDataList();
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

         FootstepDataMessage footstepData = footstepDataMessageList.get(stepNumber);

         footstepData.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         Point3D waypointOne = new Point3D(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(0));
         Point3D waypointTwo = new Point3D(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(1));
         MessageTools.copyData(new Point3D[] {waypointOne, waypointTwo}, footstepData.getCustomPositionWaypoints());

         footPoses.put(side, nextFootPose);
      }

      return processedOutput;
   }

   /** {@inheritDoc} **/
   @Override
   public PostProcessingEnum getElementName()
   {
      return PostProcessingEnum.SWING_OVER_REGIONS;
   }
}
