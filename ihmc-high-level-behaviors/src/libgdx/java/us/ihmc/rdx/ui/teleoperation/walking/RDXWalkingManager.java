package us.ihmc.rdx.ui.teleoperation.walking;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersBasics;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.*;
import us.ihmc.rdx.ui.footstepPlanner.RDXFootstepPlanning;
import us.ihmc.rdx.ui.graphics.RDXBodyPathPlanGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.teleoperation.RDXLegControlMode;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXWalkingManager
{
    private RDXBaseUI baseUI;
    private final DRCRobotModel robotModel;
    private final ROS2SyncedRobotModel syncedRobot;
    private final CommunicationHelper communicationHelper;
    private final RDXTeleoperationParameters teleoperationParameters;

    private final RDXFootstepPlanGraphic footstepsSentToControllerGraphic;
    private final RDXBodyPathPlanGraphic bodyPathPlanGraphic = new RDXBodyPathPlanGraphic();

    private final SideDependentList<RDXInteractableFoot> interactableFeet = new SideDependentList<>();
    private final RDXBallAndArrowPosePlacement ballAndArrowMidFeetPosePlacement = new RDXBallAndArrowPosePlacement();
    private final RDXInteractableFootstepPlan interactableFootstepPlan = new RDXInteractableFootstepPlan();
    private final RDXFootstepPlanning footstepPlanning;
    private final RDXManualFootstepPlacement manualFootstepPlacement = new RDXManualFootstepPlacement();
    private final RDXWalkPathControlRing walkPathControlRing = new RDXWalkPathControlRing();
    private RDXLegControlMode legControlMode = RDXLegControlMode.DISABLED;

    private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
    private final ImBoolean showGraphics = new ImBoolean(true);
    private boolean isPlacingFootstep = false;

    public RDXWalkingManager(DRCRobotModel robotModel,
                             CommunicationHelper communicationHelper,
                             ROS2SyncedRobotModel syncedRobot,
                             RDXTeleoperationParameters teleoperationParameters,
                             ROS2ControllerHelper ros2Helper)
    {
        this.communicationHelper = communicationHelper;
        this.robotModel = robotModel;
        this.syncedRobot = syncedRobot;
        this.teleoperationParameters = teleoperationParameters;

        footstepPlanning = new RDXFootstepPlanning(robotModel, teleoperationParameters, syncedRobot);

        // TODO remove ros from this module, and have it call from the higher level.
        ros2Helper.subscribeViaCallback(ROS2Tools.PERSPECTIVE_RAPID_REGIONS, regions ->
        {
            footstepPlanning.setPlanarRegions(regions);
            interactableFootstepPlan.setPlanarRegionsList(regions);
        });
        ros2Helper.subscribeViaCallback(ROS2Tools.HEIGHT_MAP_OUTPUT, heightMap ->
        {
            footstepPlanning.setHeightMapData(heightMap);
            interactableFootstepPlan.setHeightMapMessage(heightMap);
        });

        footstepsSentToControllerGraphic = new RDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
        communicationHelper.subscribeToControllerViaCallback(FootstepDataListMessage.class, footsteps ->
            footstepsSentToControllerGraphic.generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footsteps, "Teleoperation Panel Controller Spy")));
    }


    public void create(RDXBaseUI baseUI)
    {
        this.baseUI = baseUI;

        ballAndArrowMidFeetPosePlacement.create(Color.YELLOW);
        baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(ballAndArrowMidFeetPosePlacement::processImGui3DViewInput);

        interactableFootstepPlan.create(baseUI, communicationHelper, syncedRobot, teleoperationParameters, footstepPlanning.getFootstepPlannerParameters());
        baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(interactableFootstepPlan::processImGui3DViewInput);
        baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(interactableFootstepPlan::calculate3DViewPick);

        manualFootstepPlacement.create(baseUI, interactableFootstepPlan);
        baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(manualFootstepPlacement::processImGui3DViewInput);
        baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(manualFootstepPlacement::calculate3DViewPick);

        walkPathControlRing.create(baseUI.getPrimary3DPanel(), robotModel, syncedRobot, teleoperationParameters);
    }

    public void update()
    {
        footstepsSentToControllerGraphic.update();

        if (ballAndArrowMidFeetPosePlacement.getPlacedNotification().poll())
        {
            footstepPlanning.setMidFeetGoalPose(ballAndArrowMidFeetPosePlacement.getGoalPose());
            footstepPlanning.planAsync();
        }

        // TODO: make footsteps from footstepPlan interactable (modifiable)
        if (footstepPlanning.pollHasNewPlanAvailable()) // failed
        {
            FootstepPlannerOutput output = footstepPlanning.pollOutput();
            interactableFootstepPlan.updateFromPlan(output.getFootstepPlan(), output.getSwingTrajectories());
            if (output.getBodyPath().size() > 0)
                bodyPathPlanGraphic.generateMeshesAsync(output.getBodyPath());
            else
                bodyPathPlanGraphic.clear();
        }

        if (walkPathControlRing.pollIsNewlyModified())
        {
            legControlMode = RDXLegControlMode.PATH_CONTROL_RING;
            interactableFootstepPlan.clear();
            bodyPathPlanGraphic.clear();
        }

        if (manualFootstepPlacement.pollIsModeNewlyActivated())
        {
            legControlMode = RDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT;
        }

        if (legControlMode != RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING)
        {
            for (RobotSide side : interactableFeet.sides())
            {
                interactableFeet.get(side).delete();
            }
        }

        if (legControlMode != RDXLegControlMode.PATH_CONTROL_RING)
        {
            walkPathControlRing.delete();
        }

        if (legControlMode != RDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT)
        {
            manualFootstepPlacement.exitPlacement();
        }

        if (legControlMode == RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING || legControlMode == RDXLegControlMode.DISABLED)
        {
            interactableFootstepPlan.clear();
            bodyPathPlanGraphic.clear();
        }

        manualFootstepPlacement.update();
        bodyPathPlanGraphic.update();
        interactableFootstepPlan.update();

        if (interactableFootstepPlan.getNumberOfFootsteps() > 0)
        {
            footstepPlanning.setReadyToWalk(false);
            footstepsSentToControllerGraphic.clear();
        }

        footstepsSentToControllerGraphic.update();

        boolean isCurrentlyPlacingFootstep = getManualFootstepPlacement().isPlacingFootstep() || ballAndArrowMidFeetPosePlacement.isPlacingGoal();
        if (isPlacingFootstep != isCurrentlyPlacingFootstep)
            baseUI.setModelSceneMouseCollisionEnabled(isCurrentlyPlacingFootstep);
        isPlacingFootstep = isCurrentlyPlacingFootstep;
    }

    public void renderImGuiWidgets()
    {
        ImGui.separator();

        ImGui.text("Leg control mode: " + legControlMode.name());
        if (ImGui.radioButton(labels.get("Disabled"), legControlMode == RDXLegControlMode.DISABLED))
        {
            legControlMode = RDXLegControlMode.DISABLED;
        }
        ImGui.sameLine();
        if (ImGui.radioButton(labels.get("Manual foostep placement"), legControlMode == RDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT))
        {
            legControlMode = RDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT;
        }
        if (ImGui.radioButton(labels.get("Path control ring"), legControlMode == RDXLegControlMode.PATH_CONTROL_RING))
        {
            legControlMode = RDXLegControlMode.PATH_CONTROL_RING;
            walkPathControlRing.becomeModified(true);
        }
        ImGui.sameLine();
        if (ImGui.radioButton(labels.get("Single support foot posing"), legControlMode == RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING))
        {
            legControlMode = RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING;
        }

        manualFootstepPlacement.renderImGuiWidgets();

        if (ballAndArrowMidFeetPosePlacement.renderPlaceGoalButton())
            legControlMode = RDXLegControlMode.PATH_CONTROL_RING;

        ImGui.text("Walk path control ring planner:");
        walkPathControlRing.renderImGuiWidgets();

        interactableFootstepPlan.renderImGuiWidgets();
        ImGui.separator();
        ImGui.checkbox(labels.get("Show footstep related graphics"), showGraphics);
        ImGui.separator();
    }

    public void updateWalkPathControlRing()
    {
        walkPathControlRing.update(interactableFootstepPlan);
    }

    public void caculateWalkPathControlRing3DViewPick(ImGui3DViewInput input)
    {
        if (!manualFootstepPlacement.isPlacingFootstep())
            walkPathControlRing.calculate3DViewPick(input);
    }

    public void processWalkPathControlRing3dViewInput(ImGui3DViewInput input)
    {
        if (!manualFootstepPlacement.isPlacingFootstep())
            walkPathControlRing.process3DViewInput(input);
    }

    public void destroy()
    {
        walkPathControlRing.destroy();
        footstepsSentToControllerGraphic.destroy();
        bodyPathPlanGraphic.destroy();
        interactableFootstepPlan.destroy();
    }

    public void deleteAll()
    {
        footstepsSentToControllerGraphic.clear();
        ballAndArrowMidFeetPosePlacement.clear();
        manualFootstepPlacement.exitPlacement();
        interactableFootstepPlan.clear();
        bodyPathPlanGraphic.clear();
        walkPathControlRing.delete();

        legControlMode = RDXLegControlMode.DISABLED;
    }

    public void setFootstepPlannerParameters(FootstepPlannerParametersBasics footstepPlannerParameters)
    {
        footstepPlanning.setFootstepPlannerParameters(footstepPlannerParameters);
    }

    public void setBodyPathPlannerParameters(AStarBodyPathPlannerParametersBasics bodyPathPlannerParameters)
    {
        footstepPlanning.setBodyPathPlannerParameters(bodyPathPlannerParameters);
    }

    public void setSwingParameters(SwingPlannerParametersBasics swingParameters)
    {
        interactableFootstepPlan.setSwingPlannerParameters(swingParameters);
        footstepPlanning.setSwingFootPlannerParameters(swingParameters);
    }

    public void setLegControlModeToSingleSupportFootPosing()
    {
        legControlMode = RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING;
    }

    public RDXManualFootstepPlacement getManualFootstepPlacement()
    {
        return manualFootstepPlacement;
    }

    public void getWalkPathControlRingVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
    {
        walkPathControlRing.getVirtualRenderables(renderables, pool);
    }

    public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
    {
        footstepsSentToControllerGraphic.getRenderables(renderables, pool);
        ballAndArrowMidFeetPosePlacement.getRenderables(renderables, pool);
        manualFootstepPlacement.getRenderables(renderables, pool);
        interactableFootstepPlan.getRenderables(renderables, pool);
        bodyPathPlanGraphic.getRenderables(renderables, pool);
    }
}
