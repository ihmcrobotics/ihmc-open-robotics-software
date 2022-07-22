package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepPoseHeuristicChecker;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;

public class SimpleStepChecker
{
    private CommunicationHelper communicationHelper;

    // TODO: need to pass in FootstepPlannerParametersReadOnly , FootstepSnapAndWiggler, YoRegistry (parent)
    private final double reachabilityThreshold = 2.2;
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
    private final SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
    private final DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
    private final FootstepSnapAndWiggler snapper;
    private final FootstepPoseHeuristicChecker stepChecker;

    private BipedalFootstepPlannerNodeRejectionReason prevReason = null;
    private BipedalFootstepPlannerNodeRejectionReason reason = null;


    // TODO: swap stance and swing if candidate step for the very first step of the footsteparraylist is going to be on different side compared to swing's side.
    private DiscreteFootstep candidate;
    private DiscreteFootstep stance = new DiscreteFootstep(0,0,0, RobotSide.RIGHT);
    private DiscreteFootstep swing = new DiscreteFootstep(0,0,0,RobotSide.LEFT);
    private FootstepPlannerParametersBasics footstepPlannerParameters;

    private String text = null;

    GDXImGuiBasedUI baseUI;
    private ImGui3DViewInput latestInput;
    private GDX3DPanel primary3DPanel;
    boolean renderTooltip = false;
    private boolean isFirstStep = false;
    private boolean placingGoal = false;

    // CONSTRUCTOR
    public SimpleStepChecker(GDXImGuiBasedUI baseUI, CommunicationHelper communicationHelper, ROS2SyncedRobotModel syncedRobot)
    {
        this.baseUI = baseUI;
        footstepPlannerParameters = communicationHelper.getRobotModel().getFootstepPlannerParameters();
        snapper = new FootstepSnapAndWiggler(footPolygons, footstepPlannerParameters);
        stepChecker = new FootstepPoseHeuristicChecker(footstepPlannerParameters, snapper, registry);
        primary3DPanel = baseUI.getPrimary3DPanel();
        primary3DPanel.addWindowDrawListAddition(this::renderTooltips);

        // Set initial feet.
        RigidBodyTransform initialRightFootTransform = syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame();
        RigidBodyTransform initialLeftFootTransform = syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame();

        Vector3DBasics initialRightFoot =  initialRightFootTransform.getTranslation();
        Vector3DBasics initialLeftFoot =  initialLeftFootTransform.getTranslation();

        stance = new DiscreteFootstep(initialRightFoot.getX(), initialRightFoot.getY(), initialRightFootTransform.getRotation().getYaw(), RobotSide.RIGHT);
        swing = new DiscreteFootstep(initialLeftFoot.getX(), initialLeftFoot.getY(), initialLeftFootTransform.getRotation().getYaw(), RobotSide.LEFT);
    }

    public void getInput(ImGui3DViewInput input , boolean placingGoal)
    {
        latestInput = input;
        this.placingGoal = placingGoal;
    }

    private void renderTooltips()
    {
        if (placingGoal && this.latestInput!=null && renderTooltip && text!=null)
        {
            float offsetX = 10.0f;
            float offsetY = 10.0f;
            float mousePosX = latestInput.getMousePosX();
            float mousePosY = latestInput.getMousePosY();
            float drawStartX = primary3DPanel.getWindowDrawMinX() + mousePosX + offsetX;
            float drawStartY = primary3DPanel.getWindowDrawMinY() + mousePosY + offsetY;

            ImGui.getWindowDrawList().addRectFilled(drawStartX , drawStartY, drawStartX + text.length()*12, drawStartY + 21.0f, new Color(0.2f, 0.2f, 0.2f, 0.7f).toIntBits());


            ImGui.getWindowDrawList()
                    .addText(ImGuiTools.getSmallFont(),
                            ImGuiTools.getSmallFont().getFontSize(),
                            drawStartX + 5.0f,
                            drawStartY + 2.0f,
                            Color.WHITE.toIntBits(),
                            text);
        }
    }

    // This should update candidate, stance, and swing in the ImGuiGDXManualFootstepPlacement.
    public void update(ArrayList<SingleFootstep> stepList, DiscreteFootstep candidate, boolean placingGoal)
    {
        this.candidate = candidate;
        int size = stepList.size();
        if(stepList.size()>1)
        {
            swing = convertToDiscrete(stepList.get(size-2));
            stance = convertToDiscrete(stepList.get(size-1));
        }

        if (candidate.getRobotSide()!=swing.getRobotSide())
        {
            swapSteps();
        }

        if(placingGoal) renderTooltip = true;
        else renderTooltip = false;
    }

    // TODO: updates RejectionReason, and generate warning message in the UI screen.
    public void checkValidStep()
    {
        reason = stepChecker.checkStepValidity(candidate,stance,swing);
        makeWarning();
    }

    // TODO: This should be used when first step of the manual step cycle has different RobotSide than current swing.
    public void swapSteps()
    {
        DiscreteFootstep temp = stance;
        stance = swing;
        swing = temp;
    }
    public DiscreteFootstep convertToDiscrete(SingleFootstep step)
    {
        Pose3DReadOnly pose = step.getPose();
        Point3DReadOnly position = pose.getPosition();
        return new DiscreteFootstep(position.getX(), position.getY(), pose.getYaw(), step.getFootstepSide());
    }

    public BipedalFootstepPlannerNodeRejectionReason getReason()
    {
        return reason;
    }

    public void makeWarning()
    {
        if(reason==null)
        {
            text = new String(" Looks Good");
        }
        else
        {
            text = new String(" Warning! " + reason.name());
        }
    }
}
