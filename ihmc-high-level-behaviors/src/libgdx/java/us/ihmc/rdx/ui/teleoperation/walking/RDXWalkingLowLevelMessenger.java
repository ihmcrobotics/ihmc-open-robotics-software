package us.ihmc.rdx.ui.teleoperation.walking;

import imgui.ImGui;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;

public class RDXWalkingLowLevelMessenger
{
    private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

    private final CommunicationHelper communicationHelper;
    private final RDXTeleoperationParameters teleoperationParameters;
    private final RobotLowLevelMessenger robotLowLevelMessenger;

    public RDXWalkingLowLevelMessenger(CommunicationHelper communicationHelper, RDXTeleoperationParameters teleoperationParameters)
    {
        this.communicationHelper = communicationHelper;
        this.teleoperationParameters = teleoperationParameters;
        robotLowLevelMessenger = communicationHelper.getOrCreateRobotLowLevelMessenger();

        if (robotLowLevelMessenger == null)
        {
            String robotName = communicationHelper.getRobotModel().getSimpleRobotName();
            throw new RuntimeException("Please add implementation of RobotLowLevelMessenger for " + robotName);
        }
    }

    public void renderImGuiWidgets()
    {
        if (ImGui.button(labels.get("Pause")))
        {
            sendPauseWalkingRequest();
        }
        ImGui.sameLine();
        if (ImGui.button(labels.get("Continue")))
        {
            sendContinueWalkingRequest();
        }
    }

    public void sendPauseWalkingRequest()
    {
        robotLowLevelMessenger.sendPauseWalkingRequest();
    }

    public void sendContinueWalkingRequest()
    {
        robotLowLevelMessenger.sendContinueWalkingRequest();
    }
}
