package us.ihmc.rdx.ui.vr;

import us.ihmc.log.LogTools;

public class RDXVRPelvisControlState
{
    public static final double UPPER_LIMIT_PELVIS_PITCH = 0.30;
    public static final double UPPER_LIMIT_PELVIS_HEIGHT = 1.051;
    public static final double LOWER_LIMIT_PELVIS_HEIGHT = 0.65;
    public static final double LOWER_LIMIT_PELVIS_HEIGHT_WITH_BACK_BENDING = 0.84;
    public static final double VR_INPUT_PELVIS_CHANGE_RATIO = 0.005;

    private enum ControlState
    {
        HEIGHT, PITCH;
    }
    private ControlState state = ControlState.HEIGHT;

    public void switchToHeightControl()
    {
        state = ControlState.HEIGHT;
    }

    public void switchToPitchControl()
    {
        state = ControlState.PITCH;
    }

    public boolean isHeightControlled()
    {
        return state == ControlState.HEIGHT;
    }

    public boolean isPitchControlled()
    {
        return state == ControlState.PITCH;
    }
}
