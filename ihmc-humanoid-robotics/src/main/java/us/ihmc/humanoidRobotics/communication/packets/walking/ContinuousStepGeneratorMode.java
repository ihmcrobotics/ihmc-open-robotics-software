package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

public enum ContinuousStepGeneratorMode
{
    @RosEnumValueDocumentation(documentation = "Footstep positions will be determined by step length required to achieve desired walking speed and swing duration.")
    STANDARD,
    @RosEnumValueDocumentation(documentation = "Footstep positions will be determined by Raibert heuristic-style control law, where gain is calculated via pole placement.")
    QFP;

    public static final ContinuousStepGeneratorMode[] values = values();

    public byte toByte()
    {
        return (byte) ordinal();
    }

    public static ContinuousStepGeneratorMode fromByte(byte enumAsByte)
    {
        return values[enumAsByte];
    }
}
