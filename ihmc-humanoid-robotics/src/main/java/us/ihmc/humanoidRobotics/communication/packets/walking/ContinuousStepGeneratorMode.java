package us.ihmc.humanoidRobotics.communication.packets.walking;

public enum ContinuousStepGeneratorMode
{
   STANDARD,
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
