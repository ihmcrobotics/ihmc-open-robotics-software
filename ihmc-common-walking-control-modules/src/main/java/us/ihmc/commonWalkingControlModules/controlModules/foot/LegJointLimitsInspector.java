package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class LegJointLimitsInspector
{
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
    private static final int largeGlitchWindowSize = 10;
    private static final double jointLimitThreshold = 0.02;

    private final DoubleProvider ankleLowerLimitToTriggerToeOff;
    private final DoubleProvider kneeUpperLimitToTriggerToeOff;
    private final DoubleProvider kneeLowerLimitToTriggerToeOff;
    private final YoDouble ankleMaxLowerLimitForToeOff = new YoDouble("ankleMaxLowerLimitToTriggerToeOff", registry);
    private final YoDouble kneeMinUpperLimitToTriggerToeOff = new YoDouble("kneeMinUpperLimitToTriggerToeOff", registry);
    private final YoDouble kneeMaxLowerLimitToTriggerToeOff = new YoDouble("kneeMaxLowerLimitToTriggerToeOff", registry);

    private final YoBoolean isRearAnklePitchHittingLimit = new YoBoolean("isRearAnklePitchHittingLimit", registry);
    private final YoBoolean isLeadingKneePitchHittingUpperLimit = new YoBoolean("isLeadingKneePitchHittingUpperLimit", registry);
    private final YoBoolean isRearKneePitchHittingLowerLimit = new YoBoolean("isRearKneePitchHittingLowerLimit", registry);

    private final GlitchFilteredYoBoolean isRearAnklePitchHittingLimitFilt = new GlitchFilteredYoBoolean("isRearAnklePitchHittingLimitFilt", registry,
            isRearAnklePitchHittingLimit, largeGlitchWindowSize);
    private final GlitchFilteredYoBoolean isLeadingKneePitchHittingUpperLimitFilt = new GlitchFilteredYoBoolean("isLeadingKneePitchHittingUpperLimitFilt",
            registry, isLeadingKneePitchHittingUpperLimit,
            largeGlitchWindowSize);
    private final GlitchFilteredYoBoolean isRearKneePitchHittingLowerLimitFilt = new GlitchFilteredYoBoolean("isRearKneePitchHittingLowerLimitFilt", registry,
            isRearKneePitchHittingLowerLimit,
            largeGlitchWindowSize);

    public LegJointLimitsInspector(ToeOffParameters toeOffParameters, YoRegistry parentRegistry)
    {
        ankleLowerLimitToTriggerToeOff = new DoubleParameter("ankleLowerLimitToTriggerToeOff", registry, toeOffParameters.getAnkleLowerLimitToTriggerToeOff());
        kneeUpperLimitToTriggerToeOff = new DoubleParameter("kneeUpperLimitToTriggerToeOff", registry, toeOffParameters.getKneeUpperLimitToTriggerToeOff());
        kneeLowerLimitToTriggerToeOff = new DoubleParameter("kneeLowerLimitToTriggerToeOff", registry, toeOffParameters.getKneeLowerLimitToTriggerToeOff());
        parentRegistry.addChild(registry);
    }

    public void reset()
    {
        isRearAnklePitchHittingLimit.set(false);
        isRearAnklePitchHittingLimitFilt.set(false);
    }

    public void updateTrailingAnkleLowerLimitsStatus(OneDoFJointBasics anklePitch)
    {
        ankleMaxLowerLimitForToeOff.set(Math.max(anklePitch.getJointLimitLower() + jointLimitThreshold, ankleLowerLimitToTriggerToeOff.getValue()));
        isRearAnklePitchHittingLimit.set(anklePitch.getQ() < ankleMaxLowerLimitForToeOff.getDoubleValue());
        isRearAnklePitchHittingLimitFilt.update();
    }

    public void updateLeadingKneeUpperLimitsStatus(OneDoFJointBasics kneePitch)
    {
        kneeMinUpperLimitToTriggerToeOff.set(Math.min(kneePitch.getJointLimitUpper() - jointLimitThreshold, kneeUpperLimitToTriggerToeOff.getValue()));
        isLeadingKneePitchHittingUpperLimit.set(kneePitch.getQ() > kneeMinUpperLimitToTriggerToeOff.getDoubleValue());
        isLeadingKneePitchHittingUpperLimitFilt.update();
    }

    public void updateTrailingKneeLowerLimitsStatus(OneDoFJointBasics kneePitch)
    {
        kneeMaxLowerLimitToTriggerToeOff.set(Math.max(kneePitch.getJointLimitLower() + jointLimitThreshold, kneeLowerLimitToTriggerToeOff.getValue()));
        isRearKneePitchHittingLowerLimit.set(kneePitch.getQ() < kneeMaxLowerLimitToTriggerToeOff.getDoubleValue());
        isRearKneePitchHittingLowerLimitFilt.update();
    }

    public boolean isRearAnklePitchHittingLowerLimitFilt()
    {
        return isRearAnklePitchHittingLimitFilt.getBooleanValue();
    }

    public boolean isKneePitchHittingUpperLimitFilt()
    {
        return isLeadingKneePitchHittingUpperLimitFilt.getBooleanValue();
    }

    public boolean isKneePitchHittingLowerLimitFilt()
    {
        return isRearKneePitchHittingLowerLimitFilt.getBooleanValue();
    }

    public double getAnkleMaxLowerLimitForToeOff()
    {
        return ankleMaxLowerLimitForToeOff.getDoubleValue();
    }

    public double getKneeMinUpperLimitToTriggerToeOff()
    {
        return kneeMinUpperLimitToTriggerToeOff.getDoubleValue();
    }

    public double getKneeMaxLowerLimitToTriggerToeOff()
    {
        return kneeMaxLowerLimitToTriggerToeOff.getDoubleValue();
    }
}
