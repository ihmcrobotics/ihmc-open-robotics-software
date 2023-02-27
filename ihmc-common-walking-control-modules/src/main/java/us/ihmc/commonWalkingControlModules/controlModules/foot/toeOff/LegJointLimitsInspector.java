package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
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

    private final BooleanProvider doToeOffWhenHittingAnkleLimit;
    private final BooleanProvider doToeOffWhenHittingLeadingKneeUpperLimit;
    private final BooleanProvider doToeOffWhenHittingRearKneeLowerLimit;

    private final YoBoolean needToSwitchToToeOffForJointLimit = new YoBoolean("needToSwitchToToeOffForJointLimit", registry);
    private final YoBoolean needToSwitchToToeOffForAnkleLimit = new YoBoolean("needToSwitchToToeOffForAnkleLimit", registry);
    private final YoBoolean needToSwitchToToeOffForLeadingKneeAtLimit = new YoBoolean("needToSwitchToToeOffForLeadingKneeAtLimit", registry);
    private final YoBoolean needToSwitchToToeOffForTrailingKneeAtLimit = new YoBoolean("needToSwitchToToeOffForTrailingKneeAtLimit", registry);

    private final FullHumanoidRobotModel fullRobotModel;

    public LegJointLimitsInspector(ToeOffParameters toeOffParameters, FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry)
    {
        this.fullRobotModel = fullRobotModel;

        ankleLowerLimitToTriggerToeOff = new DoubleParameter("ankleLowerLimitToTriggerToeOff", registry, toeOffParameters.getAnkleLowerLimitToTriggerToeOff());
        kneeUpperLimitToTriggerToeOff = new DoubleParameter("kneeUpperLimitToTriggerToeOff", registry, toeOffParameters.getKneeUpperLimitToTriggerToeOff());
        kneeLowerLimitToTriggerToeOff = new DoubleParameter("kneeLowerLimitToTriggerToeOff", registry, toeOffParameters.getKneeLowerLimitToTriggerToeOff());

        doToeOffWhenHittingAnkleLimit = new BooleanParameter("doToeOffWhenHittingAnkleLimit", registry, toeOffParameters.doToeOffWhenHittingAnkleLimit());
        doToeOffWhenHittingLeadingKneeUpperLimit = new BooleanParameter("doToeOffWhenHittingLeadingKneeUpperLimit", registry, toeOffParameters.doToeOffWhenHittingLeadingKneeUpperLimit());
        doToeOffWhenHittingRearKneeLowerLimit = new BooleanParameter("doToeOffWhenHittingRearKneeLowerLimit", registry, toeOffParameters.doToeOffWhenHittingTrailingKneeLowerLimit());

        needToSwitchToToeOffForJointLimit.set(false);
        parentRegistry.addChild(registry);
    }

    public void reset()
    {
        isRearAnklePitchHittingLimit.set(false);
        isRearAnklePitchHittingLimitFilt.set(false);
        isLeadingKneePitchHittingUpperLimit.set(false);
        isLeadingKneePitchHittingUpperLimitFilt.set(false);
        isRearKneePitchHittingLowerLimit.set(false);
        isRearKneePitchHittingLowerLimitFilt.set(false);

        needToSwitchToToeOffForJointLimit.set(false);
        needToSwitchToToeOffForTrailingKneeAtLimit.set(false);
        needToSwitchToToeOffForLeadingKneeAtLimit.set(false);
        needToSwitchToToeOffForAnkleLimit.set(false);
    }

    public void updateToeOffConditions(RobotSide trailingLeg)
    {
        OneDoFJointBasics trailingAnklePitch = fullRobotModel.getLegJoint(trailingLeg, LegJointName.ANKLE_PITCH);
        OneDoFJointBasics leadingKneePitch = fullRobotModel.getLegJoint(trailingLeg.getOppositeSide(), LegJointName.KNEE_PITCH);
        OneDoFJointBasics trailingKneePitch = fullRobotModel.getLegJoint(trailingLeg, LegJointName.KNEE_PITCH);

        updateTrailingAnkleLowerLimitsStatus(trailingAnklePitch);
        updateLeadingKneeUpperLimitsStatus(leadingKneePitch);
        updateTrailingKneeLowerLimitsStatus(trailingKneePitch);
        updateSwitchToToeOffDueToJointLimits();
    }

    public void updateTrailingAnkleLowerLimitsStatus(OneDoFJointReadOnly anklePitch)
    {
        ankleMaxLowerLimitForToeOff.set(Math.max(anklePitch.getJointLimitLower() + jointLimitThreshold, ankleLowerLimitToTriggerToeOff.getValue()));
        isRearAnklePitchHittingLimit.set(anklePitch.getQ() < ankleMaxLowerLimitForToeOff.getDoubleValue());
        isRearAnklePitchHittingLimitFilt.update();

        // check if we need to switch to toe off due to ankle limits
        if (!doToeOffWhenHittingAnkleLimit.getValue())
            needToSwitchToToeOffForAnkleLimit.set(false);
        else
            needToSwitchToToeOffForAnkleLimit.set(isRearAnklePitchHittingLimitFilt.getBooleanValue());
    }

    public void updateLeadingKneeUpperLimitsStatus(OneDoFJointReadOnly kneePitch)
    {
        kneeMinUpperLimitToTriggerToeOff.set(Math.min(kneePitch.getJointLimitUpper() - jointLimitThreshold, kneeUpperLimitToTriggerToeOff.getValue()));
        isLeadingKneePitchHittingUpperLimit.set(kneePitch.getQ() > kneeMinUpperLimitToTriggerToeOff.getDoubleValue());
        isLeadingKneePitchHittingUpperLimitFilt.update();

        // check if we need to switch to toe off due to leading knee limits
        if (!doToeOffWhenHittingLeadingKneeUpperLimit.getValue())
            needToSwitchToToeOffForLeadingKneeAtLimit.set(false);
        else
            needToSwitchToToeOffForLeadingKneeAtLimit.set(isLeadingKneePitchHittingUpperLimitFilt.getBooleanValue());
    }

    public void updateTrailingKneeLowerLimitsStatus(OneDoFJointReadOnly kneePitch)
    {
        kneeMaxLowerLimitToTriggerToeOff.set(Math.max(kneePitch.getJointLimitLower() + jointLimitThreshold, kneeLowerLimitToTriggerToeOff.getValue()));
        isRearKneePitchHittingLowerLimit.set(kneePitch.getQ() < kneeMaxLowerLimitToTriggerToeOff.getDoubleValue());
        isRearKneePitchHittingLowerLimitFilt.update();

        // check if we need to switch to toe off due to trailing knee limits
        if (!doToeOffWhenHittingRearKneeLowerLimit.getValue())
            needToSwitchToToeOffForTrailingKneeAtLimit.set(false);
        else
            needToSwitchToToeOffForTrailingKneeAtLimit.set(isRearKneePitchHittingLowerLimitFilt.getBooleanValue());
    }

    public void updateSwitchToToeOffDueToJointLimits()
    {
        needToSwitchToToeOffForJointLimit.set(needToSwitchToToeOffForAnkleLimit.getBooleanValue() ||
                needToSwitchToToeOffForLeadingKneeAtLimit.getBooleanValue() ||
                needToSwitchToToeOffForTrailingKneeAtLimit.getBooleanValue());
    }

    public boolean needToSwitchToToeOffDueToJointLimit()
    {
        return needToSwitchToToeOffForJointLimit.getBooleanValue();
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
