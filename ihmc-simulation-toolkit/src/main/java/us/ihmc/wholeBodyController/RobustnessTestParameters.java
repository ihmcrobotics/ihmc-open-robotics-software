package us.ihmc.wholeBodyController;

import us.ihmc.euclid.tuple3D.Vector3D;

public abstract class RobustnessTestParameters
{
    // Record for test configuration
    public record RobustnessTestConfig(double minForce, double maxForce, double minDuration,
                                       double maxDuration, double pushInterval, Vector3D pushDirection, Vector3D pushOffset) {}

    // Abstract methods - robot-specific configuration
    public abstract RobustnessTestConfig getTestConfiguration();
    public abstract double getLightPushForce();
    public abstract double getLightPushDuration();
    public abstract double getHeavyPushForce();
    public abstract double getHeavyPushDuration();
    public abstract int getNumberOfTestLevels();

    // Concrete methods - generic logic
    public double getMinForce() { return getTestConfiguration().minForce(); }
    public double getMaxForce() { return getTestConfiguration().maxForce(); }
    public double getMinDuration() { return getTestConfiguration().minDuration(); }
    public double getMaxDuration() { return getTestConfiguration().maxDuration(); }
    public double getPushInterval() { return getTestConfiguration().pushInterval(); }
    public Vector3D getPushDirection() { return getTestConfiguration().pushDirection(); }
    public Vector3D getDefaultPushDirection() { return getTestConfiguration().pushDirection(); }
    public Vector3D getPushOffset() { return getTestConfiguration().pushOffset(); }

    /**
     * Calculates progressive test configuration for a specific level
     */
    public RobustnessTestConfig getTestConfigurationForLevel(int level)
    {
        if (level < 1 || level > getNumberOfTestLevels()) {
            throw new IllegalArgumentException("Level must be between 1 and " + getNumberOfTestLevels());
        }

        RobustnessTestConfig baseConfig = getTestConfiguration();
        double forceIncrement = (baseConfig.maxForce() - baseConfig.minForce()) / (getNumberOfTestLevels() - 1);
        double durationIncrement = (baseConfig.maxDuration() - baseConfig.minDuration()) / (getNumberOfTestLevels() - 1);

        double currentForce = baseConfig.minForce() + ((level - 1) * forceIncrement);
        double currentDuration = baseConfig.minDuration() + ((level - 1) * durationIncrement);

        return new RobustnessTestConfig(currentForce, currentForce, currentDuration,
                currentDuration, baseConfig.pushInterval(), baseConfig.pushDirection(), baseConfig.pushOffset());
    }
}