package us.ihmc.wholeBodyController;

import us.ihmc.euclid.tuple3D.Vector3D;

public interface RobustnessTestParameters
{
    RobustnessTestConfig getTestConfiguration();
    RobustnessTestConfig getTestConfigurationForLevel(int level);

    int getNumberOfTestLevels();

    // Individual parameter access
    double getMinForce();
    double getMaxForce();
    double getMinDuration();
    double getMaxDuration();
    double getPushInterval();
    Vector3D getPushDirection();
    Vector3D getPushOffset();

    // Quick access methods for manual testing
    double getLightPushForce();
    double getLightPushDuration();
    double getHeavyPushForce();
    double getHeavyPushDuration();
    Vector3D getDefaultPushDirection();

    record RobustnessTestConfig(double minForce, double maxForce, double minDuration,
                                double maxDuration, double pushInterval, Vector3D pushDirection,
                                Vector3D pushOffset)
    {
        public RobustnessTestConfig(double minForce, double maxForce, double minDuration,
                                    double maxDuration, double pushInterval, Vector3D pushDirection,
                                    Vector3D pushOffset)
        {
            this.minForce = minForce;
            this.maxForce = maxForce;
            this.minDuration = minDuration;
            this.maxDuration = maxDuration;
            this.pushInterval = pushInterval;
            this.pushDirection = new Vector3D(pushDirection);
            this.pushDirection.normalize(); // Ensure unit vector
            this.pushOffset = pushOffset;
        }
    }
}