package us.ihmc.wholeBodyController;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class AbstractFlatGroundWalkingTrackRobustness implements RobustnessTestUIManager.RobustnessTestCallbacks
{
    // Common control variables
    protected final YoBoolean testRunning = new YoBoolean("robustnessTestRunning", null);
    protected final YoInteger currentTestLevel = new YoInteger("currentTestLevel", null);
    protected final YoBoolean testCompleted = new YoBoolean("robustnessTestCompleted", null);
    protected Thread currentTestThread;

    // Common components
    protected PushRobotControllerSCS2 pushController;
    protected RobustnessTestParameters robustnessTestParameters;
    protected RobustnessTestUIManager uiManager;

    protected AbstractFlatGroundWalkingTrackRobustness()
    {
        // Initialize control variables
        testRunning.set(false);
        currentTestLevel.set(0);
        testCompleted.set(false);
        testCompleted.set(false);
    }

    // Abstract methods for robot-specific implementation
    protected abstract RobustnessTestParameters createRobustnessTestParameters();
    protected abstract String getPushJointName();
    protected abstract Vector3D getPushOffset();
    protected abstract void setupRobotSpecificSimulation();
    protected abstract void addUIButtons();
    protected abstract DoubleProvider getSimulationTime();
    protected abstract Robot getRobot();

    // Common robustness test setup
    protected void setupRobustnessTest()
    {
        robustnessTestParameters = createRobustnessTestParameters();
        setupPushController();
        setupUI();
    }

    protected void setupPushController()
    {
        String pushJoint = getPushJointName();
        Vector3D forceOffset = getPushOffset();

        pushController = new PushRobotControllerSCS2(
                getSimulationTime(),
                getRobot(),
                pushJoint,
                forceOffset);

        // Configure default push parameters
        pushController.setPushDuration(robustnessTestParameters.getMinDuration());
        pushController.setPushForceMagnitude(robustnessTestParameters.getMinForce());
        pushController.setPushForceDirection(robustnessTestParameters.getPushDirection());
    }

    protected void setupUI()
    {
        uiManager = new RobustnessTestUIManager(robustnessTestParameters, this);
        addUIButtons();
    }

    // Common callback implementations
    @Override
    public void startRobustnessTest()
    {
        if (testRunning.getValue()) {
            LogTools.warn("Robustness test already running. Stop current test first.");
            return;
        }

        if (testCompleted.getValue()) {
            LogTools.info("Previous test completed. Starting new test from level 1.");
            resetRobustnessTest();
        }

        testRunning.set(true);
        testCompleted.set(false);

        LogTools.info("=== Starting Robustness Test Sequence ===");
        currentTestThread = new Thread(new RobustnessTestSequence(), "RobustnessTestThread");
        currentTestThread.setDaemon(true);
        currentTestThread.start();
    }

    @Override
    public void stopRobustnessTest()
    {
        if (testRunning.getValue()) {
            testRunning.set(false);
            if (currentTestThread != null) {
                currentTestThread.interrupt();
            }
            LogTools.info("Robustness test stopped at level " + currentTestLevel.getValue());
        } else {
            LogTools.info("No robustness test currently running.");
        }
    }

    @Override
    public void resetRobustnessTest()
    {
        stopRobustnessTest();
        currentTestLevel.set(0);
        testCompleted.set(false);
        LogTools.info("Robustness test reset to initial state.");
    }

    @Override
    public void applyLightPush()
    {
        applyTestPush(robustnessTestParameters.getLightPushForce(),
                robustnessTestParameters.getLightPushDuration(),
                robustnessTestParameters.getDefaultPushDirection());
    }

    @Override
    public void applyHeavyPush()
    {
        applyTestPush(robustnessTestParameters.getHeavyPushForce(),
                robustnessTestParameters.getHeavyPushDuration(),
                robustnessTestParameters.getDefaultPushDirection());
    }

    public void applyTestPush(double forceMagnitude, double duration, Vector3D direction)
    {
        pushController.setPushForceMagnitude(forceMagnitude);
        pushController.setPushDuration(duration);
        pushController.setPushForceDirection(direction);
        pushController.applyForce(direction, forceMagnitude, duration);

        LogTools.info("Manual push applied: Force={}N, Duration={}s, Direction={}", forceMagnitude, duration, direction);
    }

    // Common robustness test sequence
    private class RobustnessTestSequence implements Runnable
    {
        @Override
        public void run()
        {
            try {
                Thread.sleep(1500); // stabilize
                int maxLevels = robustnessTestParameters.getNumberOfTestLevels();

                for (int level = 1; level <= maxLevels && testRunning.getValue(); level++) {
                    currentTestLevel.set(level);
                    var levelConfig = robustnessTestParameters.getTestConfigurationForLevel(level);

                    LogTools.info(String.format("=== Robustness Test Level %d/%d === Force: %.1fN, Duration: %.1fs",
                            level, maxLevels, levelConfig.minForce(), levelConfig.minDuration()));

                    // Apply push
                    pushController.setPushForceMagnitude(levelConfig.minForce());
                    pushController.setPushDuration(levelConfig.minDuration());
                    pushController.setPushForceDirection(levelConfig.pushDirection());
                    pushController.applyForce(levelConfig.pushDirection(), levelConfig.minForce(), levelConfig.minDuration());

                    // Wait for recovery
                    if (level < maxLevels && testRunning.getValue()) {
                        Thread.sleep((long)(robustnessTestParameters.getPushInterval() * 1000));
                    }
                }

                if (testRunning.getValue()) {
                    testCompleted.set(true);
                    testRunning.set(false);
                    LogTools.info("=== Robustness Test Sequence Completed ===");
                }
            } catch (InterruptedException e) {
                LogTools.info("Robustness test sequence interrupted.");
                testRunning.set(false);
            } catch (Exception e) {
                LogTools.error("Error in robustness test sequence: " + e.getMessage());
                e.printStackTrace();
                testRunning.set(false);
            }
        }
    }
}
