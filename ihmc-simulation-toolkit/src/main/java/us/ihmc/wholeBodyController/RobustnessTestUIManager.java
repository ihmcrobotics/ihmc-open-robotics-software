package us.ihmc.wholeBodyController;

import javafx.scene.control.Button;
import javafx.scene.control.Tooltip;
import us.ihmc.scs2.SimulationConstructionSet2;

public class RobustnessTestUIManager
{
    public interface RobustnessTestCallbacks
    {
        void startRobustnessTest();
        void stopRobustnessTest();
        void resetRobustnessTest();
        void applyLightPush();
        void applyHeavyPush();
    }

    private final RobustnessTestParameters parameters;
    private final RobustnessTestCallbacks callbacks;

    public RobustnessTestUIManager(RobustnessTestParameters parameters, RobustnessTestCallbacks callbacks)
    {
        this.parameters = parameters;
        this.callbacks = callbacks;
    }

    /**
     * Adds all robustness test buttons to the provided simulation construction set.
     */
    public void addRobustnessTestButtons(SimulationConstructionSet2 scs2)
    {
        scs2.executeOrScheduleVisualizerTask(() -> {
            // Start Robustness Test Button
            Button startTestButton = new Button("Start Robustness Test");
            startTestButton.setTooltip(new Tooltip("Start robustness test sequence (levels 1-" + parameters.getNumberOfTestLevels() + ")"));
            startTestButton.setOnAction(e -> callbacks.startRobustnessTest());

            // Stop Test Button
            Button stopTestButton = new Button("Stop Test");
            stopTestButton.setTooltip(new Tooltip("Stop current robustness test"));
            stopTestButton.setOnAction(e -> callbacks.stopRobustnessTest());

            // Reset Test Button
            Button resetTestButton = new Button("Reset Test");
            resetTestButton.setTooltip(new Tooltip("Reset test progress to level 1"));
            resetTestButton.setOnAction(e -> callbacks.resetRobustnessTest());

            // Quick Test Buttons
            Button lightPushButton = new Button("Light Push");
            lightPushButton.setTooltip(new Tooltip("Apply single light push (" + parameters.getLightPushForce() + "N, " + parameters.getLightPushDuration() + "s)"));
            lightPushButton.setOnAction(e -> callbacks.applyLightPush());

            Button heavyPushButton = new Button("Heavy Push");
            heavyPushButton.setTooltip(new Tooltip("Apply single heavy push (" + parameters.getHeavyPushForce() + "N, " + parameters.getHeavyPushDuration() + "s)"));
            heavyPushButton.setOnAction(e -> callbacks.applyHeavyPush());

            // Add all buttons to scs2
            scs2.addCustomGUIControl(startTestButton);
            scs2.addCustomGUIControl(stopTestButton);
            scs2.addCustomGUIControl(resetTestButton);
            scs2.addCustomGUIControl(lightPushButton);
            scs2.addCustomGUIControl(heavyPushButton);
        });
    }

}