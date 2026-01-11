package us.ihmc.exampleSimulations.controllerCore.robotArmWithMovingBase;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.MidiSliderBoard;

public class MovingBaseRobotArmSimulation
{
   public static void main(String[] args)
   {
      double controlDT = 5.0e-5;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      MovingBaseRobotArmDefinition robotArmDefinition = new MovingBaseRobotArmDefinition();
//      robotArmController.registerControllerCoreModeChangedListener((mode) -> robotArmDefinition.setDynamic(mode == WholeBodyControllerCoreMode.INVERSE_DYNAMICS));

      SimulationConstructionSet2 scs = new SimulationConstructionSet2(ReferenceFrame.getWorldFrame());
      Robot robotArm = scs.addRobot(robotArmDefinition);
      MovingBaseRobotArmController robotArmController = new MovingBaseRobotArmController(robotArm,
                                                                                         scs.getTime(),
                                                                                         scs.getGravity().getZ(),
                                                                                         controlDT,
                                                                                         yoGraphicsListRegistry);
      robotArm.addThrottledController(robotArmController, controlDT);
      setupSliderBoard(scs);
      scs.initializeBufferSize((int) Math.pow(2, 16)); // => 65536
      scs.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(yoGraphicsListRegistry));
      scs.setDT(controlDT);
      scs.setBufferRecordTickPeriod(10);
      scs.startSimulationThread();
   }

   private static void setupSliderBoard(SimulationConstructionSet2 scs)
   {
      MidiSliderBoard midiSliderBoard = new MidiSliderBoard(scs);
      int sliderIndex = 1;
      midiSliderBoard.setSlider(sliderIndex++, "handTargetX", scs, -1.0, 1.0);
      midiSliderBoard.setSlider(sliderIndex++, "handTargetY", scs, -1.0, 1.0);
      midiSliderBoard.setSlider(sliderIndex++, "handTargetZ", scs,  0.0, 2.0);
      midiSliderBoard.setSlider(sliderIndex++, "handTargetYaw", scs, -Math.PI, Math.PI);
      midiSliderBoard.setSlider(sliderIndex++, "handTargetPitch", scs, -Math.PI, Math.PI);
      midiSliderBoard.setSlider(sliderIndex++, "handTargetRoll", scs, -Math.PI, Math.PI);

      int buttonIndex = 1;
      midiSliderBoard.setButton(buttonIndex++, "goToTarget", scs);
      midiSliderBoard.setButton(buttonIndex++, "setRandomConfiguration", scs);

      buttonIndex = 9; // Second row
      midiSliderBoard.setButton(buttonIndex++, "controlLinearX", scs);
      midiSliderBoard.setButton(buttonIndex++, "controlLinearY", scs);
      midiSliderBoard.setButton(buttonIndex++, "controlLinearZ", scs);
      midiSliderBoard.setButton(buttonIndex++, "controlAngularX", scs);
      midiSliderBoard.setButton(buttonIndex++, "controlAngularY", scs);
      midiSliderBoard.setButton(buttonIndex++, "controlAngularZ", scs);

      int knobIndex = 1;
      midiSliderBoard.setKnob(knobIndex++, "baseTrajectoryAmplitudeX", scs, 0.0, 2.0);
      midiSliderBoard.setKnob(knobIndex++, "baseTrajectoryAmplitudeY", scs, 0.0, 2.0);
      midiSliderBoard.setKnob(knobIndex++, "baseTrajectoryAmplitudeZ", scs, 0.0, 2.0);
      midiSliderBoard.setKnob(knobIndex++, "baseTrajectoryFrequencyX", scs, 0.0, 2.0);
      midiSliderBoard.setKnob(knobIndex++, "baseTrajectoryFrequencyY", scs, 0.0, 2.0);
      midiSliderBoard.setKnob(knobIndex++, "baseTrajectoryFrequencyZ", scs, 0.0, 2.0);
   }
}
