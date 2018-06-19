package us.ihmc.quadrupedRobotics.input;

import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerRequestedEvent;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.MidiSliderBoard;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class QuadrupedXGaitSliderBoard
{
   private static final boolean SHOW_VIRTUAL_SLIDER_BOARD = false;
   private final MidiSliderBoard sliderBoard;

   public QuadrupedXGaitSliderBoard(SimulationConstructionSet scs)
   {
      sliderBoard = new MidiSliderBoard(scs, SHOW_VIRTUAL_SLIDER_BOARD);

      sliderBoard.setSlider(1, "stepDurationInput", scs, 0.1, 1.0);
      sliderBoard.setSlider(2, "endDoubleSupportDurationInput", scs, 0.001, 0.5);
      sliderBoard.setSlider(3, "stanceLengthInput", scs, 0.4, 1.4);
      sliderBoard.setSlider(4, "stanceWidthInput", scs, 0.1, 0.6);
      sliderBoard.setSlider(5, "stepGroundClearanceInput", scs, 0.01, 0.25);
      sliderBoard.setSlider(6, "teleopDesiredVelocityX", scs, -1.5, 1.5);
      sliderBoard.setSlider(7, "teleopDesiredVelocityY", scs, -0.5, 0.5);

      sliderBoard.setKnob(1, "endPhaseShiftInput", scs, 0.0, 359);
      sliderBoard.setKnob(2, "teleopDesiredVelocityZ", scs, -0.4, 0.4);

//      sliderBoard.setButtonEnum(0, "teleopControllerRequestedEvent", scs, QuadrupedControllerRequestedEvent.REQUEST_STAND_PREP);
      sliderBoard.setButtonEnum(9, "teleopControllerRequestedEvent", scs, QuadrupedControllerRequestedEvent.REQUEST_STEPPING);
      sliderBoard.setButtonEnum(1, "teleopControllerRequestedEvent", scs, QuadrupedControllerRequestedEvent.REQUEST_FREEZE);
      sliderBoard.setButton(2, "xGaitRequested", scs);
      sliderBoard.setButton(3, "standingRequested", scs);


   }
}
