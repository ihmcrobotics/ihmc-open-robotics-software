package us.ihmc.quadrupedRobotics.input;

import us.ihmc.simulationConstructionSetTools.util.inputdevices.MidiSliderBoard;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class QuadrupedXGaitSliderBoard
{
   private static final boolean SHOW_VIRTUAL_SLIDER_BOARD = false;
   private final MidiSliderBoard sliderBoard;

   public QuadrupedXGaitSliderBoard(SimulationConstructionSet scs)
   {
      sliderBoard = new MidiSliderBoard(scs, SHOW_VIRTUAL_SLIDER_BOARD);

      sliderBoard.setSlider(0, "stepDurationInput", scs, 0.1, 1.0);
      sliderBoard.setSlider(1, "endDoubleSupportDurationInput", scs, 0.001, 0.5);
      sliderBoard.setSlider(2, "stanceLengthInput", scs, 0.4, 1.4);
      sliderBoard.setSlider(3, "stanceWidthInput", scs, 0.1, 0.6);
      sliderBoard.setSlider(4, "stepGroundClearanceInput", scs, 0.01, 0.25);

      sliderBoard.setKnob(0, "endPhaseShiftInput", scs, 0.0, 359);
      sliderBoard.setKnob(1, "teleopDesiredVelocityZ", scs, -0.25, 0.25);

      sliderBoard.setButton(0, "xGaitRequested", scs);

      sliderBoard.setSlider(5, "teleopDesiredVelocityX", scs, -1.5, 1.5);
      sliderBoard.setSlider(6, "teleopDesiredVelocityY", scs, -0.5, 0.5);
   }
}
