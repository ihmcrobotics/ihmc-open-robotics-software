package us.ihmc.parameterTuner.sliderboard;

public interface ButtonListener
{
   /**
    * Will be called when a button on a slider board was pressed.
    *
    * @param status whether the button was switched to on or off.
    */
   public void buttonPressed(boolean status);
}
