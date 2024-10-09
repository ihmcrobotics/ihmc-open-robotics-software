package us.ihmc.parameterTuner.sliderboard;

public interface SliderboardListener
{
   /**
    * Will be called when the slider on a slider board was moved.
    *
    * @param sliderPercentage the new slider value between 0.0 and 1.0
    */
   public void sliderMoved(double sliderPercentage);
}
