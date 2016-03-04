package us.ihmc.quadrupedRobotics.sliderboard;

import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;

public enum QuadrupedSliderBoardMode
{
   POSITIONCRAWL_COM_SHIFT, POSITIONCRAWL_FOOTSTEP_CHOOSER, POSITIONCRAWL_ORIENTATION_TUNING, TROT_WALK, LEG_JOINT_SLIDERBOARD, NECK_JOINT_SLIDER_BOARD;
   
   @SuppressWarnings("unchecked")
   public static EnumYoVariable<QuadrupedSliderBoardMode> getYoVariable(YoVariableHolder yoVariableHolder)
   {
      return (EnumYoVariable<QuadrupedSliderBoardMode>) yoVariableHolder.getVariable("sliderboardMode");
   }
   
   public static EnumYoVariable<QuadrupedSliderBoardMode> createYoVariable(YoVariableRegistry registry)
   {
      return new EnumYoVariable<>("sliderboardMode", registry,QuadrupedSliderBoardMode.class);
   }
}
