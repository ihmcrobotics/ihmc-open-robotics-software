package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoFramePointArray
{
   private final YoFramePoint[] yoFramePointArray;
   
   public YoFramePointArray(int arraySize, String namePrefix, YoVariableRegistry yoVariableRegistry)
   {
      yoFramePointArray = new YoFramePoint[arraySize];
      
      for (int index = 0; index < arraySize; index++)
      {
         yoFramePointArray[index] = new YoFramePoint(namePrefix + index, ReferenceFrame.getWorldFrame(), yoVariableRegistry);
      }
   }
   
   public YoFramePoint get(int index)
   {
      return yoFramePointArray[index];
   }
}
