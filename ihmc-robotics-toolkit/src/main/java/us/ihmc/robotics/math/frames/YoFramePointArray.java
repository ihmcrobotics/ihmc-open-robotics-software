package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoFramePointArray
{
   private final YoFramePoint3D[] yoFramePointArray;
   
   public YoFramePointArray(int arraySize, String namePrefix, YoRegistry yoVariableRegistry)
   {
      yoFramePointArray = new YoFramePoint3D[arraySize];
      
      for (int index = 0; index < arraySize; index++)
      {
         yoFramePointArray[index] = new YoFramePoint3D(namePrefix + index, ReferenceFrame.getWorldFrame(), yoVariableRegistry);
      }
   }
   
   public YoFramePoint3D get(int index)
   {
      return yoFramePointArray[index];
   }
}
