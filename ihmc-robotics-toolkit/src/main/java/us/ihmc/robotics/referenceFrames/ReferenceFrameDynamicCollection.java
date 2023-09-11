package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

import java.util.List;
import java.util.function.Function;

public class ReferenceFrameDynamicCollection
{
   private final List<String> frameNameList;
   private final Function<String, ReferenceFrame> frameLookup;

   public ReferenceFrameDynamicCollection(List<String> frameNameList, Function<String, ReferenceFrame> frameLookup)
   {
      this.frameNameList = frameNameList;
      this.frameLookup = frameLookup;
   }

   public List<String> getFrameNameList()
   {
      return frameNameList;
   }

   public Function<String, ReferenceFrame> getFrameLookup()
   {
      return frameLookup;
   }
}
