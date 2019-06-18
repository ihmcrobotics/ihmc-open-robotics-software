package us.ihmc.sensorProcessing.frames;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface ReferenceFrames
{
   public void updateFrames();

   public ReferenceFrame getCenterOfMassFrame();

   /**
    * This function should map reference frames to custom hash ids, Each reference frame generates a
    * hash id based on its name and its parent name, In some instances it is desirable to also assign a
    * custom id to make reference frames robot agnostic ( not depend on their name).
    */
   default TLongObjectHashMap<ReferenceFrame> getReferenceFrameDefaultHashIds()
   {
      return null;
   }
}