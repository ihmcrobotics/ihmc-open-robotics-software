package us.ihmc.graphicsDescription.dataBuffer;

import us.ihmc.yoVariables.variable.YoVariable;

public interface DataEntryHolder
{
   public DataEntry getEntry(YoVariable<?> yoVariable);
}
