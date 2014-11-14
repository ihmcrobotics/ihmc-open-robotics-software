package us.ihmc.simulationconstructionset.dataBuffer;

import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public interface DataEntryHolder
{
   public DataEntry getEntry(YoVariable<?> yoVariable);
}
