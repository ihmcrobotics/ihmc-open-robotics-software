package us.ihmc.sensorProcessing.outputData;

public interface LowLevelOutputWriter
{
   void setLowLevelOneDoFJointDesiredDataHolderList(LowLevelOneDoFJointDesiredDataHolderList lowLevelDataHolder);
   
   void write();
}
