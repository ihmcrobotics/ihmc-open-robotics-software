package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output;

public interface KSTOutputProcessor
{
   void initialize();

   void update(double time, boolean wasStreaming, boolean isStreaming, KSTOutputDataReadOnly latestOutput);

   KSTOutputDataReadOnly getProcessedOutput();
}
