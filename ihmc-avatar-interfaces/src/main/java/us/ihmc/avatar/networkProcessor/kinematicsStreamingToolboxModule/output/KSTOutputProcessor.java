package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.output;

/**
 * Processing stage for transforming raw IK output into controller-ready setpoints.
 */
public interface KSTOutputProcessor
{
   void initialize();

   void update(double time, boolean wasStreaming, boolean isStreaming, KSTOutputDataReadOnly latestOutput);

   KSTOutputDataReadOnly getProcessedOutput();
}
