package us.ihmc.quadrupedCommunication.networkProcessing;

public class QuadrupedNetworkModuleParameters
{
   private boolean useNetworkProcessor = true;
   private boolean useRobotEnvironmentAwarenessModule;
   private boolean useStepTeleopModule;
   private boolean visualizeStepTeleopModule;
   private boolean logStepTeleopModule;
   private boolean useQuadrupedSupportPlanarRegionPublisher;
   private boolean useFootstepPlanningModule;
   private boolean visualizeFootstepPlanningModule;
   private boolean logFootstepPlanningModule;
   private boolean useRemoteObjectDetectionFeedback;
   private boolean useAutoREAStateUpdater;

   public void enableRobotEnvironmentAwarenessModule(boolean enable)
   {
      this.useRobotEnvironmentAwarenessModule = enable;
   }
   
   public void enableAutoREAStateUpdater(boolean enable)
   {
      this.useAutoREAStateUpdater = enable;
   }
   
   public boolean isAutoREAStateUpdaterEnabled()
   {
      return useAutoREAStateUpdater;
   }

   public boolean isRobotEnvironmentAwarenessModuleEnabled()
   {
      return useRobotEnvironmentAwarenessModule;
   }

   public void enableFootstepPlanningModule(boolean enable)
   {
      this.useFootstepPlanningModule = enable;
   }

   public boolean isFootstepPlanningModuleEnabled()
   {
      return useFootstepPlanningModule;
   }

   public void enableQuadrupedSupportPlanarRegionPublisher(boolean enable)
   {
      useQuadrupedSupportPlanarRegionPublisher = enable;
   }

   public boolean isQuadrupedSupportPlanarRegionPublisherEnabled()
   {
      return useQuadrupedSupportPlanarRegionPublisher;
   }

   public void enableStepTeleopModule(boolean enable)
   {
      this.useStepTeleopModule = enable;
   }

   public boolean isStepTeleopModuleEnabled()
   {
      return useStepTeleopModule;
   }

   public void setVisualizeStepTeleopModule(boolean visualize)
   {
      this.visualizeStepTeleopModule = visualize;
   }

   public boolean visualizeStepTeleopModuleEnabled()
   {
      return visualizeStepTeleopModule;
   }

   public void setLogStepTeleopModule(boolean log)
   {
      logStepTeleopModule = log;
   }

   public boolean logStepTeleopModuleEnabled()
   {
      return logStepTeleopModule;
   }

   public void setVisualizeFootstepPlanningModule(boolean visualize)
   {
      this.visualizeFootstepPlanningModule = visualize;
   }

   public boolean visualizeFootstepPlanningModuleEnabled()
   {
      return visualizeFootstepPlanningModule;
   }

   public void setLogFootstepPlanningModule(boolean log)
   {
      logFootstepPlanningModule = log;
   }

   public boolean logFootstepPlanningModuleEnabled()
   {
      return logFootstepPlanningModule;
   }

   public void enableNetworkProcessor(boolean useNetworkProcessor)
   {
      this.useNetworkProcessor = useNetworkProcessor;
   }

   public boolean isNetworkProcessorEnabled()
   {
      return useNetworkProcessor;
   }

   public boolean isRemoteObjectDetectionFeedbackEnabled()
   {
      return useRemoteObjectDetectionFeedback;
   }

   public void setUseRemoteObjectDetectionFeedbackEnabled(boolean useRemoteObjectDetectionFeedback)
   {
      this.useRemoteObjectDetectionFeedback = useRemoteObjectDetectionFeedback;
   }

   @Override
   public String toString()
   {
      return "QuadrupedNetworkModuleParameters [useRobotEnvironmentAwarenessModule=" + useRobotEnvironmentAwarenessModule;
   }
}
