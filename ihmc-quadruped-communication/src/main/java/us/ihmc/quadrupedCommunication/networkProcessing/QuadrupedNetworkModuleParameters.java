package us.ihmc.quadrupedCommunication.networkProcessing;

import java.net.URI;

public class QuadrupedNetworkModuleParameters
{
   private boolean useNetworkProcessor = true;
   private boolean useRobotEnvironmentAwarenessModule;
   private boolean useBodyHeightTeleopModule;
   private boolean visualizeBodyHeightTeleopModule;
   private boolean logBodyHeightTeleopModule;
   private boolean useStepTeleopModule;
   private boolean visualizeStepTeleopModule;
   private boolean logStepTeleopModule;
   private boolean useFootstepPlanningModule;
   private boolean visualizeFootstepPlanningModule;
   private boolean logFootstepPlanningModule;
   private boolean useBodyTeleopModule;
   private boolean visualizeBodyTeleopModule;
   private boolean logBodyTeleopModule;
   private boolean useXBoxModule;
   private boolean useRemoteObjectDetectionFeedback;
   private boolean visualizeXBoxModule;

   private URI rosUri;

   public void enableRobotEnvironmentAwarenessModule(boolean enable)
   {
      this.useRobotEnvironmentAwarenessModule = enable;
   }

   public boolean isRobotEnvironmentAwarenessModuleEnabled()
   {
      return useRobotEnvironmentAwarenessModule;
   }

   public void enableBodyHeightTeleopModule(boolean enable)
   {
      this.useBodyHeightTeleopModule = enable;
   }

   public boolean isBodyHeightTeleopModuleEnabled()
   {
      return useBodyHeightTeleopModule;
   }

   public void enableFootstepPlanningModule(boolean enable)
   {
      this.useFootstepPlanningModule = enable;
   }

   public boolean isFootstepPlanningModuleEnabled()
   {
      return useFootstepPlanningModule;
   }

   public void enableStepTeleopModule(boolean enable)
   {
      this.useStepTeleopModule = enable;
   }

   public boolean isStepTeleopModuleEnabled()
   {
      return useStepTeleopModule;
   }

   public void setVisualizeBodyHeightTeleopModule(boolean visualize)
   {
      visualizeBodyHeightTeleopModule = visualize;
   }

   public boolean visualizeBodyHeightTeleopModuleEnabled()
   {
      return visualizeBodyHeightTeleopModule;
   }

   public void setLogBodyHeightTeleopModule(boolean log)
   {
      logBodyHeightTeleopModule = log;
   }

   public boolean logBodyHeightTeleopModuleEnabled()
   {
      return logBodyHeightTeleopModule;
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

   public void setVisualizeBodyTeleopModule(boolean visualize)
   {
      visualizeBodyTeleopModule = visualize;
   }

   public boolean visualizeBodyTeleopModuleEnabled()
   {
      return visualizeBodyTeleopModule;
   }

   public void setVisualizeXBoxModule(boolean visualize)
   {
      this.visualizeXBoxModule = visualize;
   }

   public boolean visualizeXBoxModule()
   {
      return visualizeXBoxModule;
   }

   public void setLogBodyTeleopModule(boolean log)
   {
      logBodyTeleopModule = log;
   }

   public boolean logBodyTeleopModuleEnabled()
   {
      return logBodyTeleopModule;
   }

   public void enableBodyTeleopModule(boolean enable)
   {
      this.useBodyTeleopModule = enable;
   }

   public boolean isBodyTeleopModuleEnabled()
   {
      return useBodyTeleopModule;
   }

   public void enableXBoxModule(boolean enable)
   {
      this.useXBoxModule = enable;
   }

   public boolean isXBoxModuleEnabled()
   {
      return useXBoxModule;
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
