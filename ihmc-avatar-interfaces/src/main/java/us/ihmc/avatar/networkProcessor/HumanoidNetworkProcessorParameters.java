package us.ihmc.avatar.networkProcessor;

import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.communication.net.LocalObjectCommunicator;

public class HumanoidNetworkProcessorParameters
{
   private URI rosURI;
   private LocalObjectCommunicator simulatedSensorCommunicator;

   private boolean useTextToSpeechEngine;
   private boolean useZeroPoseRobotConfigurationPublisherModule;
   private boolean useWholeBodyTrajectoryToolboxModule, visualizeWholeBodyTrajectoryToolboxModule;
   private boolean useKinematicsToolboxModule, visualizeKinematicsToolboxModule;
   private boolean useKinematicsPlanningToolboxModule, visualizeKinematicsPlanningToolboxModule;
   private boolean useKinematicsStreamingToolboxModule, visualizeKinematicsStreamingToolboxModule;
   private boolean useFootstepPlanningToolboxModule, visualizeFootstepPlanningToolboxModule;
   private boolean useMocapModule;
   private boolean useBehaviorModule, visualizeBehaviorModule;
   private boolean useAutomaticDiagnostic;
   private double automatedDiagnosticInitialDelay;
   private boolean useROSModule;
   private boolean useSensorModule;
   private boolean useHeightQuadTreeToolboxModule;
   private boolean useFiducialDetectorToolboxModule;
   private boolean useObjectDetectorToolboxModule;


   private boolean useRobotEnvironmentAwerenessModule;
   private String reaConfigurationFilePath;
   private boolean useBipedalSupportPlanarRegionPublisherModule;
   private boolean useWalkingPreviewModule, visualizeWalkingPreviewModule;
   private boolean useHumanoidAvatarREAStateUpdater;

   public void setRosURI(URI rosURI)
   {
      this.rosURI = rosURI;
   }

   public void setROSLocalhostURI()
   {
      try
      {
         setRosURI(new URI("http://localhost:11311"));
      }
      catch (URISyntaxException e)
      {
         e.printStackTrace();
      }
   }

   public void setSimulatedSensorCommunicator(LocalObjectCommunicator simulatedSensorCommunicator)
   {
      this.simulatedSensorCommunicator = simulatedSensorCommunicator;
   }

   public void setUseTextToSpeechEngine(boolean enable)
   {
      this.useTextToSpeechEngine = enable;
   }

   public void setUseZeroPoseRobotConfigurationPublisherModule(boolean enable)
   {
      this.useZeroPoseRobotConfigurationPublisherModule = enable;
   }

   public void setUseWholeBodyTrajectoryToolboxModule(boolean enable)
   {
      setUseWholeBodyTrajectoryToolboxModule(enable, false);
   }

   public void setUseWholeBodyTrajectoryToolboxModule(boolean enable, boolean visualize)
   {
      this.useWholeBodyTrajectoryToolboxModule = enable;
      this.visualizeWholeBodyTrajectoryToolboxModule = visualize;
   }

   public void setUseKinematicsToolboxModule(boolean enable)
   {
      setUseKinematicsToolboxModule(enable, false);
   }

   public void setUseKinematicsToolboxModule(boolean enable, boolean visualize)
   {
      this.useKinematicsToolboxModule = enable;
      this.visualizeKinematicsToolboxModule = visualize;
   }

   public void setUseKinematicsPlanningToolboxModule(boolean enable)
   {
      setUseKinematicsPlanningToolboxModule(enable, false);
   }

   public void setUseKinematicsPlanningToolboxModule(boolean enable, boolean visualize)
   {
      this.useKinematicsPlanningToolboxModule = enable;
      this.visualizeKinematicsPlanningToolboxModule = visualize;
   }

   public void setUseKinematicsStreamingToolboxModule(boolean enable)
   {
      setUseKinematicsStreamingToolboxModule(enable, false);
   }

   public void setUseKinematicsStreamingToolboxModule(boolean enable, boolean visualize)
   {
      this.useKinematicsStreamingToolboxModule = enable;
      this.visualizeKinematicsStreamingToolboxModule = visualize;
   }

   public void setUseFootstepPlanningToolboxModule(boolean enable)
   {
      setUseFootstepPlanningToolboxModule(enable, false);
   }

   public void setUseFootstepPlanningToolboxModule(boolean enable, boolean visualize)
   {
      this.useFootstepPlanningToolboxModule = enable;
      this.visualizeFootstepPlanningToolboxModule = visualize;
   }

   public void setUseMocapModule(boolean enable)
   {
      this.useMocapModule = enable;
   }

   public void setUseBehaviorModule(boolean enable)
   {
      setUseBehaviorModule(enable, false);
   }

   public void setUseBehaviorModule(boolean enable, boolean visualize)
   {
      this.useBehaviorModule = enable;
      this.visualizeBehaviorModule = visualize;
   }

   public void setUseAutomaticDiagnostic(boolean enable, double autamotedDiagnosticeInitialDelay)
   {
      setUseAutomaticDiagnostic(enable, false, autamotedDiagnosticeInitialDelay);
   }

   public void setUseAutomaticDiagnostic(boolean enable, boolean visualize, double autamotedDiagnosticInitialDelay)
   {
      if (enable == true)
         useBehaviorModule = true;
      this.useAutomaticDiagnostic = enable;
      this.visualizeBehaviorModule = visualize;
      this.automatedDiagnosticInitialDelay = autamotedDiagnosticInitialDelay;
   }

   public void setUseROSModule(boolean enable)
   {
      this.useROSModule = enable;
   }

   public void setUseSensorModule(boolean useSensorModule)
   {
      this.useSensorModule = useSensorModule;
   }

   public void setUseHeightQuadTreeToolboxModule(boolean enable)
   {
      this.useHeightQuadTreeToolboxModule = enable;
   }

   public void setUseRobotEnvironmentAwerenessModule(boolean enable)
   {
      setUseRobotEnvironmentAwerenessModule(enable, null);
   }

   public void setUseRobotEnvironmentAwerenessModule(boolean enable, String reaConfigurationFilePath)
   {
      this.useRobotEnvironmentAwerenessModule = enable;
      this.reaConfigurationFilePath = reaConfigurationFilePath;
   }

   public void setUseBipedalSupportPlanarRegionPublisherModule(boolean enable)
   {
      this.useBipedalSupportPlanarRegionPublisherModule = enable;
   }

   public void setUseWalkingPreviewModule(boolean enable)
   {
      setUseWalkingPreviewModule(enable, false);
   }

   public void setUseWalkingPreviewModule(boolean enable, boolean visualize)
   {
      this.useWalkingPreviewModule = enable;
      this.visualizeWalkingPreviewModule = visualize;
   }

   public void setUseHumanoidAvatarREAStateUpdater(boolean enable)
   {
      this.useHumanoidAvatarREAStateUpdater = enable;
   }

   public URI getRosURI()
   {
      return rosURI;
   }

   public LocalObjectCommunicator getSimulatedSensorCommunicator()
   {
      return simulatedSensorCommunicator;
   }

   public boolean isUseTextToSpeechEngine()
   {
      return useTextToSpeechEngine;
   }

   public boolean isUseZeroPoseRobotConfigurationPublisherModule()
   {
      return useZeroPoseRobotConfigurationPublisherModule;
   }

   public boolean isUseWholeBodyTrajectoryToolboxModule()
   {
      return useWholeBodyTrajectoryToolboxModule;
   }

   public boolean isVisualizeWholeBodyTrajectoryToolboxModule()
   {
      return visualizeWholeBodyTrajectoryToolboxModule;
   }

   public boolean isUseKinematicsToolboxModule()
   {
      return useKinematicsToolboxModule;
   }

   public boolean isVisualizeKinematicsToolboxModule()
   {
      return visualizeKinematicsToolboxModule;
   }

   public boolean isUseKinematicsPlanningToolboxModule()
   {
      return useKinematicsPlanningToolboxModule;
   }

   public boolean isVisualizeKinematicsPlanningToolboxModule()
   {
      return visualizeKinematicsPlanningToolboxModule;
   }

   public boolean isUseKinematicsStreamingToolboxModule()
   {
      return useKinematicsStreamingToolboxModule;
   }

   public boolean isVisualizeKinematicsStreamingToolboxModule()
   {
      return visualizeKinematicsStreamingToolboxModule;
   }

   public boolean isUseFootstepPlanningToolboxModule()
   {
      return useFootstepPlanningToolboxModule;
   }

   public boolean isVisualizeFootstepPlanningToolboxModule()
   {
      return visualizeFootstepPlanningToolboxModule;
   }

   public boolean isUseMocapModule()
   {
      return useMocapModule;
   }

   public boolean isUseBehaviorModule()
   {
      return useBehaviorModule;
   }

   public boolean isVisualizeBehaviorModule()
   {
      return visualizeBehaviorModule;
   }

   public boolean isUseAutomaticDiagnostic()
   {
      return useAutomaticDiagnostic;
   }

   public double getAutomatedDiagnosticInitialDelay()
   {
      return automatedDiagnosticInitialDelay;
   }

   public boolean isUseROSModule()
   {
      return useROSModule;
   }

   public boolean isUseSensorModule()
   {
      return useSensorModule;
   }

   public boolean isUseHeightQuadTreeToolboxModule()
   {
      return useHeightQuadTreeToolboxModule;
   }
   
   
   public void setUseFiducialDetectorToolboxModule(boolean enable)
   {
      this.useFiducialDetectorToolboxModule = enable;
   }
   
   public boolean isUseFiducialDetectorToolboxModule()
   {
      return useFiducialDetectorToolboxModule;
   }
   public void setUseObjectDetectorToolboxModule(boolean enable)
   {
      this.useObjectDetectorToolboxModule = enable;
   }
   
   public boolean isUseObjectDetectorToolboxModule()
   {
      return useObjectDetectorToolboxModule;
   }


   public boolean isUseRobotEnvironmentAwerenessModule()
   {
      return useRobotEnvironmentAwerenessModule;
   }

   public String getREAConfigurationFilePath()
   {
      return reaConfigurationFilePath;
   }

   public boolean isUseBipedalSupportPlanarRegionPublisherModule()
   {
      return useBipedalSupportPlanarRegionPublisherModule;
   }

   public boolean isUseWalkingPreviewModule()
   {
      return useWalkingPreviewModule;
   }

   public boolean isVisualizeWalkingPreviewModule()
   {
      return visualizeWalkingPreviewModule;
   }

   public boolean isUseHumanoidAvatarREAStateUpdater()
   {
      return useHumanoidAvatarREAStateUpdater;
   }
}
