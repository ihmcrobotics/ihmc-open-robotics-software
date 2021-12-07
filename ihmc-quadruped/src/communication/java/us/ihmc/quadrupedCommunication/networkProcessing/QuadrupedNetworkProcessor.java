package us.ihmc.quadrupedCommunication.networkProcessing;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.depthOutputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.lidarOutputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.outputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.stereoOutputTopic;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedCommunication.networkProcessing.pawPlanning.PawPlanningModule;
import us.ihmc.quadrupedCommunication.networkProcessing.reaUpdater.QuadrupedREAStateUpdater;
import us.ihmc.quadrupedCommunication.networkProcessing.stepTeleop.QuadrupedStepTeleopModule;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersBasics;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

public class QuadrupedNetworkProcessor
{
   private QuadrupedStepTeleopModule stepTeleopModule;

   public static final int footstepPlanningPort = 8007;
   public static final int continuousPlanningPort = 8007;

   private final List<QuadrupedToolboxModule> modules = new ArrayList<>();
   private QuadrupedSupportPlanarRegionPublisher supportPublisher = null;

   public QuadrupedNetworkProcessor(FullQuadrupedRobotModelFactory robotModel,
                                    QuadrupedNetworkModuleParameters params,
                                    QuadrantDependentList<ArrayList<Point2D>> groundContactPoints,
                                    VisibilityGraphsParametersBasics visibilityGraphsParameters,
                                    PawStepPlannerParametersBasics pawPlannerParameters,
                                    QuadrupedXGaitSettings xGaitSettings,
                                    PointFootSnapperParameters pointFootSnapperParameters)
   {
      this(robotModel, params, groundContactPoints, visibilityGraphsParameters, pawPlannerParameters, xGaitSettings, pointFootSnapperParameters,
           DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public QuadrupedNetworkProcessor(FullQuadrupedRobotModelFactory robotModel,
                                    QuadrupedNetworkModuleParameters params,
                                    QuadrantDependentList<ArrayList<Point2D>> groundContactPoints,
                                    VisibilityGraphsParametersBasics visibilityGraphsParameters,
                                    PawStepPlannerParametersBasics pawPlannerParameters,
                                    QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                    PointFootSnapperParameters pointFootSnapperParameters,
                                    DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this(robotModel, null, params, groundContactPoints, visibilityGraphsParameters, pawPlannerParameters, xGaitSettings, pointFootSnapperParameters,
           pubSubImplementation);
   }

   public QuadrupedNetworkProcessor(FullQuadrupedRobotModelFactory robotModel,
                                    LogModelProvider logModelProvider,
                                    QuadrupedNetworkModuleParameters params,
                                    QuadrantDependentList<ArrayList<Point2D>> groundContactPoints,
                                    VisibilityGraphsParametersBasics visibilityGraphsParameters,
                                    PawStepPlannerParametersBasics pawPlannerParameters,
                                    QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                    PointFootSnapperParameters pointFootSnapperParameters,
                                    DomainFactory.PubSubImplementation pubSubImplementation)
   {
      tryToStartModule(() -> setupFootstepPlanningModule(robotModel,
                                                         visibilityGraphsParameters,
                                                         pawPlannerParameters,
                                                         xGaitSettings,
                                                         pointFootSnapperParameters,
                                                         logModelProvider,
                                                         params,
                                                         pubSubImplementation));
      tryToStartModule(() -> setupStepTeleopModule(robotModel, xGaitSettings, pointFootSnapperParameters, logModelProvider, params, pubSubImplementation));
      tryToStartModule(() -> setupRobotEnvironmentAwarenessModule(params, pubSubImplementation));
      tryToStartModule(() -> setupREAStateUpdater(robotModel.getRobotDefinition().getName(), params));

      setupQuadrupedSupportPlanarRegionPublisherModule(robotModel, groundContactPoints, params, pubSubImplementation);
   }

   public void setRootRegistry(YoRegistry rootRegistry, YoGraphicsListRegistry rootGraphicsListRegistry)
   {
      for (QuadrupedToolboxModule module : modules)
      {
         module.setRootRegistry(rootRegistry, rootGraphicsListRegistry);
      }
   }

   public void setShiftPlanBasedOnStepAdjustment(boolean shift)
   {
      if (stepTeleopModule != null)
         stepTeleopModule.setShiftPlanBasedOnStepAdjustment(shift);
   }

   public void close()
   {
      for (int i = 0; i < modules.size(); i++)
      {
         modules.get(i).destroy();
      }
      if (supportPublisher != null)
         supportPublisher.close();
   }

   private void setupStepTeleopModule(FullQuadrupedRobotModelFactory modelFactory,
                                      QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                      PointFootSnapperParameters pointFootSnapperParameters,
                                      LogModelProvider logModelProvider,
                                      QuadrupedNetworkModuleParameters params,
                                      DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (!params.isStepTeleopModuleEnabled())
         return;
      stepTeleopModule = new QuadrupedStepTeleopModule(modelFactory,
                                                       xGaitSettings,
                                                       pointFootSnapperParameters,
                                                       logModelProvider,
                                                       params.visualizeStepTeleopModuleEnabled(),
                                                       params.logStepTeleopModuleEnabled(),
                                                       pubSubImplementation);
      modules.add(stepTeleopModule);
   }

   private void setupFootstepPlanningModule(FullQuadrupedRobotModelFactory modelFactory,
                                            VisibilityGraphsParametersBasics visibilityGraphsParameters,
                                            PawStepPlannerParametersBasics pawPlannerParameters,
                                            QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                            PointFootSnapperParameters pointFootSnapperParameters,
                                            LogModelProvider logModelProvider,
                                            QuadrupedNetworkModuleParameters params,
                                            DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (!params.isFootstepPlanningModuleEnabled())
         return;
      modules.add(new PawPlanningModule(modelFactory,
                                        visibilityGraphsParameters,
                                        pawPlannerParameters,
                                        xGaitSettings,
                                        pointFootSnapperParameters,
                                        logModelProvider,
                                        params.visualizeFootstepPlanningModuleEnabled(),
                                        params.logFootstepPlanningModuleEnabled(),
                                        pubSubImplementation));
   }

   private void setupQuadrupedSupportPlanarRegionPublisherModule(FullQuadrupedRobotModelFactory modelFactory,
                                                                 QuadrantDependentList<ArrayList<Point2D>> groundContactPoints,
                                                                 QuadrupedNetworkModuleParameters params,
                                                                 DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (params.isQuadrupedSupportPlanarRegionPublisherEnabled())
      {
         QuadrupedSupportPlanarRegionPublisher module = new QuadrupedSupportPlanarRegionPublisher(modelFactory, groundContactPoints, pubSubImplementation);
         module.start();
      }
   }

   private void setupRobotEnvironmentAwarenessModule(QuadrupedNetworkModuleParameters params, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (params.isRobotEnvironmentAwarenessModuleEnabled())
      {
         try
         {
            REANetworkProvider networkProvider = new REAPlanarRegionPublicNetworkProvider(outputTopic, lidarOutputTopic, stereoOutputTopic, depthOutputTopic);
            FilePropertyHelper filePropertyHelper = new FilePropertyHelper(System.getProperty("user.home")
                  + "/.ihmc/Configurations/defaultREAModuleConfiguration.txt");
            if (pubSubImplementation == DomainFactory.PubSubImplementation.FAST_RTPS)
               LIDARBasedREAModule.createRemoteModule(filePropertyHelper, networkProvider).start();
            else
               LIDARBasedREAModule.createIntraprocessModule(filePropertyHelper, networkProvider).start();
         }
         catch (Exception e)
         {
            throw new RuntimeException(e);
         }
      }
   }

   private void setupREAStateUpdater(String robotName, QuadrupedNetworkModuleParameters params)
   {
      if (params.isAutoREAStateUpdaterEnabled())
         new QuadrupedREAStateUpdater(robotName, PubSubImplementation.FAST_RTPS);
   }

   private void tryToStartModule(ModuleStarter runnable)
   {
      try
      {
         runnable.startModule();
      }
      catch (RuntimeException | IOException e)
      {
         PrintTools.error(this, "Failed to start a module in the network processor, stack trace:");
         e.printStackTrace();
      }
   }

   private interface ModuleStarter
   {
      void startModule() throws IOException;
   }
}
