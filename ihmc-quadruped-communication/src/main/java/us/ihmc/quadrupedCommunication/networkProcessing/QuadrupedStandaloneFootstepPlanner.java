package us.ihmc.quadrupedCommunication.networkProcessing;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.networkProcessing.bodyTeleop.QuadrupedBodyTeleopModule;
import us.ihmc.quadrupedCommunication.networkProcessing.footstepPlanning.QuadrupedFootstepPlanningModule;
import us.ihmc.quadrupedCommunication.networkProcessing.heightTeleop.QuadrupedBodyHeightTeleopModule;
import us.ihmc.quadrupedCommunication.networkProcessing.stepTeleop.QuadrupedStepTeleopModule;
import us.ihmc.quadrupedCommunication.networkProcessing.xBox.QuadrupedXBoxModule;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class QuadrupedStandaloneFootstepPlanner
{
   private final boolean DEBUG = false;

   public static final int footstepPlanningPort = 8007;

   private final List<QuadrupedToolboxModule> modules = new ArrayList<>();
   private QuadrupedSupportPlanarRegionPublisher supportPublisher = null;

   public QuadrupedStandaloneFootstepPlanner(FullQuadrupedRobotModelFactory robotModel,  FootstepPlannerParameters footstepPlannerParameters, QuadrupedXGaitSettings xGaitSettings,
                                             PointFootSnapperParameters pointFootSnapperParameters)
   {
      this(robotModel, footstepPlannerParameters, xGaitSettings, pointFootSnapperParameters,
           DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public QuadrupedStandaloneFootstepPlanner(FullQuadrupedRobotModelFactory robotModel,
                                             FootstepPlannerParameters footstepPlannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                             PointFootSnapperParameters pointFootSnapperParameters, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this(robotModel, null, footstepPlannerParameters, xGaitSettings, pointFootSnapperParameters, pubSubImplementation);
   }

   public QuadrupedStandaloneFootstepPlanner(FullQuadrupedRobotModelFactory robotModel, LogModelProvider logModelProvider,
                                             FootstepPlannerParameters footstepPlannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                             PointFootSnapperParameters pointFootSnapperParameters, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      tryToStartModule(() -> setupFootstepPlanningModule(robotModel, footstepPlannerParameters, xGaitSettings, pointFootSnapperParameters, logModelProvider,
                                                         pubSubImplementation));
   }

   public void setRootRegistry(YoVariableRegistry rootRegistry, YoGraphicsListRegistry rootGraphicsListRegistry)
   {
      for (QuadrupedToolboxModule module : modules)
      {
         module.setRootRegistry(rootRegistry, rootGraphicsListRegistry);
      }
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



   private void setupFootstepPlanningModule(FullQuadrupedRobotModelFactory modelFactory, FootstepPlannerParameters footstepPlannerParameters,
                                            QuadrupedXGaitSettingsReadOnly xGaitSettings, PointFootSnapperParameters pointFootSnapperParameters,
                                            LogModelProvider logModelProvider, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      modules.add(new QuadrupedFootstepPlanningModule(modelFactory, footstepPlannerParameters, xGaitSettings, pointFootSnapperParameters, logModelProvider,
                                                      true, false,
                                                      pubSubImplementation));
   }


   protected void connect(PacketCommunicator communicator)
   {
      try
      {
         communicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void printModuleConnectedDebugStatement(PacketDestination destination, String methodName)
   {
      if (DEBUG)
      {
         PrintTools.debug(this, methodName + ": " + destination);
      }
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
