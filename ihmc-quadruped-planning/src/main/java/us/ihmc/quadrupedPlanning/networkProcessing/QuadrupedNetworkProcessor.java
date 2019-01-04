package us.ihmc.quadrupedPlanning.networkProcessing;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedPlanning.networkProcessing.bodyTeleop.QuadrupedBodyTeleopModule;
import us.ihmc.quadrupedPlanning.networkProcessing.heightTeleop.QuadrupedBodyHeightTeleopModule;
import us.ihmc.quadrupedPlanning.networkProcessing.stepTeleop.QuadrupedStepTeleopModule;
import us.ihmc.quadrupedPlanning.networkProcessing.xBox.QuadrupedXBoxModule;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class QuadrupedNetworkProcessor
{
   private final boolean DEBUG = false;
   private QuadrupedStepTeleopModule stepTeleopModule;

   private final List<QuadrupedToolboxModule> modules = new ArrayList<>();

   public QuadrupedNetworkProcessor(FullQuadrupedRobotModelFactory robotModel, QuadrupedNetworkModuleParameters params, double nominalHeight,
                                    QuadrupedXGaitSettings xGaitSettings, PointFootSnapperParameters pointFootSnapperParameters)
   {
      this(robotModel, params, nominalHeight, xGaitSettings, pointFootSnapperParameters, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public QuadrupedNetworkProcessor(FullQuadrupedRobotModelFactory robotModel, QuadrupedNetworkModuleParameters params, double nominalHeight,
                                    QuadrupedXGaitSettingsReadOnly xGaitSettings, PointFootSnapperParameters pointFootSnapperParameters,
                                    DomainFactory.PubSubImplementation pubSubImplementation)
   {
      tryToStartModule(() -> setupStepTeleopModule(robotModel, xGaitSettings, pointFootSnapperParameters, params, pubSubImplementation));
      tryToStartModule(() -> setupBodyHeightTeleopModule(robotModel, params, nominalHeight, pubSubImplementation));
      tryToStartModule(() -> setupBodyTeleopModule(robotModel, params, pubSubImplementation));
      tryToStartModule(() -> setupXBoxModule(robotModel, params, xGaitSettings, nominalHeight, pubSubImplementation));
      tryToStartModule(() -> setupRobotEnvironmentAwerenessModule(params, pubSubImplementation));
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
   }

   private void setupStepTeleopModule(FullQuadrupedRobotModelFactory modelFactory, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                      PointFootSnapperParameters pointFootSnapperParameters, QuadrupedNetworkModuleParameters params,
                                      DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (!params.isStepTeleopModuleEnabled())
         return;
      stepTeleopModule = new QuadrupedStepTeleopModule(modelFactory, xGaitSettings, pointFootSnapperParameters, null, params.visualizeStepTeleopModuleEnabled(),
                                                       pubSubImplementation);
      modules.add(stepTeleopModule);
   }

   private void setupBodyHeightTeleopModule(FullQuadrupedRobotModelFactory modelFactory, QuadrupedNetworkModuleParameters params, double nominalHeight,
                                            DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (!params.isBodyHeightTeleopModuleEnabled())
         return;
      modules.add(new QuadrupedBodyHeightTeleopModule(modelFactory, nominalHeight, null, pubSubImplementation));
   }

   private void setupBodyTeleopModule(FullQuadrupedRobotModelFactory modelFactory, QuadrupedNetworkModuleParameters params,
                                      DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (!params.isBodyTeleopModuleEnabled())
         return;
      modules.add(new QuadrupedBodyTeleopModule(modelFactory, null, pubSubImplementation));
   }

   private void setupXBoxModule(FullQuadrupedRobotModelFactory modelFactory, QuadrupedNetworkModuleParameters params,
                                QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double nominalBodyHeight,
                                DomainFactory.PubSubImplementation pubSubImplementation) throws IOException
   {
      if (!params.isXBoxModuleEnabled())
         return;
      modules.add(new QuadrupedXBoxModule(modelFactory, defaultXGaitSettings, nominalBodyHeight, null, pubSubImplementation));
   }

   private void setupRobotEnvironmentAwerenessModule(QuadrupedNetworkModuleParameters params, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (params.isRobotEnvironmentAwerenessModuleEnabled())
      {
         try
         {
            if (pubSubImplementation == DomainFactory.PubSubImplementation.FAST_RTPS)
               LIDARBasedREAModule.createRemoteModule(System.getProperty("user.home") + "/.ihmc/Configurations/defaultREAModuleConfiguration.txt").start();
            else
               LIDARBasedREAModule.createIntraprocessModule(System.getProperty("user.home") + "/.ihmc/Configurations/defaultREAModuleConfiguration.txt")
                                  .start();

         }
         catch (Exception e)
         {
            throw new RuntimeException(e);
         }
      }
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
