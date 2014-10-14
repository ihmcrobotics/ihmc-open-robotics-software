package us.ihmc.atlas.drcsim;

import java.io.IOException;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DataProducerVariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.subscribers.ExternalPelvisPoseSubscriberInterface;
import us.ihmc.communication.subscribers.ExternalTimeStampedPoseSubscriber;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCControllerThread;
import us.ihmc.darpaRoboticsChallenge.DRCEstimatorThread;
import us.ihmc.darpaRoboticsChallenge.controllers.concurrent.ThreadDataSynchronizer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotDataCommunication.logger.YoVariableLoggerDispatcher;
import us.ihmc.robotDataCommunication.visualizer.SCSYoVariablesVisualizer;
import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.net.KryoObjectServer;

public class DRCSimControllerFactory
{

   private final AtlasSensorInformation sensorInformation;

   private static final double gravity = -9.81;

   public DRCSimControllerFactory()
   {
      boolean runningOnRealRobot = true;

      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS, false, true);
      /*
       * Create registries
       */
      sensorInformation = (AtlasSensorInformation) robotModel.getSensorInformation();

      AtlasContactPointParameters contactPointParameters = robotModel.getContactPointParameters();
      contactPointParameters.createHandKnobContactPoints();

      /*
       * Create network servers/clients
       */
      KryoObjectServer drcNetworkProcessorServer = new KryoObjectServer(NetworkConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT,
            new IHMCCommunicationKryoNetClassList());
      YoVariableServer yoVariableServer = new YoVariableServer(SCSYoVariablesVisualizer.defaultPort, robotModel.getEstimatorDT());
      GlobalDataProducer dataProducer = new GlobalDataProducer(drcNetworkProcessorServer);

      /*
       * Create controllers
       */
      MomentumBasedControllerFactory controllerFactory = createDRCControllerFactory(robotModel, dataProducer);
      /*
       * Create sensors
       */
      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();
      SensorNoiseParameters sensorNoiseParameters = stateEstimatorParameters.getSensorNoiseParameters();
      SensorFilterParameters sensorFilterParameters = stateEstimatorParameters.getSensorFilterParameters();

      DRCSimSensorReaderFactory sensorReaderFactory = new DRCSimSensorReaderFactory(sensorInformation, sensorFilterParameters, sensorNoiseParameters);

      /*
       * Create output writer
       */

      DRCSimOutputWriter outputWriter = new DRCSimOutputWriter();

      ExternalPelvisPoseSubscriberInterface externalPelvisPoseSubscriber = null;
      externalPelvisPoseSubscriber = new ExternalTimeStampedPoseSubscriber();
      dataProducer.attachListener(StampedPosePacket.class, externalPelvisPoseSubscriber);

      /*
       * Build controller
       */
      ThreadDataSynchronizer threadDataSynchronizer = new ThreadDataSynchronizer(robotModel);
      DRCEstimatorThread estimatorThread = new DRCEstimatorThread(robotModel, sensorReaderFactory, threadDataSynchronizer, dataProducer, yoVariableServer,
            gravity);
      estimatorThread.setExternelPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
      DRCControllerThread controllerThread = new DRCControllerThread(robotModel, controllerFactory, threadDataSynchronizer, outputWriter, dataProducer,
            yoVariableServer, gravity);

      /*
       * Setup threads
       */

      DRCSimThreadedRobotController robotController = new DRCSimThreadedRobotController();
      robotController.addController(estimatorThread, 1, false);
      int estimatorTicksPerSimulationTick = (int) Math.round(robotModel.getEstimatorDT() / robotModel.getEstimatorDT());
      int controllerTicksPerSimulationTick = (int) Math.round(robotModel.getControllerDT() / robotModel.getEstimatorDT());

      robotController.addController(estimatorThread, estimatorTicksPerSimulationTick, false);
      robotController.addController(controllerThread, controllerTicksPerSimulationTick, true);
    
      try
      {
         drcNetworkProcessorServer.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      yoVariableServer.start();
      if (robotModel.getNetworkParameters().getLoggingHostIP() != null)
      {
         YoVariableLoggerDispatcher.requestLogSession(robotModel.getNetworkParameters().getLoggingHostIP(), DRCSimControllerFactory.class.getSimpleName());
      }

      Thread simulationThread = new Thread(robotController);
      simulationThread.start();
      try
      {
         simulationThread.join();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }

   }

   private MomentumBasedControllerFactory createDRCControllerFactory(DRCRobotModel robotModel, GlobalDataProducer dataProducer)
   {
      ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();

      ArmControllerParameters armControllerParameters = robotModel.getArmControllerParameters();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      final HighLevelState initialBehavior;
      initialBehavior = HighLevelState.DO_NOTHING_BEHAVIOR; // HERE!!

      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForSlowWalkingOnRobot(walkingControllerParameters);

      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory,
            sensorInformation.getFeetForceSensorNames(), walkingControllerParameters, armControllerParameters, initialBehavior);

      VariousWalkingProviderFactory variousWalkingProviderFactory = new DataProducerVariousWalkingProviderFactory(dataProducer, footstepTimingParameters);
      controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);

      return controllerFactory;
   }

   
   public static void main(String[] args)
   {
      new DRCSimControllerFactory();
   }
}
