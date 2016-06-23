package us.ihmc.quadrupedRobotics.factories;

import java.io.IOException;

import javax.vecmath.Point3d;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.partNames.QuadrupedJointName;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.quadrupedRobotics.communication.QuadrupedGlobalDataProducer;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSimulationController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerManager;
import us.ihmc.quadrupedRobotics.controller.forceDevelopment.QuadrupedForceDevelopmentControllerManager;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.sensorProcessing.simulatedSensors.SDFQuadrupedPerfectSimulatedSensor;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedStateEstimatorFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.model.QuadrupedSimulationInitialPositionParameters;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactParameters;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.sensorProcessing.model.DesiredJointDataHolder;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.gui.tools.VisualizerUtils;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.AlternatingSlopesGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.RollingGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.RotatablePlaneTerrainProfile;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;

public class QuadrupedSimulationFactory
{
   private SDFFullQuadrupedRobotModel fullRobotModel;
   private QuadrupedPhysicalProperties physicalProperties;
   private QuadrupedControlMode controlMode;
   private SDFRobot sdfRobot;
   private double controlDT;
   private double gravity;
   private int recordFrequency;
   private boolean useTrackAndDolly;
   private boolean showPlotter;
   private QuadrupedModelFactory modelFactory;
   private SimulationConstructionSetParameters scsParameters;
   private QuadrupedGroundContactModelType groundContactModelType;
   private QuadrupedGroundContactParameters groundContactParameters;
   private QuadrupedSimulationInitialPositionParameters initialPositionParameters;
   private QuadrupedRobotControllerFactory headControllerFactory;
   private QuadrupedControllerManager controllerManager;
   private OutputWriter outputWriter;
   private boolean useNetworking;
   private NetClassList netClassList;
   private SensorTimestampHolder timestampProvider;
   private PacketCommunicator packetCommunicator;
   private boolean useStateEstimator;
   private QuadrupedSensorInformation sensorInformation;
   private StateEstimatorParameters stateEstimatorParameters;
   private QuadrupedReferenceFrames referenceFrames;
   private SensorProcessingConfiguration sensorProcessingConfiguration;
   
   // TO CONSTRUCT
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead;
   private SensorReader sensorReader;
   private QuadrantDependentList<ContactablePlaneBody> contactableFeet;
   private QuadrantDependentList<FootSwitchInterface> footSwitches;
   private DRCKinematicsBasedStateEstimator stateEstimator;
   private GlobalDataProducer globalDataProducer;
   private RobotController headController;
   private DRCPoseCommunicator poseCommunicator;
   private GroundProfile3D groundProfile3D;
   private LinearGroundContactModel groundContactModel;
   private QuadrupedSimulationController simulationController;
   
   // CREATION
   
   private void setupYoRegistries()
   {
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);
      yoGraphicsListRegistryForDetachedOverhead = new YoGraphicsListRegistry();
   }

   private void createSensorReader()
   {
      if (useStateEstimator)
      {
         SixDoFJoint rootInverseDynamicsJoint = fullRobotModel.getRootJoint();
         IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
         ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
         ContactSensorHolder contactSensorHolder = null;
         RawJointSensorDataHolderMap rawJointSensorDataHolderMap = null;
         DesiredJointDataHolder estimatorDesiredJointDataHolder = null;

         SimulatedSensorHolderAndReaderFromRobotFactory sensorReaderFactory;
         sensorReaderFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(sdfRobot, sensorProcessingConfiguration);
         sensorReaderFactory.build(rootInverseDynamicsJoint, imuDefinitions, forceSensorDefinitions, contactSensorHolder, rawJointSensorDataHolderMap,
                                   estimatorDesiredJointDataHolder, sdfRobot.getRobotsYoVariableRegistry());

         sensorReader = sensorReaderFactory.getSensorReader();
      }
      else
      {
         sensorReader = new SDFQuadrupedPerfectSimulatedSensor(sdfRobot, fullRobotModel, referenceFrames);
      }
   }
   
   private void createContactibleFeet()
   {
      contactableFeet = QuadrupedStateEstimatorFactory.createFootContactableBodies(fullRobotModel, referenceFrames, physicalProperties);
   }
   
   private void createFootSwitches()
   {
      footSwitches = QuadrupedStateEstimatorFactory.createFootSwitches(contactableFeet, gravity, fullRobotModel, sdfRobot.getRobotsYoVariableRegistry());
   }
   
   private void createStateEstimator()
   {
      if (useStateEstimator)
      {
         stateEstimator = QuadrupedStateEstimatorFactory
               .createStateEstimator(sensorInformation, stateEstimatorParameters, fullRobotModel, sensorReader.getSensorOutputMapReadOnly(), contactableFeet,
                                     footSwitches, gravity, controlDT, sdfRobot.getRobotsYoVariableRegistry(), yoGraphicsListRegistry);
      }
      else
      {
         stateEstimator = null;
      }
   }
   
   private void createPacketCommunicator() throws IOException
   {
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, netClassList);
      packetCommunicator.connect();
   }
   
   private void createGlobalDataProducer()
   {
      if (useNetworking)
      {
         globalDataProducer = new QuadrupedGlobalDataProducer(packetCommunicator);
      }
   }
   
   private void createHeadController()
   {
      if (headControllerFactory != null)
      {
         headControllerFactory.setControlDt(controlDT);
         headControllerFactory.setFullRobotModel(fullRobotModel);
         headControllerFactory.setGlobalDataProducer(globalDataProducer);
      
         headController = headControllerFactory.createRobotController();
      }
   }
   
   public void createControllerManager() throws IOException
   {
      
      QuadrupedRuntimeEnvironment runtimeEnvironment = new QuadrupedRuntimeEnvironment(controlDT, sdfRobot.getYoTime(), fullRobotModel, sdfRobot.getRobotsYoVariableRegistry(),
                                                           yoGraphicsListRegistry, yoGraphicsListRegistryForDetachedOverhead, globalDataProducer, footSwitches);
      switch (controlMode)
      {
      case FORCE:
         controllerManager = new QuadrupedForceControllerManager(runtimeEnvironment, physicalProperties);
         break;
      case FORCE_DEV:
         controllerManager = new QuadrupedForceDevelopmentControllerManager(runtimeEnvironment, physicalProperties);
         break;
      case POSITION:
         controllerManager = null;
         break;
      case POSITION_DEV:
         controllerManager = null;
         break;
      default:
         controllerManager = null;
         break;
      }
   }
   
   private void createPoseCommunicator()
   {
      JointConfigurationGatherer jointConfigurationGathererAndProducer = new JointConfigurationGatherer(fullRobotModel);
      PeriodicThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler("PoseCommunicator");
      
      if (useNetworking)
      {
         poseCommunicator = new DRCPoseCommunicator(fullRobotModel, jointConfigurationGathererAndProducer, null, globalDataProducer,
                                                    timestampProvider, sensorReader.getSensorRawOutputMapReadOnly(),
                                                    controllerManager.getMotionStatusHolder(), null, scheduler, netClassList);
      }
      else
      {
         poseCommunicator = null;
      }
   }
   
   private void createGroundContactModel()
   {
      switch (groundContactModelType)
      {
      case FLAT:
         groundProfile3D = new FlatGroundProfile(0.0);
         break;
      case ROLLING_HILLS:
      groundProfile3D =  new RollingGroundProfile(0.025, 1.0, 0.0, -20.0, 20.0, -20.0, 20.0);
         break;
      case ROTATABLE:
         groundProfile3D = new RotatablePlaneTerrainProfile(new Point3d(), sdfRobot, yoGraphicsListRegistry, controlDT);
         break;
      case SLOPES:
         double xMin = -5.0, xMax = 40.0;
         double yMin = -5.0, yMax =  5.0;
         double[][] xSlopePairs = new double[][]
         {
            {1.0, 0.0}, {3.0, 0.1}
         };
         groundProfile3D = new AlternatingSlopesGroundProfile(xSlopePairs, xMin, xMax, yMin, yMax);
         break;
      default:
         groundProfile3D = null;
         break;
      }
      
      groundContactModel = new LinearGroundContactModel(sdfRobot, sdfRobot.getRobotsYoVariableRegistry());
      groundContactModel.setZStiffness(groundContactParameters.getZStiffness());
      groundContactModel.setZDamping(groundContactParameters.getZDamping());
      groundContactModel.setXYStiffness(groundContactParameters.getXYStiffness());
      groundContactModel.setXYDamping(groundContactParameters.getXYDamping());
      groundContactModel.setGroundProfile3D(groundProfile3D);
   }
   
   private void createSimulationController()
   {
      simulationController = new QuadrupedSimulationController(sdfRobot, sensorReader, outputWriter, controllerManager,
                                                               stateEstimator, poseCommunicator, headController);
   }
   
   private void setupSDFRobot()
   {
      sdfRobot.setController(simulationController);
      sdfRobot.setPositionInWorld(initialPositionParameters.getInitialBodyPosition());
      for (QuadrupedJointName quadrupedJointName : modelFactory.getQuadrupedJointNames())
      {
         OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = sdfRobot.getOneDegreeOfFreedomJoint(modelFactory.getSDFNameForJointName(quadrupedJointName));
         oneDegreeOfFreedomJoint.setQ(initialPositionParameters.getInitialJointPosition(quadrupedJointName));
      }
      try
      {
         sdfRobot.update();
         sdfRobot.doDynamicsButDoNotIntegrate();
         sdfRobot.update();
      }
      catch (UnreasonableAccelerationException unreasonableAccelerationException)
      {
         throw new RuntimeException("UnreasonableAccelerationException");
      }
      sdfRobot.setGravity(gravity);
      sdfRobot.setGroundContactModel(groundContactModel);
   }
   
   public SimulationConstructionSet createSimulation() throws IOException
   {
      setupYoRegistries();
      createSensorReader();
      createContactibleFeet();
      createFootSwitches();
      createStateEstimator();
      createPacketCommunicator();
      createGlobalDataProducer();
      createHeadController();
      createControllerManager();
      createPoseCommunicator();
      createGroundContactModel();
      createSimulationController();
      setupSDFRobot();
      
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot, scsParameters);
      if (groundContactModelType == QuadrupedGroundContactModelType.ROTATABLE)
      {
         scs.setGroundVisible(false);
      }
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      VisualizerUtils.createOverheadPlotter(scs, false, "centerOfMass", yoGraphicsListRegistry);
      VisualizerUtils.createOverheadPlotterInSeparateWindow(scs, false, "centerOfMass", yoGraphicsListRegistryForDetachedOverhead);
      scs.setDT(controlDT, recordFrequency);
      if (scs.getSimulationConstructionSetParameters().getCreateGUI())
      {
         scs.setCameraTrackingVars("q_x", "q_y", "q_z");
         scs.setCameraDollyVars("q_x", "q_y", "q_z");
         scs.setCameraTracking(useTrackAndDolly, useTrackAndDolly, useTrackAndDolly, useTrackAndDolly);
         scs.setCameraDolly(useTrackAndDolly, useTrackAndDolly, useTrackAndDolly, false);
         scs.setCameraDollyOffsets(4.0, 4.0, 1.0);
         if (showPlotter)
         {
            scs.getStandardSimulationGUI().selectPanel("Plotter");
         }
      }
      return scs;
   }
   
   // OPTIONS
   
   public void setControlDT(double controlDT)
   {
      this.controlDT = controlDT;
   }
   
   public void setGravity(double gravity)
   {
      this.gravity = gravity;
   }
   
   public void setRecordFrequency(int recordFrequency)
   {
      this.recordFrequency = recordFrequency;
   }
   
   public void setUseTrackAndDolly(boolean useTrackAndDolly)
   {
      this.useTrackAndDolly = useTrackAndDolly;
   }
   
   public void setShowPlotter(boolean showPlotter)
   {
      this.showPlotter = showPlotter;
   }
   
   public void setModelFactory(QuadrupedModelFactory modelFactory)
   {
      this.modelFactory = modelFactory;
   }
   
   public void setSCSParameters(SimulationConstructionSetParameters scsParameters)
   {
      this.scsParameters = scsParameters;
   }
   
   public void setGroundContactModelType(QuadrupedGroundContactModelType groundContactModelType)
   {
      this.groundContactModelType = groundContactModelType;
   }
   
   public void setGroundContactParameters(QuadrupedGroundContactParameters groundContactParameters)
   {
      this.groundContactParameters = groundContactParameters;
   }
   
   public void setHeadControllerFactory(QuadrupedRobotControllerFactory headControllerFactory)
   {
      this.headControllerFactory = headControllerFactory;
   }
   
   public void setOutputWriter(OutputWriter outputWriter)
   {
      this.outputWriter = outputWriter;
   }
   
   public void setInitialPositionParameters(QuadrupedSimulationInitialPositionParameters initialPositionParameters)
   {
      this.initialPositionParameters = initialPositionParameters;
   }
   
   public void setPhysicalProperties(QuadrupedPhysicalProperties physicalProperties)
   {
      this.physicalProperties = physicalProperties;
   }
   
   public void setControlMode(QuadrupedControlMode controlMode)
   {
      this.controlMode = controlMode;
   }
   
   public void setFullRobotModel(SDFFullQuadrupedRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
   }
   
   public void setGlobalDataProducer(GlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
   }
   
   public void setUseNetworking(boolean useNetworking)
   {
      this.useNetworking = useNetworking;
   }
   
   public void setNetClassList(NetClassList netClassList)
   {
      this.netClassList = netClassList;
   }
   
   public void setTimestampHolder(SensorTimestampHolder timestampProvider)
   {
      this.timestampProvider = timestampProvider;
   }

   public void setSDFRobot(SDFRobot sdfRobot)
   {
      this.sdfRobot = sdfRobot;
   }
   
   public void setUseStateEstimator(boolean useStateEstimator)
   {
      this.useStateEstimator = useStateEstimator;
   }

   public void setSensorInformation(QuadrupedSensorInformation sensorInformation)
   {
      this.sensorInformation = sensorInformation;
   }

   public void setStateEstimatorParameters(StateEstimatorParameters stateEstimatorParameters)
   {
      this.stateEstimatorParameters = stateEstimatorParameters;
   }

   public void setReferenceFrames(QuadrupedReferenceFrames referenceFrames)
   {
      this.referenceFrames = referenceFrames;
   }

   public void setSensorProcessingConfiguration(SensorProcessingConfiguration sensorProcessingConfiguration)
   {
      this.sensorProcessingConfiguration = sensorProcessingConfiguration;
   }
}
