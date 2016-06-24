package us.ihmc.quadrupedRobotics.factories;

import java.io.IOException;
import java.net.BindException;

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
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;

public class QuadrupedSimulationFactory
{
   private RequiredFactoryField<SDFFullQuadrupedRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private RequiredFactoryField<QuadrupedPhysicalProperties> physicalProperties = new RequiredFactoryField<>("physicalProperties");
   private RequiredFactoryField<QuadrupedControlMode> controlMode = new RequiredFactoryField<>("controlMode");
   private RequiredFactoryField<SDFRobot> sdfRobot = new RequiredFactoryField<>("sdfRobot");
   private RequiredFactoryField<Double> controlDT = new RequiredFactoryField<>("controlDT");
   private RequiredFactoryField<Double> gravity = new RequiredFactoryField<>("gravity");
   private RequiredFactoryField<Integer> recordFrequency = new RequiredFactoryField<>("recordFrequency");
   private RequiredFactoryField<Boolean> useTrackAndDolly = new RequiredFactoryField<>("useTrackAndDolly");
   private RequiredFactoryField<Boolean> showPlotter = new RequiredFactoryField<>("showPlotter");
   private RequiredFactoryField<QuadrupedModelFactory> modelFactory = new RequiredFactoryField<>("modelFactory");
   private RequiredFactoryField<SimulationConstructionSetParameters> scsParameters = new RequiredFactoryField<>("scsParameters");
   private RequiredFactoryField<QuadrupedGroundContactModelType> groundContactModelType = new RequiredFactoryField<>("groundContactModelType");
   private RequiredFactoryField<QuadrupedGroundContactParameters> groundContactParameters = new RequiredFactoryField<>("groundContactParameters");
   private RequiredFactoryField<QuadrupedSimulationInitialPositionParameters> initialPositionParameters = new RequiredFactoryField<>("initialPositionParameters");
   private RequiredFactoryField<OutputWriter> outputWriter = new RequiredFactoryField<>("outputWriter");
   private RequiredFactoryField<Boolean> useNetworking = new RequiredFactoryField<>("useNetworking");
   private RequiredFactoryField<NetClassList> netClassList = new RequiredFactoryField<>("netClassList");
   private RequiredFactoryField<SensorTimestampHolder> timestampProvider = new RequiredFactoryField<>("timestampProvider");
   private RequiredFactoryField<Boolean> useStateEstimator = new RequiredFactoryField<>("useStateEstimator");
   private RequiredFactoryField<QuadrupedSensorInformation> sensorInformation = new RequiredFactoryField<>("sensorInformation");
   private RequiredFactoryField<StateEstimatorParameters> stateEstimatorParameters = new RequiredFactoryField<>("stateEstimatorParameters");
   private RequiredFactoryField<QuadrupedReferenceFrames> referenceFrames = new RequiredFactoryField<>("referenceFrames");
   
   private OptionalFactoryField<QuadrupedRobotControllerFactory> headControllerFactory = new OptionalFactoryField<>("headControllerFactory");
   
   // TO CONSTRUCT
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead;
   private SensorReader sensorReader;
   private QuadrantDependentList<ContactablePlaneBody> contactableFeet;
   private QuadrantDependentList<FootSwitchInterface> footSwitches;
   private DRCKinematicsBasedStateEstimator stateEstimator;
   private PacketCommunicator packetCommunicator;
   private GlobalDataProducer globalDataProducer;
   private RobotController headController;
   private QuadrupedControllerManager controllerManager;
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
      if (useStateEstimator.get())
      {
         SixDoFJoint rootInverseDynamicsJoint = fullRobotModel.get().getRootJoint();
         IMUDefinition[] imuDefinitions = fullRobotModel.get().getIMUDefinitions();
         ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.get().getForceSensorDefinitions();
         ContactSensorHolder contactSensorHolder = null;
         RawJointSensorDataHolderMap rawJointSensorDataHolderMap = null;
         DesiredJointDataHolder estimatorDesiredJointDataHolder = null;

         SimulatedSensorHolderAndReaderFromRobotFactory sensorReaderFactory;
         sensorReaderFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(sdfRobot.get(), stateEstimatorParameters.get());
         sensorReaderFactory.build(rootInverseDynamicsJoint, imuDefinitions, forceSensorDefinitions, contactSensorHolder, rawJointSensorDataHolderMap,
                                   estimatorDesiredJointDataHolder, sdfRobot.get().getRobotsYoVariableRegistry());

         sensorReader = sensorReaderFactory.getSensorReader();
      }
      else
      {
         sensorReader = new SDFQuadrupedPerfectSimulatedSensor(sdfRobot.get(), fullRobotModel.get(), referenceFrames.get());
      }
   }
   
   private void createContactibleFeet()
   {
      contactableFeet = QuadrupedStateEstimatorFactory.createFootContactableBodies(fullRobotModel.get(), referenceFrames.get(), physicalProperties.get());
   }
   
   private void createFootSwitches()
   {
      footSwitches = QuadrupedStateEstimatorFactory.createFootSwitches(contactableFeet, gravity.get(), fullRobotModel.get(), sdfRobot.get().getRobotsYoVariableRegistry());
   }
   
   private void createStateEstimator()
   {
      if (useStateEstimator.get())
      {
         stateEstimator = QuadrupedStateEstimatorFactory
               .createStateEstimator(sensorInformation.get(), stateEstimatorParameters.get(), fullRobotModel.get(), sensorReader.getSensorOutputMapReadOnly(), contactableFeet,
                                     footSwitches, gravity.get(), controlDT.get(), sdfRobot.get().getRobotsYoVariableRegistry(), yoGraphicsListRegistry);
      }
      else
      {
         stateEstimator = null;
      }
   }
   
   private void createPacketCommunicator() throws IOException
   {
      try
      {
         packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, netClassList.get());
         packetCommunicator.connect();
      }
      catch (BindException bindException)
      {
         PrintTools.error(this, bindException.getMessage());
         PrintTools.warn(this, "Continuing without networking");
         useNetworking.set(false);
      }
   }
   
   private void createGlobalDataProducer()
   {
      if (useNetworking.get())
      {
         globalDataProducer = new QuadrupedGlobalDataProducer(packetCommunicator);
      }
   }
   
   private void createHeadController()
   {
      if (headControllerFactory.hasBeenSet())
      {
         headControllerFactory.get().setControlDt(controlDT.get());
         headControllerFactory.get().setFullRobotModel(fullRobotModel.get());
         headControllerFactory.get().setGlobalDataProducer(globalDataProducer);
      
         headController = headControllerFactory.get().createRobotController();
      }
   }
   
   public void createControllerManager() throws IOException
   {
      
      QuadrupedRuntimeEnvironment runtimeEnvironment = new QuadrupedRuntimeEnvironment(controlDT.get(), sdfRobot.get().getYoTime(), fullRobotModel.get(), sdfRobot.get().getRobotsYoVariableRegistry(),
                                                           yoGraphicsListRegistry, yoGraphicsListRegistryForDetachedOverhead, globalDataProducer, footSwitches);
      switch (controlMode.get())
      {
      case FORCE:
         controllerManager = new QuadrupedForceControllerManager(runtimeEnvironment, physicalProperties.get());
         break;
      case FORCE_DEV:
         controllerManager = new QuadrupedForceDevelopmentControllerManager(runtimeEnvironment, physicalProperties.get());
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
      JointConfigurationGatherer jointConfigurationGathererAndProducer = new JointConfigurationGatherer(fullRobotModel.get());
      PeriodicThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler("PoseCommunicator");
      
      if (useNetworking.get())
      {
         poseCommunicator = new DRCPoseCommunicator(fullRobotModel.get(), jointConfigurationGathererAndProducer, null, globalDataProducer,
                                                    timestampProvider.get(), sensorReader.getSensorRawOutputMapReadOnly(),
                                                    controllerManager.getMotionStatusHolder(), null, scheduler, netClassList.get());
      }
      else
      {
         poseCommunicator = null;
      }
   }
   
   private void createGroundContactModel()
   {
      switch (groundContactModelType.get())
      {
      case FLAT:
         groundProfile3D = new FlatGroundProfile(0.0);
         break;
      case ROLLING_HILLS:
      groundProfile3D =  new RollingGroundProfile(0.025, 1.0, 0.0, -20.0, 20.0, -20.0, 20.0);
         break;
      case ROTATABLE:
         groundProfile3D = new RotatablePlaneTerrainProfile(new Point3d(), sdfRobot.get(), yoGraphicsListRegistry, controlDT.get());
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
      
      groundContactModel = new LinearGroundContactModel(sdfRobot.get(), sdfRobot.get().getRobotsYoVariableRegistry());
      groundContactModel.setZStiffness(groundContactParameters.get().getZStiffness());
      groundContactModel.setZDamping(groundContactParameters.get().getZDamping());
      groundContactModel.setXYStiffness(groundContactParameters.get().getXYStiffness());
      groundContactModel.setXYDamping(groundContactParameters.get().getXYDamping());
      groundContactModel.setGroundProfile3D(groundProfile3D);
   }
   
   private void createSimulationController()
   {
      simulationController = new QuadrupedSimulationController(sdfRobot.get(), sensorReader, outputWriter.get(), controllerManager,
                                                               stateEstimator, poseCommunicator, headController);
   }
   
   private void setupSDFRobot()
   {
      sdfRobot.get().setController(simulationController);
      sdfRobot.get().setPositionInWorld(initialPositionParameters.get().getInitialBodyPosition());
      for (QuadrupedJointName quadrupedJointName : modelFactory.get().getQuadrupedJointNames())
      {
         OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = sdfRobot.get().getOneDegreeOfFreedomJoint(modelFactory.get().getSDFNameForJointName(quadrupedJointName));
         oneDegreeOfFreedomJoint.setQ(initialPositionParameters.get().getInitialJointPosition(quadrupedJointName));
      }
      try
      {
         sdfRobot.get().update();
         sdfRobot.get().doDynamicsButDoNotIntegrate();
         sdfRobot.get().update();
      }
      catch (UnreasonableAccelerationException unreasonableAccelerationException)
      {
         throw new RuntimeException("UnreasonableAccelerationException");
      }
      double totalMass = sdfRobot.get().computeCenterOfMass(initialPositionParameters.get().getInitialBodyPosition());
      sdfRobot.get().setGravity(gravity.get());
      sdfRobot.get().setGroundContactModel(groundContactModel);
      System.out.println("Total mass: " + totalMass);
   }
   
   public SimulationConstructionSet createSimulation() throws IOException
   {
      FactoryTools.checkAllRequiredFactoryFieldsAreSet(this);
      
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
      
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot.get(), scsParameters.get());
      if (groundContactModelType.get() == QuadrupedGroundContactModelType.ROTATABLE)
      {
         scs.setGroundVisible(false);
      }
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      VisualizerUtils.createOverheadPlotter(scs, false, "centerOfMass", yoGraphicsListRegistry);
      VisualizerUtils.createOverheadPlotterInSeparateWindow(scs, false, "centerOfMass", yoGraphicsListRegistryForDetachedOverhead);
      scs.setDT(controlDT.get(), recordFrequency.get());
      if (scs.getSimulationConstructionSetParameters().getCreateGUI())
      {
         scs.setCameraTrackingVars("q_x", "q_y", "q_z");
         scs.setCameraDollyVars("q_x", "q_y", "q_z");
         scs.setCameraTracking(useTrackAndDolly.get(), useTrackAndDolly.get(), useTrackAndDolly.get(), useTrackAndDolly.get());
         scs.setCameraDolly(useTrackAndDolly.get(), useTrackAndDolly.get(), useTrackAndDolly.get(), false);
         scs.setCameraDollyOffsets(4.0, 4.0, 1.0);
         if (showPlotter.get())
         {
            scs.getStandardSimulationGUI().selectPanel("Plotter");
         }
      }
      return scs;
   }
   
   // OPTIONS
   
   public void setControlDT(double controlDT)
   {
      this.controlDT.set(controlDT);
   }
   
   public void setGravity(double gravity)
   {
      this.gravity.set(gravity);
   }
   
   public void setRecordFrequency(int recordFrequency)
   {
      this.recordFrequency.set(recordFrequency);
   }
   
   public void setUseTrackAndDolly(boolean useTrackAndDolly)
   {
      this.useTrackAndDolly.set(useTrackAndDolly);
   }
   
   public void setShowPlotter(boolean showPlotter)
   {
      this.showPlotter.set(showPlotter);
   }
   
   public void setModelFactory(QuadrupedModelFactory modelFactory)
   {
      this.modelFactory.set(modelFactory);
   }
   
   public void setSCSParameters(SimulationConstructionSetParameters scsParameters)
   {
      this.scsParameters.set(scsParameters);
   }
   
   public void setGroundContactModelType(QuadrupedGroundContactModelType groundContactModelType)
   {
      this.groundContactModelType.set(groundContactModelType);
   }
   
   public void setGroundContactParameters(QuadrupedGroundContactParameters groundContactParameters)
   {
      this.groundContactParameters.set(groundContactParameters);
   }
   
   public void setHeadControllerFactory(QuadrupedRobotControllerFactory headControllerFactory)
   {
      this.headControllerFactory.set(headControllerFactory);
   }
   
   public void setOutputWriter(OutputWriter outputWriter)
   {
      this.outputWriter.set(outputWriter);
   }
   
   public void setInitialPositionParameters(QuadrupedSimulationInitialPositionParameters initialPositionParameters)
   {
      this.initialPositionParameters.set(initialPositionParameters);
   }
   
   public void setPhysicalProperties(QuadrupedPhysicalProperties physicalProperties)
   {
      this.physicalProperties.set(physicalProperties);
   }
   
   public void setControlMode(QuadrupedControlMode controlMode)
   {
      this.controlMode.set(controlMode);
   }
   
   public void setFullRobotModel(SDFFullQuadrupedRobotModel fullRobotModel)
   {
      this.fullRobotModel.set(fullRobotModel);
   }
   
   public void setUseNetworking(boolean useNetworking)
   {
      this.useNetworking.set(useNetworking);
   }
   
   public void setNetClassList(NetClassList netClassList)
   {
      this.netClassList.set(netClassList);
   }
   
   public void setTimestampHolder(SensorTimestampHolder timestampProvider)
   {
      this.timestampProvider.set(timestampProvider);
   }

   public void setSDFRobot(SDFRobot sdfRobot)
   {
      this.sdfRobot.set(sdfRobot);
   }
   
   public void setUseStateEstimator(boolean useStateEstimator)
   {
      this.useStateEstimator.set(useStateEstimator);
   }

   public void setSensorInformation(QuadrupedSensorInformation sensorInformation)
   {
      this.sensorInformation.set(sensorInformation);
   }

   public void setStateEstimatorParameters(StateEstimatorParameters stateEstimatorParameters)
   {
      this.stateEstimatorParameters.set(stateEstimatorParameters);
   }

   public void setReferenceFrames(QuadrupedReferenceFrames referenceFrames)
   {
      this.referenceFrames.set(referenceFrames);
   }
}
