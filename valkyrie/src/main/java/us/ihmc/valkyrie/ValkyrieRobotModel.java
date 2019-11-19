package us.ihmc.valkyrie;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.drcRobot.shapeContactSettings.DRCRobotModelShapeCollisionSettings;
import us.ihmc.avatar.factory.SimulatedHandControlTask;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.HumanoidRobotKinematicsCollisionModel;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.avatar.ros.WallTimeBasedROSClockCalculator;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SliderBoardParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.PlanarRegionFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.QuadTreeFootstepPlanningParameters;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.modelFileLoaders.SdfLoader.DRCRobotSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.SDFContactSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.modelFileLoaders.SdfLoader.SDFForceSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFModelLoader;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFVisual;
import us.ihmc.multicastLogDataProtocol.modelLoaders.DefaultLogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFromDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ContactSensorType;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.fingers.SimulatedValkyrieFingerController;
import us.ihmc.valkyrie.fingers.ValkyrieHandModel;
import us.ihmc.valkyrie.parameters.ValkyrieCollisionBoxProvider;
import us.ihmc.valkyrie.parameters.ValkyrieContactPointParameters;
import us.ihmc.valkyrie.parameters.ValkyrieFootstepPlannerParameters;
import us.ihmc.valkyrie.parameters.ValkyrieFootstepPlanningParameters;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyriePlanarRegionFootstepPlannerParameters;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.valkyrie.parameters.ValkyrieSliderBoardParameters;
import us.ihmc.valkyrie.parameters.ValkyrieSmoothCMPPlannerParameters;
import us.ihmc.valkyrie.parameters.ValkyrieStateEstimatorParameters;
import us.ihmc.valkyrie.parameters.ValkyrieUIParameters;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;
import us.ihmc.valkyrie.sensors.ValkyrieSensorSuiteManager;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.UIParameters;

public class ValkyrieRobotModel implements DRCRobotModel, SDFDescriptionMutator
{
   private static final boolean PRINT_MODEL = false;

   private final String[] resourceDirectories = {"models/", "models/gazebo/", "models/val_description/", "models/val_description/sdf/"};

   private final ValkyrieRobotVersion robotVersion;
   private final RobotTarget target;

   private ValkyrieJointMap jointMap;
   private ValkyrieContactPointParameters contactPointParameters;
   private ValkyrieSensorInformation sensorInformation;
   private HighLevelControllerParameters highLevelControllerParameters;
   private ValkyrieCalibrationParameters calibrationParameters;

   private PlanarRegionFootstepPlanningParameters planarRegionFootstepPlanningParameters;
   private ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters;
   private WalkingControllerParameters walkingControllerParameters;
   private StateEstimatorParameters stateEstimatorParamaters;

   private final WallTimeBasedROSClockCalculator rosClockCalculator = new WallTimeBasedROSClockCalculator();

   private boolean useShapeCollision = false;
   private final boolean useOBJGraphics;
   private final double transparency;
   private final String customModel;
   private FootContactPoints<RobotSide> simulationContactPoints;
   private JaxbSDFLoader loader;
   private RobotDescription robotDescription;

   public ValkyrieRobotModel(RobotTarget target, FootContactPoints<RobotSide> simulationContactPoints)
   {
      this(target, ValkyrieRobotVersion.DEFAULT, null, simulationContactPoints, false);
   }

   public ValkyrieRobotModel(RobotTarget target, boolean headless)
   {
      this(target, ValkyrieRobotVersion.DEFAULT, null, null, false);
   }

   public ValkyrieRobotModel(RobotTarget target, ValkyrieRobotVersion robotVersion)
   {
      this(target, robotVersion, null, null, false);
   }

   public ValkyrieRobotModel(RobotTarget target, ValkyrieRobotVersion robotVersion, String model)
   {
      this(target, robotVersion, model, null, false);
   }

   public ValkyrieRobotModel(RobotTarget target, String model, FootContactPoints<RobotSide> simulationContactPoints)
   {
      this(target, ValkyrieRobotVersion.DEFAULT, model, simulationContactPoints, false);
   }

   public ValkyrieRobotModel(RobotTarget target, ValkyrieRobotVersion robotVersion, String model, FootContactPoints<RobotSide> simulationContactPoints,
                             boolean useShapeCollision)
   {
      this(target, robotVersion, model, simulationContactPoints, useShapeCollision, true);
   }

   public ValkyrieRobotModel(RobotTarget target, ValkyrieRobotVersion robotVersion, String model, FootContactPoints<RobotSide> simulationContactPoints,
                             boolean useShapeCollision, boolean useOBJGraphics)
   {
      this(target, robotVersion, model, simulationContactPoints, useShapeCollision, useOBJGraphics, Double.NaN);
   }

   public ValkyrieRobotModel(RobotTarget target, ValkyrieRobotVersion robotVersion, String customModel, FootContactPoints<RobotSide> simulationContactPoints,
                             boolean useShapeCollision, boolean useOBJGraphics, double transparency)
   {
      this.target = target;
      this.robotVersion = robotVersion;
      this.customModel = customModel;
      this.simulationContactPoints = simulationContactPoints;
      this.useShapeCollision = useShapeCollision;
      this.useOBJGraphics = useOBJGraphics;
      this.transparency = transparency;
   }

   public ValkyrieRobotVersion getRobotVersion()
   {
      return robotVersion;
   }

   public RobotTarget getTarget()
   {
      return target;
   }

   @Override
   public ValkyrieJointMap getJointMap()
   {
      if (jointMap == null)
         jointMap = new ValkyrieJointMap(robotVersion);
      return jointMap;
   }

   @Override
   public DRCRobotModelShapeCollisionSettings getShapeCollisionSettings()
   {
      return new ValkyrieRobotModelShapeCollisionSettings(useShapeCollision);
   }

   @Override
   public InputStream getWholeBodyControllerParametersFile()
   {
      switch (target)
      {
         case SCS:
            return getClass().getResourceAsStream("/us/ihmc/valkyrie/parameters/controller_simulation.xml");
         case GAZEBO:
         case REAL_ROBOT:
            return getClass().getResourceAsStream("/us/ihmc/valkyrie/parameters/controller_hardware.xml");
         default:
            throw new UnsupportedOperationException("Unsupported target: " + target);
      }
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      if (robotDescription == null)
      {
         boolean useCollisionMeshes = false;

         GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
         RobotDescriptionFromSDFLoader descriptionLoader = new RobotDescriptionFromSDFLoader();

         if (useShapeCollision)
         {
            robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, jointMap, null, false, transparency);
            ValkyrieCollisionMeshDefinitionDataHolder collisionMeshDefinitionDataHolder = new ValkyrieCollisionMeshDefinitionDataHolder(jointMap);
            collisionMeshDefinitionDataHolder.setVisible(false);

            robotDescription.addCollisionMeshDefinitionData(collisionMeshDefinitionDataHolder);
         }
         else
         {
            robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel,
                                                                             jointMap,
                                                                             getContactPointParameters(),
                                                                             useCollisionMeshes,
                                                                             transparency);
         }
      }
      return robotDescription;
   }

   private String[] getResourceDirectories()
   {
      return resourceDirectories;
   }

   private InputStream sdfModelInputStream = null;

   private InputStream getSDFModelInputStream()
   {
      if (sdfModelInputStream == null)
      {
         InputStream inputStream = null;

         if (customModel != null)
         {

            System.out.println("Loading robot model from: '" + customModel + "'");
            inputStream = getClass().getClassLoader().getResourceAsStream(customModel);

            if (inputStream == null)
            {
               try
               {
                  inputStream = new FileInputStream(customModel);
               }
               catch (FileNotFoundException e)
               {
                  throw new RuntimeException(e);
               }
            }
         }
         else
         {
            String sdfFile = null;

            if (target == RobotTarget.REAL_ROBOT)
               sdfFile = robotVersion.getRealRobotSdfFile();
            else
               sdfFile = robotVersion.getSimSdfFile();

            sdfModelInputStream = getClass().getClassLoader().getResourceAsStream(sdfFile);
         }

      }
      return sdfModelInputStream;
   }

   @Override
   public String toString()
   {
      return getSimpleRobotName();
   }

   @Override
   public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new ValkyrieInitialSetup(groundHeight, initialYaw);
   }

   @Override
   public ValkyrieHandModel getHandModel()
   {
      return new ValkyrieHandModel();
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel()
   {
      return new FullHumanoidRobotModelFromDescription(robotDescription, jointMap, sensorInformation.getSensorFramesToTrack());
   }

   @Override
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
   {
      boolean enableTorqueVelocityLimits = false;

      HumanoidFloatingRootJointRobot sdfRobot = new HumanoidFloatingRootJointRobot(robotDescription, jointMap, enableJointDamping, enableTorqueVelocityLimits);

      if (PRINT_MODEL)
      {
         System.out.println("\nValkyrieRobotModel Link Masses:");

         StringBuffer stringBuffer = new StringBuffer();
         sdfRobot.printRobotJointsAndMasses(stringBuffer);
         System.out.println(stringBuffer);
         System.out.println();

         System.out.println("ValkyrieRobotModel: \n" + sdfRobot);
      }

      return sdfRobot;
   }

   @Override
   public double getSimulateDT()
   {
      return getEstimatorDT() / 3.0;
   }

   @Override
   public double getEstimatorDT()
   {
      return 0.002;
   }

   @Override
   public double getControllerDT()
   {
      if (target == RobotTarget.SCS)
         return 0.004;
      else
         return 0.006;
   }

   public GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      if (loader == null)
      {
         loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), getSDFModelInputStream(), this);

         for (String forceSensorName : ValkyrieSensorInformation.forceSensorNames)
         {
            RigidBodyTransform transform = new RigidBodyTransform(ValkyrieSensorInformation.getForceSensorTransform(forceSensorName));
            loader.addForceSensor(jointMap, forceSensorName, forceSensorName, transform);
         }

         for (RobotSide side : RobotSide.values)
         {
            LinkedHashMap<String, LinkedHashMap<String, ContactSensorType>> contactsensors = ValkyrieSensorInformation.contactSensors.get(side);

            for (String parentJointName : contactsensors.keySet())
            {
               for (String sensorName : contactsensors.get(parentJointName).keySet())
               {
                  loader.addContactSensor(jointMap, sensorName, parentJointName, contactsensors.get(parentJointName).get(sensorName));
               }
            }
         }
      }

      return loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
   }

   @Override
   public RobotROSClockCalculator getROSClockCalculator()
   {
      return rosClockCalculator;
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager()
   {
      return new ValkyrieSensorSuiteManager(getSimpleRobotName(),
                                            this,
                                            getCollisionBoxProvider(),
                                            getROSClockCalculator(),
                                            sensorInformation,
                                            jointMap,
                                            target);
   }

   @Override
   public SimulatedHandControlTask createSimulatedHandController(FloatingRootJointRobot simulatedRobot, RealtimeRos2Node realtimeRos2Node)
   {
      if (!robotVersion.hasFingers())
         return null;

      boolean hasFingers = true;
      for (RobotSide robotSide : RobotSide.values)
      {
         String handLinkName = jointMap.getHandName(robotSide);
         hasFingers = hasFingers && !simulatedRobot.getLink(handLinkName).getParentJoint().getChildrenJoints().isEmpty();
      }

      if (hasFingers)
      {
         return new SimulatedValkyrieFingerController(simulatedRobot,
                                                      realtimeRos2Node,
                                                      this,
                                                      ControllerAPIDefinition.getPublisherTopicNameGenerator(getSimpleRobotName()),
                                                      ControllerAPIDefinition.getSubscriberTopicNameGenerator(getSimpleRobotName()));
      }
      else
      {
         return null;
      }
   }

   @Override
   public QuadTreeFootstepPlanningParameters getQuadTreeFootstepPlanningParameters()
   {
      return new ValkyrieFootstepPlanningParameters();
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new DefaultLogModelProvider<>(SDFModelLoader.class, jointMap.getModelName(), getSDFModelInputStream(), getResourceDirectories());
   }

   @Override
   public DataServerSettings getLogSettings()
   {
      if (target == RobotTarget.REAL_ROBOT)
      {
         return new DataServerSettings(true, "ValkyrieJSCGUI");
      }
      else
      {
         return new DataServerSettings(false, "SimulationGUI");
      }
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Valkyrie";
   }

   @Override
   public CollisionBoxProvider getCollisionBoxProvider()
   {
      return new ValkyrieCollisionBoxProvider(createFullRobotModel());
   }

   @Override
   public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
   {
      if (jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateLinkForModel(GeneralizedSDFRobotModel model, SDFLinkHolder linkHolder)
   {
      if (jointMap.getModelName().equals(model.getName()))
      {
         if (useOBJGraphics)
         {
            List<SDFVisual> visuals = linkHolder.getVisuals();
            if (visuals != null)
            {
               for (SDFVisual sdfVisual : visuals)
               {
                  SDFGeometry geometry = sdfVisual.getGeometry();
                  if (geometry == null)
                     continue;

                  SDFGeometry.Mesh mesh = geometry.getMesh();
                  if (mesh == null)
                     continue;

                  String meshUri = mesh.getUri();
                  if (meshUri.contains("meshes"))
                  {
                     String replacedURI = meshUri.replace(".dae", ".obj");
                     mesh.setUri(replacedURI);
                  }
               }
            }
         }

         switch (linkHolder.getName())
         {
            case "hokuyo_link":
               modifyHokuyoInertia(linkHolder);
               break;
            case "torso":
               modifyChestMass(linkHolder);
               break;
            default:
               break;
         }
      }
   }

   private void modifyChestMass(SDFLinkHolder chestSDFLink)
   {
      if (ValkyrieRosControlController.HAS_LIGHTER_BACKPACK)
         chestSDFLink.setMass(chestSDFLink.getMass() - 8.6);
   }

   @Override
   public void mutateSensorForModel(GeneralizedSDFRobotModel model, SDFSensor sensor)
   {
      if (jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateForceSensorForModel(GeneralizedSDFRobotModel model, SDFForceSensor forceSensor)
   {
      if (jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateContactSensorForModel(GeneralizedSDFRobotModel model, SDFContactSensor contactSensor)
   {
      if (jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateModelWithAdditions(GeneralizedSDFRobotModel model)
   {
      if (jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   private void modifyHokuyoInertia(SDFLinkHolder linkHolder)
   {
      linkHolder.getInertia().setM00(0.000401606); // i_xx
      linkHolder.getInertia().setM01(4.9927e-08); // i_xy
      linkHolder.getInertia().setM02(1.0997e-05); // i_xz
      linkHolder.getInertia().setM11(0.00208115); // i_yy
      linkHolder.getInertia().setM12(-9.8165e-09); // i_yz
      linkHolder.getInertia().setM22(0.00178402); // i_zz
   }

   @Override
   public SliderBoardParameters getSliderBoardParameters()
   {
      return new ValkyrieSliderBoardParameters();
   }

   @Override
   public FootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return new ValkyrieFootstepPlannerParameters();
   }

   @Override
   public VisibilityGraphsParametersBasics getVisibilityGraphsParameters()
   {
      return new DefaultVisibilityGraphParameters();
   }

   @Override
   public RobotContactPointParameters<RobotSide> getContactPointParameters()
   {
      if (contactPointParameters == null)
         contactPointParameters = new ValkyrieContactPointParameters(jointMap, simulationContactPoints);
      return contactPointParameters;
   }

   @Override
   public ValkyrieSensorInformation getSensorInformation()
   {
      if (sensorInformation == null)
         sensorInformation = new ValkyrieSensorInformation(target);
      return sensorInformation;
   }

   @Override
   public HighLevelControllerParameters getHighLevelControllerParameters()
   {
      if (highLevelControllerParameters == null)
         highLevelControllerParameters = new ValkyrieHighLevelControllerParameters(target, jointMap);
      return highLevelControllerParameters;
   }

   public ValkyrieCalibrationParameters getCalibrationParameters()
   {
      if (calibrationParameters == null)
         calibrationParameters = new ValkyrieCalibrationParameters(jointMap);
      return calibrationParameters;
   }

   /**
    * Adds robot specific footstep parameters
    */
   @Override
   public PlanarRegionFootstepPlanningParameters getPlanarRegionFootstepPlannerParameters()
   {
      if (planarRegionFootstepPlanningParameters == null)
         planarRegionFootstepPlanningParameters = new ValkyriePlanarRegionFootstepPlannerParameters();
      return planarRegionFootstepPlanningParameters;
   }

   @Override
   public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
   {
      if (capturePointPlannerParameters == null)
         capturePointPlannerParameters = new ValkyrieSmoothCMPPlannerParameters(target);
      return capturePointPlannerParameters;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      if (walkingControllerParameters == null)
         walkingControllerParameters = new ValkyrieWalkingControllerParameters(jointMap, target);
      return walkingControllerParameters;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      if (stateEstimatorParamaters == null)
         stateEstimatorParamaters = new ValkyrieStateEstimatorParameters(target, getEstimatorDT(), sensorInformation, jointMap);
      return stateEstimatorParamaters;
   }

   @Override
   public HumanoidRobotKinematicsCollisionModel getHumanoidRobotKinematicsCollisionModel()
   {
      return new ValkyrieKinematicsCollisionModel();
   }

   @Override
   public UIParameters getUIParameters()
   {
      return new ValkyrieUIParameters();
   }
}
