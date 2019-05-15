package us.ihmc.valkyrie;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.StringUtils;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.drcRobot.shapeContactSettings.DRCRobotModelShapeCollisionSettings;
import us.ihmc.avatar.factory.SimulatedHandControlTask;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.time.DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SliderBoardParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.PlanarRegionFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
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
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFVisual;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFromDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;
import us.ihmc.valkyrie.configuration.YamlWithIncludesLoader;
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

   private final ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final StateEstimatorParameters stateEstimatorParamaters;
   private final ValkyrieSensorInformation sensorInformation;
   private final ValkyrieJointMap jointMap;
   private final ValkyrieContactPointParameters contactPointParameters;
   private final ValkyrieCalibrationParameters calibrationParameters;
   private final String robotName = "VALKYRIE";
   private final SideDependentList<Transform> offsetHandFromWrist = new SideDependentList<Transform>();
   @SuppressWarnings("unchecked")
   private final Map<String, Double> standPrepAngles = (Map<String, Double>) YamlWithIncludesLoader.load("standPrep", "setpoints.yaml");
   private final RobotTarget target;
   private final PlanarRegionFootstepPlanningParameters planarRegionFootstepPlanningParameters;
   private final HighLevelControllerParameters highLevelControllerParameters;
   private final ValkyrieCollisionMeshDefinitionDataHolder collisionMeshDefinitionDataHolder;

   private boolean useShapeCollision = false;
   private final boolean useOBJGraphics;
   private final double transparency;

   private final String[] resourceDirectories;
   {
      resourceDirectories = new String[] {"models/", "models/gazebo/", "models/val_description/", "models/val_description/sdf/",};
   }

   private final JaxbSDFLoader loader;
   private final RobotDescription robotDescription;

   public ValkyrieRobotModel(RobotTarget target, boolean headless, FootContactPoints<RobotSide> simulationContactPoints)
   {
      this(target, headless, "DEFAULT", simulationContactPoints, false);
   }

   public ValkyrieRobotModel(RobotTarget target, boolean headless)
   {
      this(target, headless, "DEFAULT", null, false);
   }

   public ValkyrieRobotModel(RobotTarget target, boolean headless, boolean useShapeCollision)
   {
      this(target, headless, "DEFAULT", null, useShapeCollision);
   }

   public ValkyrieRobotModel(RobotTarget target, boolean headless, String model)
   {
      this(target, headless, model, null, false);
   }

   public ValkyrieRobotModel(RobotTarget target, boolean headless, String model, FootContactPoints<RobotSide> simulationContactPoints)
   {
      this(target, headless, model, simulationContactPoints, false);
   }

   public ValkyrieRobotModel(RobotTarget target, boolean headless, String model, FootContactPoints<RobotSide> simulationContactPoints,
                             boolean useShapeCollision)
   {
      this(target, headless, model, simulationContactPoints, useShapeCollision, true);
   }

   public ValkyrieRobotModel(RobotTarget target, boolean headless, String model, FootContactPoints<RobotSide> simulationContactPoints, boolean useShapeCollision, boolean useOBJGraphics)
   {
      this(target, headless, model, simulationContactPoints, useShapeCollision, useOBJGraphics, Double.NaN);
   }

   public ValkyrieRobotModel(RobotTarget target, boolean headless, String model, FootContactPoints<RobotSide> simulationContactPoints, boolean useShapeCollision, boolean useOBJGraphics, double transparency)
   {
      this.target = target;
      this.useOBJGraphics = useOBJGraphics;
      jointMap = new ValkyrieJointMap();
      contactPointParameters = new ValkyrieContactPointParameters(jointMap, simulationContactPoints);
      sensorInformation = new ValkyrieSensorInformation(target);
      highLevelControllerParameters = new ValkyrieHighLevelControllerParameters(target, jointMap);
      calibrationParameters = new ValkyrieCalibrationParameters(jointMap);
      InputStream sdf = null;

      if (model.equalsIgnoreCase("DEFAULT"))
      {
         System.out.println("Loading robot model from: '" + getSdfFile() + "'");
         sdf = getSdfFileAsStream();
      }
      else
      {
         System.out.println("Loading robot model from: '" + model + "'");
         sdf = getClass().getClassLoader().getResourceAsStream(model);
         if (sdf == null)
         {
            try
            {
               sdf = new FileInputStream(model);
            }
            catch (FileNotFoundException e)
            {
               System.err.println("failed to load sdf file - file not found");
            }
         }

      }

      this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), sdf, this);
      this.transparency = transparency;

      for (String forceSensorNames : ValkyrieSensorInformation.forceSensorNames)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         if (forceSensorNames.equals("leftAnkleRoll"))
         {
            transform.set(ValkyrieSensorInformation.transformFromSixAxisMeasurementToAnkleZUpFrames.get(RobotSide.LEFT));
         }
         else if (forceSensorNames.equals("rightAnkleRoll"))
         {
            transform.set(ValkyrieSensorInformation.transformFromSixAxisMeasurementToAnkleZUpFrames.get(RobotSide.RIGHT));
         }

         loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames, transform);
      }

      for (RobotSide side : RobotSide.values())
      {
         for (String parentJointName : ValkyrieSensorInformation.contactSensors.get(side).keySet())
         {
            for (String sensorName : ValkyrieSensorInformation.contactSensors.get(side).get(parentJointName).keySet())
            {
               loader.addContactSensor(jointMap, sensorName, parentJointName,
                                       ValkyrieSensorInformation.contactSensors.get(side).get(parentJointName).get(sensorName));
            }
         }
      }

      planarRegionFootstepPlanningParameters = new ValkyriePlanarRegionFootstepPlannerParameters();
      capturePointPlannerParameters = new ValkyrieSmoothCMPPlannerParameters();
      walkingControllerParameters = new ValkyrieWalkingControllerParameters(jointMap, target);
      stateEstimatorParamaters = new ValkyrieStateEstimatorParameters(target, getEstimatorDT(), sensorInformation, jointMap);
      collisionMeshDefinitionDataHolder = new ValkyrieCollisionMeshDefinitionDataHolder(jointMap);

      this.useShapeCollision = useShapeCollision;
      robotDescription = createRobotDescription();
   }

   private RobotDescription createRobotDescription()
   {
      boolean useCollisionMeshes = false;

      GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
      RobotDescriptionFromSDFLoader descriptionLoader = new RobotDescriptionFromSDFLoader();
      RobotDescription robotDescription;

      if (useShapeCollision)
      {
         robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, jointMap, null, false, transparency);
         collisionMeshDefinitionDataHolder.setVisible(false);

         robotDescription.addCollisionMeshDefinitionData(collisionMeshDefinitionDataHolder);
      }
      else
      {
         robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, jointMap, getContactPointParameters(), useCollisionMeshes, transparency);
      }

      return robotDescription;
   }

   @Override
   public DRCRobotModelShapeCollisionSettings getShapeCollisionSettings()
   {
      return new ValkyrieRobotModelShapeCollisionSettings(useShapeCollision);
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }

   @Override
   public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
   {
      return capturePointPlannerParameters;
   }

   @Override
   public UIParameters getUIParameters()
   {
      return new ValkyrieUIParameters();
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return stateEstimatorParamaters;
   }

   @Override
   public ValkyrieJointMap getJointMap()
   {
      return jointMap;
   }

   @Override
   public double getStandPrepAngle(String jointName)
   {
      return standPrepAngles.get(jointName);
   }

   @Override
   public Transform getJmeTransformWristToHand(RobotSide side)
   {
      //      if (offsetHandFromWrist.get(side) == null)
      //      {
      createTransforms();
      //      }

      return offsetHandFromWrist.get(side);
   }

   private void createTransforms()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         Vector3f centerOfHandToWristTranslation = new Vector3f();
         float[] angles = new float[3];

         centerOfHandToWristTranslation = new Vector3f(0f, robotSide.negateIfLeftSide(0.015f), -0.06f);
         angles[0] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));
         angles[1] = 0.0f;
         angles[2] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));
         //
         Quaternion centerOfHandToWristRotation = new Quaternion(angles);
         offsetHandFromWrist.set(robotSide, new Transform(centerOfHandToWristTranslation, centerOfHandToWristRotation));
      }
   }

   private String getSdfFile()
   {
      if (this.target == RobotTarget.REAL_ROBOT)
         return ValkyrieConfigurationRoot.REAL_ROBOT_SDF_FILE;
      else
         return ValkyrieConfigurationRoot.SIM_SDF_FILE;
   }

   private String[] getResourceDirectories()
   {
      return resourceDirectories;
   }

   private InputStream getSdfFileAsStream()
   {
      return getClass().getClassLoader().getResourceAsStream(getSdfFile());
   }

   @Override
   public String toString()
   {
      return robotName;
   }

   @Override
   public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new ValkyrieInitialSetup(groundHeight, initialYaw);
   }

   @Override
   public RobotContactPointParameters<RobotSide> getContactPointParameters()
   {
      return contactPointParameters;
   }

   @Override
   public ValkyrieHandModel getHandModel()
   {
      return new ValkyrieHandModel();
   }

   @Override
   public ValkyrieSensorInformation getSensorInformation()
   {
      return sensorInformation;
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
      return loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
   }

   @Override
   public DRCROSPPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
   {
      return new DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider();
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager()
   {
      return new ValkyrieSensorSuiteManager(getSimpleRobotName(), this, getCollisionBoxProvider(), getPPSTimestampOffsetProvider(), sensorInformation, jointMap,
                                            target);
   }

   @Override
   public SimulatedHandControlTask createSimulatedHandController(FloatingRootJointRobot simulatedRobot, RealtimeRos2Node realtimeRos2Node)
   {
      if (!ValkyrieConfigurationRoot.VALKYRIE_WITH_ARMS)
         return null;
      else
         return new SimulatedValkyrieFingerController(simulatedRobot, realtimeRos2Node, this,
                                                      ControllerAPIDefinition.getPublisherTopicNameGenerator(getSimpleRobotName()),
                                                      ControllerAPIDefinition.getSubscriberTopicNameGenerator(getSimpleRobotName()));
   }

   @Override
   public QuadTreeFootstepPlanningParameters getQuadTreeFootstepPlanningParameters()
   {
      return new ValkyrieFootstepPlanningParameters();
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new SDFLogModelProvider(jointMap.getModelName(), getSdfFileAsStream(), getResourceDirectories());
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

   public String getFullRobotName()
   {
      String fullRobotName = getSdfFile();
      fullRobotName = fullRobotName.substring(fullRobotName.lastIndexOf("/") + 1, fullRobotName.length());
      fullRobotName = StringUtils.capitalize(fullRobotName);
      fullRobotName = StringUtils.remove(fullRobotName, "_hw");
      fullRobotName = StringUtils.remove(fullRobotName, "_sim");
      fullRobotName = StringUtils.remove(fullRobotName, ".sdf");

      return fullRobotName;
   }

   @Override
   public CollisionBoxProvider getCollisionBoxProvider()
   {
      return new ValkyrieCollisionBoxProvider(createFullRobotModel());
   }

   @Override
   public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
   {
      if (this.jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateLinkForModel(GeneralizedSDFRobotModel model, SDFLinkHolder linkHolder)
   {
      if (this.jointMap.getModelName().equals(model.getName()))
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
      if (this.jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateForceSensorForModel(GeneralizedSDFRobotModel model, SDFForceSensor forceSensor)
   {
      if (this.jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateContactSensorForModel(GeneralizedSDFRobotModel model, SDFContactSensor contactSensor)
   {
      if (this.jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateModelWithAdditions(GeneralizedSDFRobotModel model)
   {
      if (this.jointMap.getModelName().equals(model.getName()))
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

   /**
    * Adds robot specific footstep parameters
    */
   @Override
   public PlanarRegionFootstepPlanningParameters getPlanarRegionFootstepPlannerParameters()
   {
      return planarRegionFootstepPlanningParameters;
   }

   @Override
   public FootstepPlannerParameters getFootstepPlannerParameters()
   {
      return new ValkyrieFootstepPlannerParameters();
   }

   @Override
   public VisibilityGraphsParameters getVisibilityGraphsParameters()
   {
      return new DefaultVisibilityGraphParameters();
   }

   @Override
   public HighLevelControllerParameters getHighLevelControllerParameters()
   {
      return highLevelControllerParameters;
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

   public ValkyrieCalibrationParameters getCalibrationParameters()
   {
      return calibrationParameters;
   }
}
