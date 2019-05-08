package us.ihmc.exampleSimulations.stickRobot;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.time.DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
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
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFromDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

/**
 * @author Shlok Agarwal on 7/5/17
 *
 */
public class StickRobotModel implements DRCRobotModel, SDFDescriptionMutator
{

   private static final boolean PRINT_MODEL = false;

   private final ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final StateEstimatorParameters stateEstimatorParamaters;
   private final HighLevelControllerParameters highLevelControllerParameters;
   private final StickRobotJointMap jointMap;
   private final StickRobotContactPointParameters contactPointParameters;
   private final String robotName = "STICK_BOT";
   private final SideDependentList<Transform> offsetHandFromWrist = new SideDependentList<Transform>();

   private final RobotTarget target;

   private final String[] resourceDirectories;
   {
      resourceDirectories = new String[] {"models/stickRobot/sdf"};
   }

   private final JaxbSDFLoader loader;
   private final RobotDescription robotDescription;

   // One of the three constructors below calls the fourth constructor with different configuration settings
   public StickRobotModel(RobotTarget target, boolean headless, FootContactPoints simulationContactPoints)
   {
      this(target, headless, "DEFAULT", simulationContactPoints);
   }

   public StickRobotModel(RobotTarget target, boolean headless)
   {
      this(target, headless, "DEFAULT", null);
   }

   public StickRobotModel(RobotTarget target, boolean headless, String model)
   {
      this(target, headless, model, null);
   }

   // main constructor being called
   public StickRobotModel(RobotTarget target, boolean headless, String model, FootContactPoints simulationContactPoints)
   {
      this.target = target;
      jointMap = new StickRobotJointMap();

      // in our case, simulationConactPoints are null currently. will call the createDefaultFootContactPoints according to SCS
      contactPointParameters = new StickRobotContactPointParameters(jointMap, simulationContactPoints);

      // physical robot dimensions required for calculations
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

      // loads the SDF directory located in the resources folder
      this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), sdf, this);

      //TODO currently set to null: change for walking
      capturePointPlannerParameters = null;
      walkingControllerParameters = null;
      stateEstimatorParamaters = null;
      highLevelControllerParameters = null;

      //
      robotDescription = createRobotDescription();
   }

   private RobotDescription createRobotDescription()
   {
      // this function would create the robot description from the SDF file.
      // it uses the joint map, SDF model and contact parameters as arguments

      boolean useCollisionMeshes = false;

      GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
      RobotDescriptionFromSDFLoader descriptionLoader = new RobotDescriptionFromSDFLoader();
      RobotDescription robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, jointMap, contactPointParameters,
                                                                                        useCollisionMeshes);
      return robotDescription;
   }

   public GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
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

   private String getSdfFile()
   {
      return "models/stickRobot/sdf/stickRobot.sdf";
   }

   private String[] getResourceDirectories()
   {
      return resourceDirectories;
   }

   @Override
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
   {
      boolean enableTorqueVelocityLimits = false;

      HumanoidFloatingRootJointRobot sdfRobot = new HumanoidFloatingRootJointRobot(robotDescription, jointMap, enableJointDamping, enableTorqueVelocityLimits);

      if (PRINT_MODEL)
      {
         System.out.println("\nStickRobotModel Link Masses:");

         StringBuffer stringBuffer = new StringBuffer();
         sdfRobot.printRobotJointsAndMasses(stringBuffer);
         System.out.println(stringBuffer);
         System.out.println();

         System.out.println("nStickRobotModel: \n" + sdfRobot);
      }

      return sdfRobot;
   }

   @Override
   public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
   {
      return capturePointPlannerParameters;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public RobotContactPointParameters getContactPointParameters()
   {
      return contactPointParameters;
   }

   @Override
   public double getControllerDT()
   {
      return 0.004;
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel()
   {
      return new FullHumanoidRobotModelFromDescription(robotDescription, jointMap);
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return stateEstimatorParamaters;
   }

   @Override
   public HighLevelControllerParameters getHighLevelControllerParameters()
   {
      return highLevelControllerParameters;
   }

   @Override
   public DRCRobotJointMap getJointMap()
   {
      return jointMap;
   }

   @Override
   public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return null;
   }

   @Override
   public Transform getJmeTransformWristToHand(RobotSide side)
   {
      createTransforms();
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

   @Override
   public double getSimulateDT()
   {
      return 0.0001; //0.00003875;
   }

   @Override
   public double getEstimatorDT()
   {
      return 0.002;
   }

   @Override
   public DRCROSPPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
   {
      return new DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider();
   }

   @Override
   public LogSettings getLogSettings()
   {
      return LogSettings.SIMULATION;
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new SDFLogModelProvider(jointMap.getModelName(), getSdfFileAsStream(), getResourceDirectories());
   }

   @Override
   public String getSimpleRobotName()
   {
      return "StickRobot";
   }

   @Override
   public double getStandPrepAngle(String jointName)
   {
      return 0;
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager()
   {
      return null;
   }

   @Override
   public HandModel getHandModel()
   {
      return null;
   }

   @Override
   public CollisionBoxProvider getCollisionBoxProvider()
   {
      return null;
   }

   @Override
   public DRCRobotSensorInformation getSensorInformation()
   {
      return null;
   }

   @Override
   public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
   {

   }

   @Override
   public void mutateLinkForModel(GeneralizedSDFRobotModel model, SDFLinkHolder linkHolder)
   {

   }

   @Override
   public void mutateSensorForModel(GeneralizedSDFRobotModel model, SDFSensor sensor)
   {

   }

   @Override
   public void mutateForceSensorForModel(GeneralizedSDFRobotModel model, SDFForceSensor forceSensor)
   {

   }

   @Override
   public void mutateContactSensorForModel(GeneralizedSDFRobotModel model, SDFContactSensor contactSensor)
   {

   }

   @Override
   public void mutateModelWithAdditions(GeneralizedSDFRobotModel model)
   {

   }

   @Override
   public InputStream getWholeBodyControllerParametersFile()
   {
      return null;
   }
}
