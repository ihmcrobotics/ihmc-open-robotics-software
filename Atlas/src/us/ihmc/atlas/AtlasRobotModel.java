package us.ihmc.atlas;

import java.net.URI;

import javax.media.j3d.Transform3D;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.atlas.parameters.AtlasArmControllerParameters;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasDrivingControllerParameters;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasRobotMultiContactControllerParameters;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.atlas.parameters.AtlasStateEstimatorParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.atlas.physics.AtlasPhysicsEngineConfiguration;
import us.ihmc.atlas.sensors.AtlasSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandType;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.AlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.AtlasPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.iRobot.model.iRobotHandModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.utilities.math.TimeTools;

import com.jme3.math.Transform;
import com.yobotics.simulationconstructionset.physics.ScsCollisionConfigure;

public class AtlasRobotModel implements DRCRobotModel
{
   private final AtlasRobotVersion selectedVersion;
   
   public static final long ESTIMATOR_DT_IN_NS = 1000000;
   public static final double ESTIMATOR_DT = TimeTools.nanoSecondstoSeconds(ESTIMATOR_DT_IN_NS);
   public static final double CONTROL_DT = 0.006;

   public static final double ATLAS_ONBOARD_SAMPLINGFREQ = 1000.0;
   public static final double ATLAS_ONBOARD_DT = 1.0 / ATLAS_ONBOARD_SAMPLINGFREQ;

   
   private final boolean runningOnRealRobot;
   private final JaxbSDFLoader loader;
  

   private DRCRobotSensorInformation sensorInformation;

   public AtlasRobotModel(AtlasRobotVersion atlasVersion, boolean runningOnRealRobot, boolean headless)
   {
      selectedVersion = atlasVersion;
      this.runningOnRealRobot = runningOnRealRobot;
      this.sensorInformation = new AtlasSensorInformation(runningOnRealRobot);
      
      if(!headless)
      {  
         this.loader = DRCRobotSDFLoader.loadDRCRobot(selectedVersion.getResourceDirectories(), selectedVersion.getSdfFileAsStream(), false);
      }
      else
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(new String[]{}, selectedVersion.getSdfFileAsStream(), headless);
      }
      
      SDFJointNameMap jointMap = getJointMap();
      for(String forceSensorNames : AtlasSensorInformation.forceSensorNames)
      {
         loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames, new Transform3D());
      }
   }

   @Override
   public ArmControllerParameters getArmControllerParameters()
   {
      return new AtlasArmControllerParameters(runningOnRealRobot);
   }

   @Override
   public WalkingControllerParameters getWalkingControlParameters()
   {
      return new AtlasWalkingControllerParameters(runningOnRealRobot);
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return new AtlasStateEstimatorParameters(runningOnRealRobot, getEstimatorDT());
   }

   @Override
   public DRCRobotPhysicalProperties getPhysicalProperties()
   {
      return new AtlasPhysicalProperties();
   }

   @Override
   public DRCRobotJointMap getJointMap()
   {
      return new AtlasJointMap(selectedVersion);
   }

   public boolean hasIRobotHands()
   {
      return selectedVersion.getHandModel() == DRCHandType.IROBOT;
   }

   public boolean hasArmExtensions()
   {
      return selectedVersion.hasArmExtensions();
   }

   public boolean hasHookHands()
   {
      return selectedVersion.getHandModel() == DRCHandType.HOOK;
   }

   public DRCHandType getHandType()
   {
      return selectedVersion.getHandModel();
   }

   public AtlasRobotVersion getAtlasVersion()
   {
      return selectedVersion;
   }

   @Override
   public Transform getOffsetHandFromWrist(RobotSide side)
   {
      return selectedVersion.getOffsetFromWrist(side);
   }

   @Override
   public String toString()
   {
      return selectedVersion.toString();
   }

   @Override
   public DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new AtlasSimInitialSetup(groundHeight, initialYaw);
   }

   @Override
   public WalkingControllerParameters getMultiContactControllerParameters()
   {
      return new AtlasRobotMultiContactControllerParameters();
   }

   @Override
   public ScsCollisionConfigure getPhysicsConfigure( SDFRobot sdfRobot )
   {
      return new AtlasPhysicsEngineConfiguration(getJointMap(),sdfRobot);
   }

   @Override
   public DRCRobotContactPointParameters getContactPointParamaters(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly)
   {
      return new AtlasContactPointParameters(selectedVersion,getJointMap(),addLoadsOfContactPoints,addLoadsOfContactPointsToFeetOnly);
   }

   @Override
   public DRCOutputWriter getOutputWriterWithAccelerationIntegration(DRCOutputWriter drcOutputWriter, boolean runningOnRealRobot)
   {
      throw new RuntimeException("Implement me!");
   }

   @Override
   public void setJointDamping(SDFRobot simulatedRobot)
   {
      AtlasDampingParameters.setDampingParameters(simulatedRobot, getHandType(), getJointMap());
   }

   @Override
   public HandModel getHandModel()
   {
      if(selectedVersion.getHandModel() == DRCHandType.IROBOT)
		   return new iRobotHandModel();
	   else
		   return null;
   }

   @Override
   public WalkingControllerParameters getDrivingControllerParameters()
   {
      return new AtlasDrivingControllerParameters();
   }

   @Override
   public DRCRobotSensorInformation getSensorInformation()
   {
      return sensorInformation;
   }
   
   @Override
   public SDFFullRobotModel createFullRobotModel()
   {
      return loader.createFullRobotModel(getJointMap());
   }

   @Override
   public SDFRobot createSdfRobot(boolean createCollisionMeshes)
   {
      return loader.createRobot(getJointMap(), createCollisionMeshes);
   }
   
   @Override
   public double getSimulateDT()
   {
      return 0.0001;
   }

   @Override
   public double getEstimatorDT()
   {
      return ESTIMATOR_DT;
   }

   @Override
   public double getControllerDT()
   {
      return CONTROL_DT;
   }
   

   @Override
   public GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
   }

   @Override
   public PPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
   {
      if(runningOnRealRobot)
      {
         return new AtlasPPSTimestampOffsetProvider();
      }
      return new AlwaysZeroOffsetPPSTimestampOffsetProvider();
   }

   @Override
   public boolean isRunningOnRealRobot()
   {
      return runningOnRealRobot;
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager(URI rosCoreURI)
   {
      return new AtlasSensorSuiteManager(rosCoreURI, getPPSTimestampOffsetProvider(), sensorInformation, getJointMap());
   }
}
