package us.ihmc.acsell.parameters;

import java.io.InputStream;
import java.net.URI;

import javax.media.j3d.Transform3D;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.acsell.controlParameters.BonoArmControlParameters;
import us.ihmc.acsell.controlParameters.BonoStateEstimatorParameters;
import us.ihmc.acsell.controlParameters.BonoWalkingControllerParameters;
import us.ihmc.acsell.initialSetup.BonoInitialSetup;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.AlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

import com.jme3.math.Transform;
import com.yobotics.simulationconstructionset.physics.ScsCollisionConfigure;

public class BonoRobotModel implements DRCRobotModel
{

   private static Class<BonoRobotModel> thisClass = BonoRobotModel.class;
   private static String[] resourceDirectories;
   
   private final boolean runningOnRealRobot;
   private JaxbSDFLoader loader;
   private DRCRobotSensorInformation sensorInformation;
   
   
   public BonoRobotModel(boolean runningOnRealRobot, boolean headless)
   {
     this.runningOnRealRobot = runningOnRealRobot;
     sensorInformation = new BonoSensorInformation();
     if(headless)
     {
        this.loader = DRCRobotSDFLoader.loadDRCRobot(new String[]{}, getSdfFileAsStream(), true);
     }
     else
     {
        this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), getSdfFileAsStream(), false);        
     }
     
     SDFJointNameMap jointMap = getJointMap();
     for(String forceSensorNames : getSensorInformation().getForceSensorNames())
     {
        loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames, new Transform3D());
     }
     
   }

   @Override
   public ArmControllerParameters getArmControllerParameters()
   {
      return new BonoArmControlParameters(runningOnRealRobot);
   }

   @Override
   public WalkingControllerParameters getWalkingControlParameters()
   {
      return new BonoWalkingControllerParameters(runningOnRealRobot);
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return new BonoStateEstimatorParameters(runningOnRealRobot, getEstimatorDT());
   }

   @Override
   public DRCRobotPhysicalProperties getPhysicalProperties()
   {
      return new BonoPhysicalProperties();
   }

   @Override
   public DRCRobotJointMap getJointMap()
   {
      return new BonoJointMap();
   }

   @Override
   public Transform getOffsetHandFromWrist(RobotSide side)
   {
      return new Transform();
   }

   private String getSdfFile()
   {
      return "../models/axl/axl_description/bono/robots/bono.sdf";
   }

   private String[] getResourceDirectories()
   {
      if (resourceDirectories == null)
      {
         resourceDirectories = new String[] { thisClass.getResource("../models/axl/axl_description").getFile() };
      }
      return resourceDirectories;
   }

   private InputStream getSdfFileAsStream()
   {
      return thisClass.getResourceAsStream(getSdfFile());
   }

   @Override
   public String toString()
   {
      return "BONO";
   }

   @Override
   public DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new BonoInitialSetup(groundHeight, initialYaw);
   }

   @Override
   public WalkingControllerParameters getMultiContactControllerParameters()
   {
      return new BonoWalkingControllerParameters();
   }
   
   //XXX: fix this
   @Override
   public DRCRobotContactPointParameters getContactPointParamaters(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly)
   {
      return new BonoContactPointParameters(getJointMap());
   }
   
   @Override
   public DRCOutputWriter getOutputWriterWithAccelerationIntegration(DRCOutputWriter drcOutputWriter, boolean runningOnRealRobot)
   {
      throw new RuntimeException("Implement me!");
   }

   @Override
   public void setJointDamping(SDFRobot simulatedRobot)
   {
      System.err.println("Joint Damping not setup for Bono. BonoRobotModel setJointDamping!");
   }

   @Override
   public HandModel getHandModel()
   {
	   return null;
   }

   @Override
   public ScsCollisionConfigure getPhysicsConfigure(SDFRobot robotModel)
   {
      return null;
   }

   @Override
   public WalkingControllerParameters getDrivingControllerParameters()
   {
      return null;
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
      return 0.001;
   }

   @Override
   public double getControllerDT()
   {
      return 0.006;
   }

   @Override
   public GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
   }
   
   @Override
   public PPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
   {
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
      return null;
   }
}
