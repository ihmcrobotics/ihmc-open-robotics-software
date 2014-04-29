package us.ihmc.acsell.parameters;

import java.io.InputStream;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.acsell.controlParameters.BonoArmControlParameters;
import us.ihmc.acsell.controlParameters.BonoStateEstimatorParameters;
import us.ihmc.acsell.controlParameters.BonoWalkingControllerParameters;
import us.ihmc.acsell.initialSetup.BonoInitialSetup;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactPointInformation;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCContactPointInformationFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandType;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
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
   
   public BonoRobotModel()
   {
      this(false);
   }
   
   public BonoRobotModel(boolean runningOnRealRobot)
   {
     this.runningOnRealRobot = runningOnRealRobot;
   }

   public ArmControllerParameters getArmControllerParameters()
   {
      return new BonoArmControlParameters(runningOnRealRobot);
   }

   public WalkingControllerParameters getWalkingControlParameters()
   {
      return new BonoWalkingControllerParameters(runningOnRealRobot);
   }

   public StateEstimatorParameters getStateEstimatorParameters(double estimatorDT)
   {
      return new BonoStateEstimatorParameters(runningOnRealRobot, estimatorDT);
   }

   public DRCRobotPhysicalProperties getPhysicalProperties()
   {
      return new BonoPhysicalProperties();
   }

   public DRCRobotJointMap getJointMap()
   {
      return new BonoJointMap();
   }

   public boolean hasIRobotHands()
   {
      return false;
   }

   public boolean hasArmExtensions()
   {
      return false;
   }

   public boolean hasHookHands()
   {
      return false;
   }

   public DRCHandType getHandType()
   {
      return DRCHandType.NONE;
   }

   public String getModelName()
   {
      return "bono";
   }

   public Transform getOffsetHandFromWrist(RobotSide side)
   {
      return new Transform();
   }

   public String getSdfFile()
   {
      return "../models/axl/axl_description/bono/robots/bono.sdf";
   }

   public String[] getResourceDirectories()
   {
      if (resourceDirectories == null)
      {
         resourceDirectories = new String[] { thisClass.getResource("../models/axl/axl_description").getFile() };
      }
      return resourceDirectories;
   }

   public InputStream getSdfFileAsStream()
   {
      return thisClass.getResourceAsStream(getSdfFile());
   }

   public String toString()
   {
      return "BONO";
   }

   public DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new BonoInitialSetup(groundHeight, initialYaw);
   }

   public WalkingControllerParameters getMultiContactControllerParameters()
   {
      return new BonoWalkingControllerParameters();
   }
   //XXX: fix this
   public DRCRobotContactPointParamaters getContactPointParamaters(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly)
   {
      return new BonoContactPointParamaters(getJointMap());
   }

   public ContactPointInformation getContactPointInformation(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly)
   {
      return DRCContactPointInformationFactory.createContactPointInformation(getJointMap(),getContactPointParamaters(addLoadsOfContactPoints, addLoadsOfContactPointsToFeetOnly)) ;
   }
   
   public DRCOutputWriter getOutputWriterWithAccelerationIntegration(DRCOutputWriter drcOutputWriter, double controlDT, boolean runningOnRealRobot)
   {
      throw new RuntimeException("Implement me!");
   }

   public void setJointDamping(SDFRobot simulatedRobot)
   {
      System.err.println("Joint Damping not setup for Bono. BonoRobotModel setJointDamping!");
   }

   public HandModel getHandModel()
   {
	   return null;
   }

   public ScsCollisionConfigure getPhysicsConfigure(SDFRobot robotModel)
   {
      return null;
   }

   public WalkingControllerParameters getDrivingControllerParameters()
   {
      return null;
   }

   public JaxbSDFLoader getJaxbSDFLoader(boolean headless)
   {
      if(loader == null)
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), getSdfFileAsStream(), headless);
      }
      return loader;
   }
}
