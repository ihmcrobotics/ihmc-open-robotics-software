package us.ihmc.acsell.parameters;

import java.io.InputStream;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.acsell.controlParameters.BonoArmControlParameters;
import us.ihmc.acsell.controlParameters.BonoStateEstimatorParameters;
import us.ihmc.acsell.controlParameters.BonoWalkingControllerParameters;
import us.ihmc.acsell.initialSetup.BonoInitialSetup;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactPointInformation;
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

public class BonoRobotModel implements DRCRobotModel
{

   private static Class<BonoRobotModel> thisClass = BonoRobotModel.class;
   private static String[] resourceDirectories;
   
   private final boolean runningOnRealRobot;
   
   public BonoRobotModel()
   {
      this(false);
   }
   
   public BonoRobotModel(boolean runningOnRealRobot)
   {
     this.runningOnRealRobot = runningOnRealRobot;
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
   public StateEstimatorParameters getStateEstimatorParameters(double estimatorDT)
   {
      return new BonoStateEstimatorParameters(runningOnRealRobot, estimatorDT);
   }

   @Override
   public DRCRobotPhysicalProperties getPhysicalProperties()
   {
      return new BonoPhysicalProperties();
   }

   @Override
   public DRCRobotJointMap getJointMap()
   {
      return new BonoJointMap(this);
   }

   @Override
   public boolean hasIRobotHands()
   {
      return false;
   }

   @Override
   public boolean hasArmExtensions()
   {
      return false;
   }

   @Override
   public boolean hasHookHands()
   {
      return false;
   }

   @Override
   public DRCHandType getHandType()
   {
      return DRCHandType.NONE;
   }

   @Override
   public String getModelName()
   {
      return "bono";
   }

   @Override
   public Transform getOffsetHandFromWrist(RobotSide side)
   {
      return new Transform();
   }

   @Override
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

   @Override
   public RobotType getType()
   {
      return RobotType.ACSELL;
   }

   public String toString()
   {
      return "BONO";
   }

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
   public DRCRobotContactPointParamaters getContactPointParamaters(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly)
   {
      return new BonoContactPointParamaters(getJointMap());
   }

   @Override
   public ContactPointInformation getContactPointInformation(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly)
   {
      return DRCContactPointInformationFactory.createContactPointInformation(getJointMap(),getContactPointParamaters(addLoadsOfContactPoints, addLoadsOfContactPointsToFeetOnly)) ;
   }
   
   @Override
   public DRCOutputWriter getOutputWriterWithAccelerationIntegration(DRCOutputWriter drcOutputWriter, double controlDT, boolean runningOnRealRobot)
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

}
