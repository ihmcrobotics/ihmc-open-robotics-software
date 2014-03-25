package us.ihmc.darpaRoboticsChallenge;

import java.io.InputStream;

import com.jme3.math.Transform;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

public interface DRCRobotModel
{
   public enum RobotType
   {
      ATLAS, VALKYRIE, ACSELL
   }
   
   public ArmControllerParameters getArmControllerParameters();
   public WalkingControllerParameters getWalkingControlParamaters();
   public StateEstimatorParameters getStateEstimatorParameters(boolean runningOnRealRobot, double estimatorDT);
   public DRCRobotPhysicalProperties getPhysicalProperties();
   public DRCRobotJointMap getJointMap(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly);
   public boolean hasIRobotHands();
   public boolean hasArmExtensions();
   public boolean hasHookHands();
   public Transform getOffsetHandFromWrist(RobotSide side);
   public DRCHandModel getHandModel();
   public RobotType getType();
   public String getModelName();
   public String getSdfFile();
   public InputStream getSdfFileAsStream();
   public String[] getResourceDirectories();
   public DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(
		double groundHeight, double initialYaw);
   public WalkingControllerParameters getMultiContactControllerParameters();
   
   //TODO: RobotBoundingBoxes.java
   
}
