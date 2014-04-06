package us.ihmc.darpaRoboticsChallenge.drcRobot;

import java.io.InputStream;

import com.jme3.math.Transform;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactPointInformation;
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

   public StateEstimatorParameters getStateEstimatorParameters(double estimatorDT);

   public DRCRobotPhysicalProperties getPhysicalProperties();

   public DRCRobotJointMap getJointMap();

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

   public DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw);

   public WalkingControllerParameters getMultiContactControllerParameters();

   public DRCRobotContactPointParamaters getContactPointParamaters(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly);

   public ContactPointInformation getContactPointInformation(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly);
   //TODO: RobotBoundingBoxes.java

}
