package us.ihmc.darpaRoboticsChallenge.drcRobot;

import java.io.InputStream;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactPointInformation;
import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandType;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

import com.jme3.math.Transform;

public interface DRCRobotModel
{
   public ArmControllerParameters getArmControllerParameters();

   public WalkingControllerParameters getWalkingControlParameters();

   public StateEstimatorParameters getStateEstimatorParameters(double estimatorDT);

   public DRCRobotPhysicalProperties getPhysicalProperties();

   public DRCRobotJointMap getJointMap();

   public boolean hasIRobotHands();

   public boolean hasArmExtensions();

   public boolean hasHookHands();

   public Transform getOffsetHandFromWrist(RobotSide side);

   public DRCHandType getHandType();

   public String getModelName();

   public String getSdfFile();

   public InputStream getSdfFileAsStream();

   public String[] getResourceDirectories();

   public DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw);

   public WalkingControllerParameters getMultiContactControllerParameters();

   public DRCRobotContactPointParamaters getContactPointParamaters(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly);

   public ContactPointInformation getContactPointInformation(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly);
   //TODO: RobotBoundingBoxes.java
   
   public DRCOutputWriter getOutputWriterWithAccelerationIntegration(DRCOutputWriter drcOutputWriter, double controlDT, boolean runningOnRealRobot);
   
   public void setJointDamping(SDFRobot simulatedRobot);
   
   public HandModel getHandModel();
}
