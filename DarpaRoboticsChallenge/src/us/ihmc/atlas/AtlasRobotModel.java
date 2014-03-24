package us.ihmc.atlas;

import java.io.InputStream;

import com.jme3.math.Transform;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.DRCLocalConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCSimDRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.StateEstimatorParameters;
import us.ihmc.robotSide.RobotSide;

public class AtlasRobotModel implements DRCRobotModel {


	private AtlasRobotVersion selectedVersion;
	
	public AtlasRobotModel()
	{
		this(AtlasRobotVersion.DRC_NO_HANDS);
	}
	
	public AtlasRobotModel(AtlasRobotVersion atlasVersion)
	{
		selectedVersion = atlasVersion;
	}
	
	public ArmControllerParameters getArmControllerParameters() {
		return new AtlasArmControllerParameters(DRCLocalConfigParameters.RUNNING_ON_REAL_ROBOT);
	}

	public WalkingControllerParameters getWalkingControlParamaters() {
		return new AtlasWalkingControllerParameters();
	}

	public StateEstimatorParameters getStateEstimatorParameters(
			boolean runningOnRealRobot) {
		return new AtlasStateEstimatorParameters(runningOnRealRobot);
	}

	public DRCRobotPhysicalProperties getPhysicalProperties() {
		return new AtlasPhysicalProperties();
	}

	public DRCRobotJointMap getJointMap(boolean addLoadsOfContactPoints,
			boolean addLoadsOfContactPointsToFeetOnly) {
		return new AtlasJointMap(this, addLoadsOfContactPoints, addLoadsOfContactPointsToFeetOnly);
	}

	public boolean hasIRobotHands() {
		return selectedVersion.getHandModel() == DRCHandModel.IROBOT;
	}

	public boolean hasArmExtensions() {
		return selectedVersion.hasArmExtensions();
	}

	public boolean hasHookHands() {
		return selectedVersion.getHandModel() == DRCHandModel.HOOK;
	}

	public DRCHandModel getHandModel() {
		return selectedVersion.getHandModel();
	}

	public String getModelName() {
		return "atlas";
	}
	
	public AtlasRobotVersion getAtlasVersion()
	{
		return selectedVersion;
	}

	@Override
	public String getSdfFile() {
		return selectedVersion.getSdfFile();
	}

	@Override
	public String[] getResourceDirectories() {
		return selectedVersion.getResourceDirectories();
	}

	@Override
	public InputStream getSdfFileAsStream() {
		return selectedVersion.getSdfFileAsStream();
	}

	@Override
	public Transform getOffsetHandFromWrist(RobotSide side) {
		return selectedVersion.getOffsetFromWrist(side);
	}

	@Override
	public RobotType getType() {
		return RobotType.ATLAS;
	}
	
	public String toString()
	{
		return selectedVersion.toString();
	}

	
	public DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new DRCSimDRCRobotInitialSetup(groundHeight, initialYaw);
   }
}
