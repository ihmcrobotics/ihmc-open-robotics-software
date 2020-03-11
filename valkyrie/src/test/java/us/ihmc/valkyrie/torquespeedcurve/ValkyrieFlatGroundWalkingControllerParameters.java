package us.ihmc.valkyrie.torquespeedcurve;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;

public class ValkyrieFlatGroundWalkingControllerParameters extends ValkyrieWalkingControllerParameters {
	ValkyrieWalkingParameterValues walkingValues;

	public ValkyrieFlatGroundWalkingControllerParameters(ValkyrieJointMap jointMap,
			ValkyriePhysicalProperties physicalProperties, RobotTarget target, ValkyrieWalkingParameterValues values) {
		super(jointMap, physicalProperties, target);
		walkingValues = values;
	}

	//	@Override
	//	public double nominalHeightAboveAnkle() {
	//		// TODO Auto-generated method stub
	//		return 0;
	//	}
	//
	//	@Override
	//	public double minimumHeightAboveAnkle() {
	//		// TODO Auto-generated method stub
	//		return 0;
	//	}
	//
	//	@Override
	//	public double maximumHeightAboveAnkle() {
	//		// TODO Auto-generated method stub
	//		return 0;
	//	}
	//
	//	@Override
	//	public ToeOffParameters getToeOffParameters() {
	//		// TODO Auto-generated method stub
	//		return null;
	//	}
	//
	//	@Override
	//	public PIDSE3Configuration getToeOffFootControlGains() {
	//		// TODO Auto-generated method stub
	//		return null;
	//	}
	//
	//	@Override
	//	public SwingTrajectoryParameters getSwingTrajectoryParameters() {
	//		// TODO Auto-generated method stub
	//		return null;
	//	}
	//
	//	@Override
	//	public PIDSE3Configuration getSwingFootControlGains() {
	//		// TODO Auto-generated method stub
	//		return null;
	//	}
	//
	//	@Override
	//	public SteppingParameters getSteppingParameters() {
	//		// TODO Auto-generated method stub
	//		return null;
	//	}
	//
	//	@Override
	//	public double getOmega0() {
	//		// TODO Auto-generated method stub
	//		return 0;
	//	}
	//
	//	@Override
	//	public MomentumOptimizationSettings getMomentumOptimizationSettings() {
	//		// TODO Auto-generated method stub
	//		return null;
	//	}
	//
	//	@Override
	//	public double getMinimumSwingTimeForDisturbanceRecovery() {
	//		// TODO Auto-generated method stub
	//		return 0;
	//	}
	//
	//	@Override
	//	public double getMaximumLegLengthForSingularityAvoidance() {
	//		// TODO Auto-generated method stub
	//		return 0;
	//	}
	//
	//	@Override
	//	public double getMaxICPErrorBeforeSingleSupportInnerY() {
	//		// TODO Auto-generated method stub
	//		return 0;
	//	}
	//
	//	@Override
	//	public double getMaxICPErrorBeforeSingleSupportForwardX() {
	//		// TODO Auto-generated method stub
	//		return 0;
	//	}
	//
	//	@Override
	//	public String[] getJointsToIgnoreInController() {
	//		// TODO Auto-generated method stub
	//		return null;
	//	}
	//
	//	@Override
	//	public ICPOptimizationParameters getICPOptimizationParameters() {
	//		// TODO Auto-generated method stub
	//		return null;
	//	}
	//
	//	@Override
	//	public double getICPErrorThresholdToSpeedUpSwing() {
	//		// TODO Auto-generated method stub
	//		return 0;
	//	}
	//
	//	@Override
	//	public ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters() {
	//		// TODO Auto-generated method stub
	//		return null;
	//	}
	//
	//	@Override
	//	public PIDSE3Configuration getHoldPositionFootControlGains() {
	//		// TODO Auto-generated method stub
	//		return null;
	//	}
	//
	//	@Override
	//	public FootSwitchFactory getFootSwitchFactory() {
	//		// TODO Auto-generated method stub
	//		return null;
	//	}
	//
	//	@Override
	//	public PDGains getCoMHeightControlGains() {
	//		// TODO Auto-generated method stub
	//		return null;
	//	}
	//
	//	@Override
	//	public double defaultOffsetHeightAboveAnkle() {
	//		// TODO Auto-generated method stub
	//		return 0;
	//	}
	//
	//	@Override
	//	public ICPControlGains createICPControlGains() {
	//		// TODO Auto-generated method stub
	//		return null;
	//	}
	//
	//	@Override
	//	public boolean allowDisturbanceRecoveryBySpeedingUpSwing() {
	//		// TODO Auto-generated method stub
	//		return false;

	@Override
	public double getDefaultTransferTime()
	{
		return walkingValues.defaultTransferTime;
	}

	@Override
	public double getDefaultSwingTime()
	{
		return walkingValues.defaultSwingTime;
	}

	/** @inheritDoc */
	@Override
	public double getDefaultInitialTransferTime()
	{
		return walkingValues.defaultInitialTransferTime;
	}

}
