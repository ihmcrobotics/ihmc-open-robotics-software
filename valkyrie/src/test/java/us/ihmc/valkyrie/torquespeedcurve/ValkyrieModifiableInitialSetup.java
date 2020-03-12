package us.ihmc.valkyrie.torquespeedcurve;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.valkyrie.ValkyrieInitialSetup;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class ValkyrieModifiableInitialSetup extends ValkyrieInitialSetup {

	public ValkyrieModifiableInitialSetup(double groundZ, double initialYaw) {
		super(groundZ, initialYaw);
		// TODO Auto-generated constructor stub
	}

	protected void setActuatorPositions(FloatingRootJointRobot robot, DRCRobotJointMap jointMap)
	{
		super.setActuatorPositions(robot, jointMap);

		robot.update();
	}
	
	public void setJointPosition(FloatingRootJointRobot robot, String joint, double position)
	{
		OneDegreeOfFreedomJoint oneDofJoint= robot.getOneDegreeOfFreedomJoint(joint);
		if (oneDofJoint == null) {
			System.err.printf("Unable to set position on joint %s\n", joint);
			return;
		}
		System.out.printf("Set joint %s to position %f\n", joint, position);
		oneDofJoint.setQ(position);
		
		
		robot.update();
	}
	
	public ValkyrieModifiableInitialSetup enforceLimits(FloatingRootJointRobot robot, DRCRobotModel robotModel)
	{
		// Enforce positional limits
		String[] joints = robotModel.getJointMap().getOrderedJointNames();
		for (String joint: joints) {
			OneDegreeOfFreedomJoint oneDofJoint= robot.getOneDegreeOfFreedomJoint(joint);
			if (oneDofJoint == null) continue;
			if (oneDofJoint.getQ() < oneDofJoint.getJointLowerLimit() ) {
				System.out.printf("Enforced lower joint limit of %f for %s\n", oneDofJoint.getJointLowerLimit(), joint);
				oneDofJoint.setQ(oneDofJoint.getJointLowerLimit());
			} else if (oneDofJoint.getQ() > oneDofJoint.getJointUpperLimit()) {
				System.out.printf("Enforced upper joint limit of %f for %s\n", oneDofJoint.getJointLowerLimit(), joint);
				oneDofJoint.setQ(oneDofJoint.getJointUpperLimit());
			}
		}
		
		robot.update();
		return this;
	}

}
