package us.ihmc.SdfLoader;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;

public class SDFExoskeleton extends SDFRobot {

	private final SideDependentList<String> jointsBeforeFeet = new SideDependentList<String>();

	private final SideDependentList<ArrayList<GroundContactPoint>> footGroundContactPoints = new SideDependentList<ArrayList<GroundContactPoint>>();

	public SDFExoskeleton(GeneralizedSDFRobotModel generalizedSDFRobotModel,
			SDFDescriptionMutator descriptionMutator, SDFJointNameMap sdfJointNameMap, boolean useCollisionMeshes) {
		this(generalizedSDFRobotModel, descriptionMutator, sdfJointNameMap, useCollisionMeshes, true, true);
	}

	public SDFExoskeleton(GeneralizedSDFRobotModel generalizedSDFRobotModel,
			SDFDescriptionMutator descriptionMutator, SDFJointNameMap sdfJointNameMap, boolean useCollisionMeshes,
			boolean enableTorqueVelocityLimits, boolean enableDamping) {
		super(generalizedSDFRobotModel, descriptionMutator, sdfJointNameMap, useCollisionMeshes,
				enableTorqueVelocityLimits, enableDamping);

		for (RobotSide robotSide : RobotSide.values) {
			footGroundContactPoints.put(robotSide, new ArrayList<GroundContactPoint>());
			if (sdfJointNameMap != null) {
				jointsBeforeFeet.put(robotSide, sdfJointNameMap.getJointBeforeFootName(robotSide));
			}
		}

		for (Joint joint : getOneDoFJoints()) {
			for (RobotSide robotSide : RobotSide.values) {
				ArrayList<GroundContactPoint> contactPointsForJoint = jointToGroundContactPointsMap.get(joint);

				if (contactPointsForJoint != null) {
					String jointName = joint.getName();
					if (jointName.equals(sdfJointNameMap.getJointBeforeFootName(robotSide))) {
						footGroundContactPoints.get(robotSide).addAll(contactPointsForJoint);
					}
				}
			}
		}
	}

	public List<GroundContactPoint> getFootGroundContactPoints(RobotSide robotSide) {
		return footGroundContactPoints.get(robotSide);
	}

	public SideDependentList<String> getJointNamesBeforeFeet() {
		return jointsBeforeFeet;
	}
}
