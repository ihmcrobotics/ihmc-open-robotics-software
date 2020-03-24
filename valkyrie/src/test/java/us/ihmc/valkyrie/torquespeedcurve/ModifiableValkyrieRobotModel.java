package us.ihmc.valkyrie.torquespeedcurve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutatorList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieSDFDescriptionMutator;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;

/* ValkyrieRobotModel with SDF modifications */
public class ModifiableValkyrieRobotModel extends ValkyrieRobotModel {
	public ModifiableValkyrieRobotModel(RobotTarget target, ModifiableValkyrieRobotConfig config) {
		super(target);
		
		// Scale robot mass and size
		setModelMassScale(config.getGlobalMassScale());
		setModelSizeScale(config.getGlobalSizeScale());
		
		
		// Apply SDF modifications
		SDFDescriptionMutatorList mutators = getSdfMutators(config);
		
		// Apply mutators to the robot. There is always at least one mutator.
	    setSDFDescriptionMutator(mutators);
	}
	
	
	private SDFDescriptionMutatorList getSdfMutators(ModifiableValkyrieRobotConfig config) {

		// Create a list of joint names with the format we need
		ValkyrieJointMap jointMap = getJointMap();
		HashSet<String> validJointNames = new HashSet<>(Arrays.asList(jointMap.getOrderedJointNames()));
		HashSet<String> validLinkNames = getLinkNames(validJointNames);
		
		// Create a list of mutators based on the torque limits defined in the params file.
		// The first mutator is of a different type, and exists to fix up various things in the Valkyrie model.
		SDFDescriptionMutatorList mutator_list = new SDFDescriptionMutatorList(
				new ValkyrieSDFDescriptionMutator(jointMap, false), new ValkyrieJointTorqueLimitMutator(jointMap));

		// Add a mutator for each joint torque limit specified in the parameter file
		for (String joint : config.getTorqueLimits().keySet()) {
			HashSet<String> jointNames = getCanonicalNames(joint, validJointNames, "torque limit joint name");
			for (String fullJointName: jointNames) {
				mutator_list.addMutator(new ValkyrieJointTorqueLimitMutator(jointMap, fullJointName,
						config.getTorqueLimits().get(joint)));
			}
		}		
		
		// Add a mutator for each joint velocity limit specified in the parameter file
		for (String joint : config.getVelocityLimits().keySet()) {
			HashSet<String> jointNames = getCanonicalNames(joint, validJointNames, "velocity limit joint name");
			for (String fullJointName: jointNames) {
				mutator_list.addMutator(new ValkyrieJointVelocityLimitMutator(jointMap, fullJointName,
						config.getVelocityLimits().get(joint)));
			}
		}
		
		// Add a mutator for each joint mass modifier specified in the parameter file
		for (String link : config.getLinkMassKg().keySet()) {
			HashSet<String> linkNames = getCanonicalNames(link, validLinkNames, "mass link name");
			for (String fullLinkName: linkNames) {
				double desiredMass = config.getLinkMassKg().get(link);
				mutator_list.addMutator(new ValkyrieLinkMassMutator(jointMap, fullLinkName, desiredMass));
			}
		}
		
		// Add a mutator for each joint position limit modifier in the parameter file
		for (String joint : config.getPositionLimits().keySet()) {
			HashSet<String> jointNames = getCanonicalNames(joint, validJointNames, "position limit joint name");
			for (String fullJointName: jointNames) {
				mutator_list.addMutator(new ValkyrieJointPositionLimitMutator(jointMap, fullJointName,
						config.getPositionLimits().get(joint)));
			}
		}
		
		// Disable ankle limits on the robot
		if (config.getAnkleLimitsDisabled()) {
			ArrayList<Double> limits = new ArrayList<Double>();
			limits.add(-180.0);
			limits.add(180.0);
			mutator_list.addMutator(new ValkyrieJointPositionLimitMutator(getJointMap(), "leftAnklePitch", limits));
			mutator_list.addMutator(new ValkyrieJointPositionLimitMutator(getJointMap(), "rightAnklePitch", limits));
		}
		
		return mutator_list;
	}
	
	private static String capitalize(String s) {
		if (s == null || s.length() == 0) {
			return s;
		}
		return s.substring(0, 1).toUpperCase() + s.substring(1);
	}

	private static String uncapitalize(String s) {
		if (s == null || s.length() == 0) {
			return s;
		}
		return s.substring(0, 1).toLowerCase() + s.substring(1);
	}	

	/**
	 * Leg and arm joints/links should be parallel on each side, so we allow user to specify, for example, "HipPitch"
	 * and we translate that into a pair of names for left and right side.
	 * @param name -- incoming name to resolve
	 * @param fullyQualifiedNames -- set of valid names to match
	 * @param description -- description of what sort of object we're working with for error output
	 * @return list of fully qualified names based on the input name
	 */
	// and we translate that into a pair of SDF mutations for leftHipPitch and
	// rightHipPitch
	private static HashSet<String> getCanonicalNames(String name, HashSet<String> fullyQualifiedNames, String description)
	{
		HashSet<String> names = new HashSet<String>();
		
		if (!fullyQualifiedNames.contains(uncapitalize(name))) {

			for (RobotSide robotSide : RobotSide.values) {
				String fullJointName = robotSide.getCamelCaseNameForStartOfExpression() + capitalize(name);
				if (!fullyQualifiedNames.contains(fullJointName)) {
					System.err.printf("Invalid %s specified in config file: %s\n", description, name);
					System.exit(1);
				} else {
					names.add(fullJointName);
				}
			}
		} else { // complete name is specified (e.g. leftHipPitch, torso)
			names.add(uncapitalize(name));
		}
		return names;
	}

	private static HashSet<String> getLinkNames(HashSet<String> validJointNames) {
		// There seems to be no simple way to get a list of links. In principle, after the SDF model
		// is read, we could get the link names as the children of the SDFJointHolders in the Generalized SDF model.
		// But getting the SDF model to access the joint holders makes us unable to apply the mutators we're
		// trying to define.
		// For now, this is a hack.
		HashSet<String> namingExceptions = new HashSet<String>(Arrays.asList(
												"pelvis", "torsoRoll", "hokuyo_joint",
												"leftAnkleRoll", "leftWristPitch","leftForearmYaw",
												"rightAnkleRoll", "rightWristPitch","rightForearmYaw"));
		HashSet<String> validLinkNames = new HashSet<>();
		for (String jointName : validJointNames) {
			if (! namingExceptions.contains(jointName))
			validLinkNames.add(jointName + "Link");
		}

		// Add exceptions to the rule
		validLinkNames.addAll(Arrays.asList("pelvis", "torso", "leftFoot", "leftPalm", "rightFoot", "rightPalm",
				                            "hokuyo_link", "leftForearmLink", "rightForearmLink"));		
		return validLinkNames;
	}
	
	public void addMutator(SDFDescriptionMutator mutator)
	{
		
	}
	
}
