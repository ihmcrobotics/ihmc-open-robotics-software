package us.ihmc.valkyrie.testsupport;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Map;

import javax.xml.parsers.ParserConfigurationException;

import org.xml.sax.SAXException;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutatorList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieSDFDescriptionMutator;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties;
import us.ihmc.valkyrie.torquespeedcurve.ValkyrieJointPositionLimitMutator;
import us.ihmc.valkyrie.torquespeedcurve.ValkyrieJointTorqueLimitMutator;
import us.ihmc.valkyrie.torquespeedcurve.ValkyrieJointVelocityLimitMutator;
import us.ihmc.valkyrie.torquespeedcurve.ValkyrieLinkMassMutator;

/* ValkyrieRobotModel with SDF modifications */
public class ModifiableValkyrieRobotModel extends ValkyrieRobotModel {
	private ModifiableValkyrieRobotConfig config;

	public ModifiableValkyrieRobotModel(RobotTarget target, ModifiableValkyrieRobotConfig robotConfig) {
		super(target);

		config = robotConfig;

		// Scale robot mass and size
		setModelMassScale(config.getGlobalMassScale());
		setModelSizeScale(config.getGlobalSizeScale());

		// Configure whether to minimize joint torques
		WholeBodyInverseDynamicsSolver.MinimizeJointTorques(config.getMinimizeJointTorques());

		// Set up SDF modifications (applied later, when reading in the SDF file)
		setSDFDescriptionMutator(getSdfMutators(config));
	
		// Apply any SDF pre-processing
		applySDFPreProcessing();
	}

	protected void applySDFPreProcessing() {
		Map<String, Double> linkLengths = config.getModifiedLinkLengths();
		InputStream originalSdfInput = super.getSDFModelInputStream();
		SDFSimpleParser parser = null;
		boolean modificationsPerformed = false;
		
		try {
			parser = new SDFSimpleParser(originalSdfInput);
		} catch (ParserConfigurationException | SAXException | IOException e) {
			System.out.println("Caught exeception while trying to parse SDF input stream");
			e.printStackTrace();
		}		

		if (linkLengths != null) {
			System.out.printf("Applying SDF Pre-processing to %d links\n", linkLengths.size());
			modificationsPerformed = true;
			for (Map.Entry<String, Double> entry: linkLengths.entrySet()) {
				System.out.printf("Scaling link %s by %f\n", entry.getKey(), entry.getValue());
				parser.scaleLinkLength(entry.getKey(), entry.getValue());
			}			
		}
		
		if (modificationsPerformed) {
			File tempFile = null;
			try {
				tempFile = File.createTempFile("valkyrie_sim_mod", ".sdf");
			} catch (IOException e) {
				System.out.println("Unable to write updated SDF model to temp file");
				e.printStackTrace();
			}
			String tempFilePath = tempFile.getAbsolutePath();
			parser.writeToFile(tempFilePath);
			setCustomModel(tempFilePath);
			
			robotPhysicalProperties = new ModifiableValkyriePhysicalProperties(config, parser);
		}
	}

	@Override
	public ValkyriePhysicalProperties getRobotPhysicalProperties() {
		if (robotPhysicalProperties == null)
			robotPhysicalProperties = new ModifiableValkyriePhysicalProperties(config);
		return robotPhysicalProperties;
	}
	
	private SDFDescriptionMutatorList getSdfMutators(ModifiableValkyrieRobotConfig config) {

		// Create a list of joint names with the format we need
		ValkyrieJointMap jointMap = getJointMap();
		HashSet<String> validJointNames = new HashSet<>(Arrays.asList(jointMap.getOrderedJointNames()));
		HashSet<String> validLinkNames = getLinkNames(validJointNames);

		// Create a list of mutators based on the torque limits defined in the params
		// file.
		// The first mutator is of a different type, and exists to fix up various things
		// in the Valkyrie model.
		SDFDescriptionMutatorList mutator_list = new SDFDescriptionMutatorList(
				new ValkyrieSDFDescriptionMutator(jointMap, false), new ValkyrieJointTorqueLimitMutator(jointMap));

		// Add a mutator for each joint torque limit specified in the parameter file
		for (String joint : config.getTorqueLimits().keySet()) {
			HashSet<String> jointNames = getCanonicalNames(joint, validJointNames, "torque limit joint name");
			for (String fullJointName : jointNames) {
				mutator_list.addMutator(new ValkyrieJointTorqueLimitMutator(jointMap, fullJointName,
						config.getTorqueLimits().get(joint)));
			}
		}

		// Add a mutator for each joint velocity limit specified in the parameter file
		for (String joint : config.getVelocityLimits().keySet()) {
			HashSet<String> jointNames = getCanonicalNames(joint, validJointNames, "velocity limit joint name");
			for (String fullJointName : jointNames) {
				mutator_list.addMutator(new ValkyrieJointVelocityLimitMutator(jointMap, fullJointName,
						config.getVelocityLimits().get(joint)));
			}
		}

		// Add a mutator for each joint mass modifier specified in the parameter file
		for (String link : config.getLinkMassKg().keySet()) {
			HashSet<String> linkNames = getCanonicalNames(link, validLinkNames, "mass link name");
			for (String fullLinkName : linkNames) {
				double desiredMass = config.getLinkMassKg().get(link);
				mutator_list.addMutator(new ValkyrieLinkMassMutator(jointMap, fullLinkName, desiredMass));
			}
		}

		// Add a mutator for each joint position limit modifier in the parameter file
		for (String joint : config.getPositionLimits().keySet()) {
			HashSet<String> jointNames = getCanonicalNames(joint, validJointNames, "position limit joint name");
			for (String fullJointName : jointNames) {
				mutator_list.addMutator(new ValkyrieJointPositionLimitMutator(jointMap, fullJointName,
						config.getPositionLimits().get(joint)));
			}
		}

		// Disable ankle limits on the robot. This can already be done generically; this
		// option is for
		// backwards compatibility.
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
	 * Leg and arm joints/links should be parallel on each side, so we allow user to
	 * specify, for example, "HipPitch" and we translate that into a pair of names
	 * for left and right side.
	 * 
	 * @param name                -- incoming name to resolve
	 * @param fullyQualifiedNames -- set of valid names to match
	 * @param description         -- description of what sort of object we're
	 *                            working with for error output
	 * @return list of fully qualified names based on the input name
	 */
	// and we translate that into a pair of SDF mutations for leftHipPitch and
	// rightHipPitch
	private static HashSet<String> getCanonicalNames(String name, HashSet<String> fullyQualifiedNames,
			String description) {
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
	
	@Override
	protected InputStream getSDFModelInputStream() {
		// Inject the SDF model filename from the config file, if there is one
		String modelFile = config.getModelFile();
		if (modelFile != null) {
			setCustomModel(modelFile);
		}
		return super.getSDFModelInputStream();
	}

	private static HashSet<String> getLinkNames(HashSet<String> validJointNames) {
		// TODO: There seems to be no simple way to get a list of links. In principle, after
		// the SDF model is read, we could get the link names as the children of the SDFJointHolders
		// in the Generalized SDF model.
		// But getting the SDF model to access the joint holders makes us unable to apply the mutators we're
		// trying to define. For now, this is a hack.
		HashSet<String> namingExceptions = new HashSet<String>(
				Arrays.asList("pelvis", "torsoRoll", "hokuyo_joint", "leftAnkleRoll", "leftWristPitch",
						"leftForearmYaw", "rightAnkleRoll", "rightWristPitch", "rightForearmYaw"));
		HashSet<String> validLinkNames = new HashSet<>();
		for (String jointName : validJointNames) {
			if (!namingExceptions.contains(jointName))
				validLinkNames.add(jointName + "Link");
		}

		// Add exceptions to the rule
		validLinkNames.addAll(Arrays.asList("pelvis", "torso", "leftFoot", "leftPalm", "rightFoot", "rightPalm",
				"hokuyo_link", "leftForearmLink", "rightForearmLink"));
		return validLinkNames;
	}

	public void addMutator(SDFDescriptionMutator mutator) {

	}
}
