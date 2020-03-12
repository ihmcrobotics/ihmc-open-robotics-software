package us.ihmc.valkyrie.torquespeedcurve;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.core.Version;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.module.SimpleModule;
import org.yaml.snakeyaml.Yaml;
import com.google.common.io.Files;
import com.google.gson.Gson;
import com.google.gson.annotations.Expose;
import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataListMessagePubSubType;
import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessagePubSubType;
import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutatorList;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieSDFDescriptionMutator;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;
import us.ihmc.valkyrie.torquespeedcurve.ValkyrieJointTorqueLimitMutator;
import us.ihmc.valkyrie.torquespeedcurve.ValkyrieTorqueSpeedCurveEndToEndTestNasa;
import us.ihmc.valkyrie.torquespeedcurve.ValkyrieTorqueSpeedTestConfig;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieSDFDescriptionMutator;

import us.ihmc.idl.serializers.extra.JSONSerializer;

public class TorqueSpeedTestRunner {
	private final static double BACKPACK_MASS_KG = 8.6;
	
	private double inchesToMeters(double inches) {
		return inches * 2.54 / 100.0;
	}

	// Given a folder name, construct a top-level output directory
	private File getOutputDirectory(String folderName) {
		String homeDirectory = System.getProperty("user.home");
		Path path = Paths.get(homeDirectory, "test_results", folderName);
		try {
			FileTools.ensureDirectoryExists(path);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return path.toFile();
	}

	private static JSAP getArgumentParser() {
		JSAP argumentParser = new JSAP();
		FlaggedOption paramFileOption = new FlaggedOption("paramfile").setShortFlag('f').setLongFlag("paramfile")
				.setRequired(true);

		try {
			argumentParser.registerParameter(paramFileOption);
		} catch (JSAPException e1) {
			System.err.println("Unable to register option parameters");
			e1.printStackTrace();
			System.exit(1);
		}

		return argumentParser;
	}

	public static void main(String[] args) {

		JSAP argumentParser = getArgumentParser();
		JSAPResult arguments = argumentParser.parse(args);

		String paramFile = arguments.getString("paramfile");
		if (paramFile == null) {
			System.err.println("A parameter file must be specified.");
			System.err.println(argumentParser.getUsage());
			System.exit(1);
		}
		System.out.printf("Got param file of %s\n", paramFile);

		File paramInput = new File(paramFile);
		if (!paramInput.exists()) {
			System.err.printf("Parameter file %s does not exist\n", paramFile);
			System.exit(1);
		}

		// Read in the test configuration from the test parameters file
		Gson gson = new Gson();
		ValkyrieTorqueSpeedTestConfig config = null;

		try (Reader reader = new FileReader(paramFile)) {
			config = gson.fromJson(reader, ValkyrieTorqueSpeedTestConfig.class);
		} catch (IOException e1) {
			System.err.printf("Error reading test params from %s: %s\n", paramFile, e1.toString());
			System.exit(1);
		}
		System.out.println(config.toString());

		// If the config file specifies footsteps, read them in here. Otherwise
		// recordedFootsteps will be null.
		FootstepDataListMessage recordedFootsteps = config.getFootsteps();

		ValkyrieRobotModel robot = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT);

		// Scale robot mass and size
		robot.setModelMassScale(config.globalMassScale);
		robot.setModelSizeScale(config.globalSizeScale);

		// Set walking parameters
		ValkyrieTorqueSpeedWalkingControllerParameters walkingParameters = new ValkyrieTorqueSpeedWalkingControllerParameters(
				robot.getJointMap(),
				robot.getRobotPhysicalProperties(), 
				RobotTarget.REAL_ROBOT,
				config.walkingValues);

		// Apply SDF modifications
		SDFDescriptionMutatorList mutators = getSdfMutators(config, robot);

		// Create test runner and test case class
		TorqueSpeedTestRunner runner = new TorqueSpeedTestRunner();
		ValkyrieTorqueSpeedCurveEndToEndTestNasa tester = new ValkyrieTorqueSpeedCurveEndToEndTestNasa();

		// Disable ankle limits on the robot
		if (config.disableAnkleLimits) {
			ArrayList<Double> limits = new ArrayList<Double>();
			limits.add(-180.0);
			limits.add(180.0);
			mutators.addMutator(new ValkyrieJointPositionLimitMutator(robot.getJointMap(), "leftAnklePitch", limits));
			mutators.addMutator(new ValkyrieJointPositionLimitMutator(robot.getJointMap(), "rightAnklePitch", limits));
		}

		// Apply mutators to the robot. There is always at least one mutator.
	    robot.setSDFDescriptionMutator(mutators);

		// Set whether tester should show a GUI
		tester.setGuiEnabled(config.showGui);

		// Set up output directories for test results
		File outputPrefixDirectory = runner.getOutputDirectory("capped_torque");
		File outputResultsDirectory = null;

		// Run the test
		try {
			switch (config.testType) {
			case STAIRS:
				outputResultsDirectory = tester.testUpstairs(robot, config.stepHeight, config.numberOfSteps,
						walkingParameters, outputPrefixDirectory);
				break;
			case SQUARE_UP_STEP:
				outputResultsDirectory = tester.testStepUpWithSquareUp(robot, config.stepStartingDistance,
						config.stepHeight, walkingParameters, outputPrefixDirectory);
				break;
			case STEP:
				outputResultsDirectory = tester.testStepUpWithoutSquareUp(robot, config.stepStartingDistance,
						config.stepHeight, walkingParameters, recordedFootsteps, outputPrefixDirectory);
				break;
			case STEP_DOWN:
				outputResultsDirectory = tester.testStepDown(robot, config.stepStartingDistance, config.stepHeight,
						walkingParameters, recordedFootsteps, outputPrefixDirectory);
				break;
			case SLOPE:
				outputResultsDirectory = tester.testWalkSlope(robot, Math.toRadians(-config.slopeDegrees),
						runner.inchesToMeters(config.stepLengthInches), 
						walkingParameters, 
						recordedFootsteps,
						outputPrefixDirectory);
			case SPEED:
				outputResultsDirectory = tester.testSpeedWalk(robot, walkingParameters, outputPrefixDirectory);		
				break;
			}

		} catch (SimulationExceededMaximumTimeException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Copy the test parameters into the results directory
		try {
			if (outputResultsDirectory != null) {
				Files.copy(paramInput, Paths
						.get(outputResultsDirectory.toString(), paramInput.toPath().getFileName().toString()).toFile());
			}
		} catch (IOException e) {
			System.err.println("Unable to copy param file to destination: " + e.getMessage());
		}

		// Without calling System.exit(0), the sim has non-terminating threads.
		// Specifically, the simulation
		// thread and the intraprocess thread (associated with ROS2 publishing) do not
		// exit.
		System.exit(0);
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

	private static SDFDescriptionMutatorList getSdfMutators(ValkyrieTorqueSpeedTestConfig config,
			ValkyrieRobotModel robot) {

		// Create a list of joint names with the format we need
		ValkyrieJointMap jointMap = robot.getJointMap();
		HashSet<String> validJointNames = new HashSet<>(Arrays.asList(jointMap.getOrderedJointNames()));
		HashSet<String> validLinkNames = getLinkNames(validJointNames);
		
		// Create a list of mutators based on the torque limits defined in the params file.
		// The first mutator is of a different type, and exists to fix up various things in the Valkyrie model.
		SDFDescriptionMutatorList mutator_list = new SDFDescriptionMutatorList(
				new ValkyrieSDFDescriptionMutator(jointMap, false), new ValkyrieJointTorqueLimitMutator(jointMap));

		// Add a mutator for each joint torque limit specified in the parameter file
		for (String joint : config.torqueLimits.keySet()) {
			HashSet<String> jointNames = getCanonicalNames(joint, validJointNames, "torque limit joint name");
			for (String fullJointName: jointNames) {
				mutator_list.addMutator(new ValkyrieJointTorqueLimitMutator(jointMap, fullJointName,
						config.torqueLimits.get(joint)));
			}
		}		
		
		// Add a mutator for each joint velocity limit specified in the parameter file
		for (String joint : config.velocityLimits.keySet()) {
			HashSet<String> jointNames = getCanonicalNames(joint, validJointNames, "velocity limit joint name");
			for (String fullJointName: jointNames) {
				mutator_list.addMutator(new ValkyrieJointVelocityLimitMutator(jointMap, fullJointName,
						config.velocityLimits.get(joint)));
			}
		}
		
		// Add a mutator for each joint mass modifier specified in the parameter file
		for (String link : config.linkMassKg.keySet()) {
			HashSet<String> linkNames = getCanonicalNames(link, validLinkNames, "mass link name");
			for (String fullLinkName: linkNames) {
				double desiredMass = config.linkMassKg.get(link);
				mutator_list.addMutator(new ValkyrieLinkMassMutator(jointMap, fullLinkName, desiredMass));
			}
		}
		
		// Add a mutator for each joint position limit modifier in the parameter file
		for (String joint : config.positionLimits.keySet()) {
			HashSet<String> jointNames = getCanonicalNames(joint, validJointNames, "position limit joint name");
			for (String fullJointName: jointNames) {
				mutator_list.addMutator(new ValkyrieJointPositionLimitMutator(jointMap, fullJointName,
						config.positionLimits.get(joint)));
			}
		}
		return mutator_list;
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

}
