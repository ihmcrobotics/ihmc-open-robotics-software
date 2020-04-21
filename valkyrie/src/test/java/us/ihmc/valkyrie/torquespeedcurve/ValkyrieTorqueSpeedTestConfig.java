package us.ihmc.valkyrie.torquespeedcurve;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.yaml.snakeyaml.Yaml;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.gson.annotations.Expose;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataListMessagePubSubType;
import us.ihmc.humanoidBehaviors.behaviors.examples.GetLidarScanExampleBehavior;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.valkyrie.testsupport.ModifiableValkyrieRobotConfig;

class ValkyrieTorqueSpeedTestConfig implements ModifiableValkyrieRobotConfig {
	enum TestType {
		STAIRS, STEP, SQUARE_UP_STEP, STEP_DOWN, SLOPE, SPEED, PUSHRECOVERY, DUMMY;
	}

	public String testCase;             // for push recovery, specifies which case to run
	public boolean keepUp;              // whether to keep up the GUI after running
	public double [] forceVector; // for push recovery, specifies direction of force
	public double forceMagnitude;       // for push recovery, specifies magnitude of the force
	public double forceDuration;        // for push recovery, specifies the duration of the force
	public double stepStartingDistance; // for step/stair scenarios, distance to the first step (inches)
	public double stepHeight;           // for step/stairs scenarios, height of each step (inches)
	public int numberOfSteps;           // for step/stair scenarios, number of steps in the staircase
	public HashMap<String, Double> torqueLimits; // Map of joint name to overridden torque limit. Only overridden
	                                             // joints need to be specified.
	public HashMap<String, Double> linkMassKg;  // Map of link name to overridden link mass in kg. Only overridden
                                                // joints need to be specified.	
	public HashMap<String, Double> velocityLimits; // Map of joint name to overridden velocity limit.
	public HashMap<String, ArrayList<Double>> positionLimits; // Map of joint name to array containing lower and upper limit
	public boolean showGui;             // whether to pop up a GUI (required if a video is wanted)
	public boolean minimizeJointTorques; // Whether to minimize joint torques as part of the QP calculation
	public TestType testType;           // type of test to run (see TestType enum)
	public String footstepsFile;        // path to a file of footstep messages
	public boolean disableAnkleLimits;  // ignore ankle limits -- falls are sometimes due to hitting ankle limits rather than insufficient torque 
	public double slopeDegrees;         // for slope scenarios, the pitch of the slope (degrees). Positive values
	                                    // indicate an upward slope.
	public double stepLengthInches;     // for slope scenarios, the length of step to take in inches
	public double globalMassScale;      // Amount to scale mass across the robot
	public double globalSizeScale;      // Amount to scale size across the robot
	public double torsoPitchDegrees;    // Amount to pitch the torso, in degrees (0.0 = vertical, positve = forward pitch)
	public ValkyrieWalkingParameterValues walkingValues; // Tweaks to default walking settings
	
	@Override
	public HashMap<String, Double> getTorqueLimits() {
		return torqueLimits;
	}
	
	@Override
	public HashMap<String, Double> getLinkMassKg() {
		return linkMassKg;
	}
	
	@Override
	public HashMap<String, Double> getVelocityLimits() {
		return velocityLimits;
	}
	
	@Override
	public HashMap<String, ArrayList<Double>> getPositionLimits() {
		return positionLimits;
	}
	
	@Override
	public double getGlobalMassScale() {
		return globalMassScale;
	}
	
	@Override
	public double getGlobalSizeScale() {
		return globalSizeScale;
	}
	
	@Override
	public boolean getAnkleLimitsDisabled() {
		return disableAnkleLimits;
	}
	
	@Override
	public boolean getMinimizeJointTorques() {
		return minimizeJointTorques;
	}
	
	@Expose (serialize=false, deserialize=false)
	private final JSONSerializer<FootstepDataListMessage> FootstepDataListMessageSerializer = new JSONSerializer<>(new FootstepDataListMessagePubSubType());
	
	// Default constructor
	public ValkyrieTorqueSpeedTestConfig() {
		keepUp = false;
		forceVector = new double [3];
		forceMagnitude = 0.0;
		forceDuration = 1.0;
		stepStartingDistance = 1.0 * 100.0 / 2.54; // 1m in inches
		stepHeight = 6.0;
		numberOfSteps = 3;
		torqueLimits = new HashMap<String, Double>();
		linkMassKg = new HashMap<String, Double>();
		velocityLimits = new HashMap<String, Double>();
		positionLimits = new HashMap<String, ArrayList<Double> >();
		showGui = true;
		minimizeJointTorques = false;
		testType = TestType.STAIRS;
		footstepsFile = null;
		disableAnkleLimits = false;
		slopeDegrees = 5.0;
		stepLengthInches = 0.5 * 100.0 / 2.54; // 0.5 meters in inches
		globalMassScale = 1.0;
		globalSizeScale = 1.0;
		torsoPitchDegrees = 0.0; // 0.0 == vertical
		walkingValues = new ValkyrieWalkingParameterValues();
	}
	
	private String hashDoubleToString(HashMap<String, Double> map, String description) {
		String value = "";
		for (String key: map.keySet()) {
			value += String.format("%s %s: %f\n", key, description, map.get(key).doubleValue());
		}
		return value;
	}

	private String hashDoubleArrayToString(HashMap<String, ArrayList<Double>> map, String description) {
		String value = "";
		for (String key: map.keySet()) {
			value += String.format("%s %s: [", key, description);
			for (Double number: map.get(key) ) {
				value += String.valueOf(number) + " ";
			}
			value += "]\n";
		}
		return value;
	}	
	
	public String toString() {
		String value = String.format("Test Type: %s\nStep Starting Distance: %f\nStep Height: %f\nNumber of Steps: %d\nShow Gui: %b\nDisable Ankle Limits: %b\n",
				testType, stepStartingDistance, stepHeight, numberOfSteps, showGui, disableAnkleLimits);
		value += String.format("Slope Degrees: %f\nStep Length (Inches): %f\nMass scale: %f\nSize scale: %f\n",
				slopeDegrees, stepLengthInches, globalMassScale, globalSizeScale);
		value += walkingValues.toString();

		if (footstepsFile != null) {
			value += String.format("Footsteps Filename: %s\n", footstepsFile);
		}
		value += hashDoubleToString(torqueLimits, "torque limit");
		value += hashDoubleToString(linkMassKg, "link mass");
		value += hashDoubleToString(velocityLimits, "velocity limit");
		value += hashDoubleArrayToString(positionLimits, "position limit");
		return value;
	}
	
	public FootstepDataListMessage getFootsteps() {
	    FootstepDataListMessage footsteps = null;
		if (footstepsFile != null) {

			try (FileInputStream reader = new FileInputStream(footstepsFile)) {
				ObjectMapper objectMapper = new ObjectMapper();
				JsonNode jsonNode = null;

				if (footstepsFile.endsWith(".yaml")) {
					Yaml yaml = new Yaml();
					@SuppressWarnings("unchecked")
					Map<String, Object> map = (Map<String, Object>) yaml.load(reader);
					System.out.print(yaml.toString());
					jsonNode = objectMapper.valueToTree(map);

				} else if (footstepsFile.endsWith(".json")) {
					jsonNode = objectMapper.readTree(reader);
					System.out.print(jsonNode.toString());

				} else {
					throw new IllegalArgumentException(String.format("Cannot determine the type of the footsteps file: %s\n", footstepsFile));
				}

				// By default the deserializer expects the class name as the root node
				// with the value being the class contents. But it is simpler for us
				// for the file content just to be the class content. Setting
				// AddTypeAsRootNode to false accomplishes this.
				FootstepDataListMessageSerializer.setAddTypeAsRootNode(false);
				footsteps = FootstepDataListMessageSerializer.deserialize(jsonNode.toString());
			
			} catch (FileNotFoundException e) {
				System.err.println("Footstep file not found: " + footstepsFile);
			} catch (IOException e) {
				System.err.println("Exception encountered while deserializing footstep file");
				e.printStackTrace();
			}
		}
		return footsteps;
	}
}
