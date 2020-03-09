package us.ihmc.valkyrie.torquespeedcurve;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.yaml.snakeyaml.Yaml;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.gson.annotations.Expose;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataListMessagePubSubType;
import us.ihmc.idl.serializers.extra.JSONSerializer;

class ValkyrieTorqueSpeedTestConfig {
	enum TestType {
		STAIRS, STEP, SQUARE_UP_STEP, STEP_DOWN, SLOPE;
	}

	public double stepStartingDistance; // for step/stair scenarios, distance to the first step (inches)
	public double stepHeight;           // for step/stairs scenarios, height of each step (inches)
	public int numberOfSteps;           // for step/stair scenarios, number of steps in the staircase
	public HashMap<String, Double> torqueLimits; // Map of joint name to overridden torque limit. Only overridden
	                                             // joints need to be specified.
	public HashMap<String, Double> linkMassKg;  // Map of link name to overridden link mass in kg. Only overridden
                                                // joints need to be specified.	
	public boolean showGui;             // whether to pop up a GUI (required if a video is wanted)
	public TestType testType;           // type of test to run (see TestType enum)
	public String footstepsFile;        // path to a file of footstep messages
	public boolean disableAnkleLimits;  // ignore ankle limits -- falls are sometimes due to hitting ankle limits rather than insufficient torque 
	public double slopeDegrees;         // for slope scenarios, the pitch of the slope (degrees). Positive values
	                                    // indicate an upward slope.
	public double stepLengthInches;     // for slope scenarios, the length of step to take in inches
	public double globalMassScale;      // Amount to scale mass across the robot
	public double globalSizeScale;      // Amount to scale size across the robot
	
	@Expose (serialize=false, deserialize=false)
	private final JSONSerializer<FootstepDataListMessage> FootstepDataListMessageSerializer = new JSONSerializer<>(new FootstepDataListMessagePubSubType());
	
	// Default constructor
	public ValkyrieTorqueSpeedTestConfig() {
		stepStartingDistance = 1.0 * 100.0 / 2.54; // 1m in inches
		stepHeight = 6.0;
		numberOfSteps = 3;
		torqueLimits = new HashMap<String, Double>();
		linkMassKg = new HashMap<String, Double>();
		showGui = true;
		testType = TestType.STAIRS;
		footstepsFile = null;
		disableAnkleLimits = false;
		slopeDegrees = 5.0;
		stepLengthInches = 0.5 * 100.0 / 2.54; // 0.5 meters in inches
		globalMassScale = 1.0;
		globalSizeScale = 1.0;
	}

	public String toString() {
		String value = String.format("Test Type: %s\nStep Starting Distance: %f\nStep Height: %f\nNumber of Steps: %d\nShow Gui: %b\nDisable Ankle Limits: %b\n",
				testType, stepStartingDistance, stepHeight, numberOfSteps, showGui, disableAnkleLimits);
		value += String.format("Slope Degrees: %f\nStep Length (Inches): %f\nMass scale: %f\nSize scale: %f\n",
				slopeDegrees, stepLengthInches, globalMassScale, globalSizeScale);

		if (footstepsFile != null) {
			value += String.format("Footsteps Filename: %s\n", footstepsFile);
		}
		for (String joint : torqueLimits.keySet()) {
			value += String.format("%s joint torque limit: %f\n", joint, torqueLimits.get(joint).doubleValue());
		}
		for (String joint : linkMassKg.keySet()) {
			value += String.format("%s link mass: %f\n", joint, linkMassKg.get(joint).doubleValue());
		}
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
