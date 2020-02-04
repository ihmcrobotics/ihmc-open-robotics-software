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
		STAIRS, STEP, SQUARE_UP_STEP;
	}

	public double stepStartingDistance;
	public double stepHeight;
	public int numberOfSteps;
	public HashMap<String, Double> torqueLimits;
	public boolean showGui;
	public TestType testType;
	public String footstepsFile;
	
	@Expose (serialize=false, deserialize=false)
	private final JSONSerializer<FootstepDataListMessage> FootstepDataListMessageSerializer = new JSONSerializer<>(new FootstepDataListMessagePubSubType());
	
	// Default constructor
	public ValkyrieTorqueSpeedTestConfig() {
		stepStartingDistance = 1.0 * 100.0 / 2.54; // 1m in inches
		stepHeight = 6.0;
		numberOfSteps = 3;
		torqueLimits = new HashMap<String, Double>();
		showGui = true;
		testType = TestType.STAIRS;
		footstepsFile = null;
	}

	public String toString() {
		String value = String.format("Test Type: %s\nStep Starting Distance: %f\nStep Height: %f\nNumber of Steps: %d\nShow Gui: %b\n",
				testType, stepStartingDistance, stepHeight, numberOfSteps, showGui);
		if (footstepsFile != null) {
			value += String.format("Footsteps Filename: %s\n", footstepsFile);
		}
		for (String joint : torqueLimits.keySet()) {
			value += String.format("%s joint torque limit: %f\n", joint, torqueLimits.get(joint).doubleValue());
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
