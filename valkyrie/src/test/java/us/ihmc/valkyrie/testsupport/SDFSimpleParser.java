package us.ihmc.valkyrie.testsupport;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.behaviors.examples.GetLidarScanExampleBehavior;

public class SDFSimpleParser extends DomParser {


	public SDFSimpleParser(String file) throws ParserConfigurationException, SAXException, IOException {
		super(file);

		Element root = getRoot();
		fillLinkMap(root, "model/link");
		fillJointMap(root, "model/joint");

		System.out.printf("Found %d links and %d joints\n", getNumberOfLinks(), getNumberOfJoints());

		for (Map.Entry<String, Element> entry : getJoints().entrySet()) {
			Element joint = entry.getValue();
			Element child = getUniqueChildElement(joint, "child");
			if (child != null) {
				addParentChildRelationship(entry.getKey(), child.getTextContent());
			}
			Element parent = getUniqueChildElement(joint, "parent");
			if (parent != null) {
				addParentChildRelationship(parent.getTextContent(), entry.getKey());
			}
		}
	}

	public double getLinkLength(String linkName) {
		double length = 0;

		String childJointName = getChild(linkName);		
		String childLinkName  = getChild(childJointName);
		
		if (childJointName == null || childLinkName == null ) {
			System.out.printf("Unable to get length of link %s\n", linkName);
			return 0;
		}

		Element link = getLinkByName(linkName);
		Element childLink = getLinkByName(childLinkName);

		SizableVector linkFrameFromModel = new SizableVector(getUniqueChildElement(link, "pose").getTextContent());
		SizableVector childFrameFromRoot = new SizableVector(getUniqueChildElement(childLink, "pose").getTextContent());
		Vector3D offset = new Vector3D(childFrameFromRoot.get(0) - linkFrameFromModel.get(0),
                        			   childFrameFromRoot.get(1) - linkFrameFromModel.get(1),
				                       childFrameFromRoot.get(2) - linkFrameFromModel.get(2));
		length = offset.length();

		return length;
	}

	// Test code
	public static void main(String[] args) {
		try {
			SDFSimpleParser parser = new SDFSimpleParser("/home/mark/git/ihmc-open-robotics-software/valkyrie/src/main/resources/models/val_description/sdf/valkyrie_sim.sdf");
			parser.writeToFile("/tmp/valkyrie_sim_mod.sdf");

			System.out.printf("Child of %s is %s\n", "leftHipYaw", parser.getChild("leftHipYaw"));
			System.out.printf("Parent of %s is %s\n", "leftHipYaw", parser.getParent("leftHipYaw"));
			System.out.printf("Parent of root is %s\n", parser.getParent("pelvis"));
			
			String linkName = "rightHipPitchLink";
			System.out.printf("Length of link %s is %f\n", linkName, parser.getLinkLength(linkName));
			
		} catch (ParserConfigurationException | SAXException | IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
