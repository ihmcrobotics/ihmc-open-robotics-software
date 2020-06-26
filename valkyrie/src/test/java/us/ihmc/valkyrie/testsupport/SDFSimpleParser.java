package us.ihmc.valkyrie.testsupport;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.behaviors.examples.GetLidarScanExampleBehavior;

/**
 * SDF parser specialized for manipulating link lengths
 * @author Mark Paterson
 *
 */
public class SDFSimpleParser extends ValkyrieDomParser {
	
	public SDFSimpleParser(InputStream stream) throws ParserConfigurationException, SAXException, IOException {
		super(stream);

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
	
	/**
	 * Given a link name, find the next child link down the chain
	 * @param linkName
	 * @return name of the child link or null
	 */
	private String getChildLinkOfLink(String linkName) {
		String childJointName = getChild(linkName);
		return getChild(childJointName);
	}
	
	public void scaleLinkByPercent(String linkName, double percent) {
		double linkLength = getLinkLength(linkName);
		double newLinkLength = linkLength*(1+percent);
		scaleLinkLength(linkName, newLinkLength);
	}
	
	/**
	 * For the given link, adjust the mesh specified by xpath by scaleFactor in the scale direction for this link.
	 * @param link -- the link element
	 * @param scaleFactor -- how much to scale the mesh
	 * @param xpath -- relative path under the link to the mesh scale element
	 */
	private void scaleMesh(Element link, double scaleFactor, String xpath)
	{
		String linkName = link.getAttribute("name");
		
		Element meshScaleElement = getUniqueChildElement(link, xpath);
		if (meshScaleElement == null) {
			System.out.println("Unable to find a mesh scale for " + linkName);
			return; // For Valkyrie, we always expect a mesh
		}
		
		SizableVector meshScale = new SizableVector(meshScaleElement.getTextContent());
		
		// Look up which direction to scale this mesh. Unfortunately, this is not consistent across links since it
		// depends on how the mesh is represented in the model (dae) file.
		Scale_Direction direction = getScaleDirection(link.getAttribute("name"));
		if (direction == null) {
			throw new IllegalArgumentException("Cannot find scale direction for link " + linkName);
		}
		int meshIndex = direction.index;

		meshScale.scaleIndex(meshIndex, scaleFactor);
		meshScaleElement.setTextContent(meshScale.toString());
	}	
	
	/**
	 * For the given link name, scale the link to the new length. Scaling a link in SDF is quite invasive since
	 * poses in SDF are relative to the model root, so every child of the scaled link will have its pose changed.
	 * Link lengths are not specified in SDF, and can only be known indirectly by computing the offset to the child 
	 * link pose. Basic logic for scaling a link L:
	 *   get child link C of L
	 *   child offset = pose(C) - pose(L)
	 *   link length = length(child offset)   
	 *   scale factor = newLength / link length
	 *   offset delta = scaled child offset - child offset = (child offset) * (scale factor - 1)
	 *   add offset delta to pose of each descendant link of S
	 *   
	 *   scale inertial CoM pose by scale factor
	 *   scale inertial matrix by (scale factor)^2
	 *   scale model visual and collision meshes
 
	 * @param linkName -- name of the link
	 * @param newLength -- new size in meters
	 */
	public void scaleLinkLength(String linkName, double newLength) {
		/* Pseudo code for scaling a link S. This is quite invasive since in SDF link poses are relative to model root
		 * Inputs: link to scale, scale factor
		 * child link C = childLinkOf(S)
		 * offset = pose(C) - pose(S)
		 * delta = offset*(1-scaleFactor)
		 * while child = childOf(S) is not null { pose(child) += offset }
		 */
		
		Element link = getLinkByName(linkName);
        String childLinkName = getChildLinkOfLink(linkName);
        if (childLinkName == null) { // If link is a leaf, how do I determine link length?
        	throw new IllegalArgumentException("Unable to scale a link that is a leaf node");
        }

        
        Element childLink = getLinkByName(childLinkName);
        SizableVector linkPose = new SizableVector(getUniqueChildElement(link, "pose").getTextContent());
        Element childLinkPoseElement = getUniqueChildElement(childLink, "pose");
        SizableVector childLinkPose = new SizableVector(childLinkPoseElement.getTextContent());
        SizableVector oldOffset = childLinkPose.subtract(linkPose);
        double oldLength = oldOffset.magnitude();
        double scaleFactor = newLength/oldLength;
        SizableVector deltaOffset = oldOffset.copy();
        deltaOffset.scale(scaleFactor - 1);
        	
        // Scale inertial CoM offset
        Element inertialElement = getUniqueChildElement(link, "inertial");
        Element inertialPoseElement = getUniqueChildElement(inertialElement, "pose");
        SizableVector inertialPose = new SizableVector(inertialPoseElement.getTextContent());
        inertialPose.scale(scaleFactor);
        inertialPoseElement.setTextContent(inertialPose.toString());
        
        // Scale inertial matrix
        Element inertiaMatrixElement = getUniqueChildElement(inertialElement, "inertia");
        double inertialScaleFactor = scaleFactor*scaleFactor; // Mass is held constant. Assume inertia should scale with length^2 
        for (String entry: Arrays.asList("ixx", "ixy", "ixz", "iyy", "iyz", "izz")) {
        	Element inertialSubElement = getUniqueChildElement(inertiaMatrixElement, entry);
        	double value = Double.parseDouble(inertialSubElement.getTextContent());
        	value *= inertialScaleFactor;
        	inertialSubElement.setTextContent(String.valueOf(value));
        }
        
        // Scale mesh
        scaleMesh(link, scaleFactor, "collision/geometry/mesh/scale");
        scaleMesh(link, scaleFactor, "visual/geometry/mesh/scale");
        
        		
        // Offset descendants
        while (childLinkName != null) {
        	childLink = getLinkByName(childLinkName);
        	childLinkPoseElement = getUniqueChildElement(childLink, "pose");
        	childLinkPose.fromText(childLinkPoseElement.getTextContent());
        	childLinkPose.add(deltaOffset);
        	childLinkPoseElement.setTextContent(childLinkPose.toString());	
        	childLinkName = getChildLinkOfLink(childLinkName);
        }      	
	}

	/**
	 * Get the length of the specified link
	 * @param linkName -- name of the link
	 * @return length of the link in meters
	 */
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
		SizableVector childFrameFromModel = new SizableVector(getUniqueChildElement(childLink, "pose").getTextContent());
		Vector3D offset = new Vector3D(childFrameFromModel.get(0) - linkFrameFromModel.get(0),
                        			   childFrameFromModel.get(1) - linkFrameFromModel.get(1),
				                       childFrameFromModel.get(2) - linkFrameFromModel.get(2));
		length = offset.length();

		return length;
	}

	// Test code
	public static void main(String[] args) {
		try {
			SDFSimpleParser parser = null;
			String file = "/home/mark/git/ihmc-open-robotics-software/valkyrie/src/main/resources/models/val_description/sdf/valkyrie_sim.sdf";
			InputStream sdfInputStream = new FileInputStream(new File(file));
			parser = new SDFSimpleParser(sdfInputStream);

			System.out.printf("Child of %s is %s\n", "leftHipYaw", parser.getChild("leftHipYaw"));
			System.out.printf("Parent of %s is %s\n", "leftHipYaw", parser.getParent("leftHipYaw"));
			System.out.printf("Parent of root is %s\n", parser.getParent("pelvis"));
			
			String linkName = "rightHipPitchLink";
			System.out.printf("Length of link %s is %f\n", linkName, parser.getLinkLength(linkName));

			parser.scaleLinkByPercent("rightKneePitchLink", 0.4);
			parser.scaleLinkByPercent("leftKneePitchLink", 0.4);
			parser.scaleLinkByPercent("rightHipPitchLink", 0.4);
			parser.scaleLinkByPercent("leftHipPitchLink", 0.4);
			parser.writeToFile("/tmp/valkyrie_sim_mod.sdf");
			
		} catch (SAXException | IOException | ParserConfigurationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
