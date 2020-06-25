package us.ihmc.valkyrie.testsupport;

import org.apache.poi.hssf.record.SCLRecord;

// Import an URDF into a DOM parser for modification

import org.w3c.dom.*;
import org.xml.sax.SAXException;

import javax.xml.parsers.*;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerConfigurationException;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpressionException;
import javax.xml.xpath.XPathFactory;

import java.io.*;
import java.util.HashMap;
import java.util.Map;

/**
 * Parser for URDF files, providing quick access to joints and links and the ability to write out to a file afterwards.
 * 
 * @author Mark Paterson
 *
 */
public class UrdfDomManipulator extends ValkyrieDomParser {	
	UrdfDomManipulator(String file) throws ParserConfigurationException, SAXException, IOException {
		super(file);
		
		Element root = getRoot();
		fillLinkMap(root, "link");
		fillJointMap(root, "joint");
		
		System.out.printf("Found %d links and %d joints\n", getNumberOfLinks(), getNumberOfJoints());
		
		for (Map.Entry<String, Element> entry : getJoints().entrySet()) {
			Element joint = entry.getValue();
			Element child = getUniqueChildElement(joint, "child");
			if (child != null) {
				addParentChildRelationship(entry.getKey(), child.getAttribute("link"));
			}
			Element parent = getUniqueChildElement(joint, "parent");
			if (parent != null) {
				addParentChildRelationship(parent.getAttribute("link"), entry.getKey());
			}
		}		
		// TODO: Consider adding schema validation here. URDF schema is available in the ROS urdfdom package
	}
	
	/**
	 * Set origin on a joint
	 */
	public void setJointOrigin(String jointName, String originValueString) {
		Element jointOrigin = getJointOrigin(jointName);
		jointOrigin.setAttribute("xyz", originValueString);
	}

	/** Retrieve offset for a joint
	 * 
	 * @param jointName Name of the joint whose offset should be retrieved
	 * @return Joint Element
	 */
	public Element getJointOrigin(String jointName) {
		Element joint = getJointByName(jointName);
		return getUniqueChildElement(joint, "origin");
	}
	
	private void scaleMesh(Element link, double scaleFactor, String xpath)
	{
		Element visualMesh = getUniqueChildElement(link, xpath);
		
		SizableVector meshScale = null;
		if (visualMesh.hasAttribute("scale")) {
			meshScale = new SizableVector(visualMesh.getAttribute("scale"));
		} else {
			meshScale = new SizableVector("1 1 1");			
		}
		
		String linkName = link.getAttribute("name");
		System.out.printf("Scaling link mesh for %s\n", linkName);
		
		// Look up which direction to scale this mesh. Unfortunately, this is not consistent
		int meshIndex = getScaleDirection(linkName).index;

		meshScale.scaleIndex(meshIndex, scaleFactor);
		visualMesh.setAttribute("scale", meshScale.toString());
	}
	
	
	/** Scale joint offset by a percentage
	 * 
	 * @param jointName Name of the joint whose offset should be scaled
	 * @param percent Percent to increase (if positive)/decrease (if negative) joint offset size
	 */
	public void scaleLinkByPercent(String linkName, double percent) {
		String childJoint = getChild(linkName);		
		if (childJoint == null) {
			throw new IllegalArgumentException("Unable to find a parent for " + linkName);
		}

		Element joint = getJointByName(childJoint);
		Element jointOrigin = getUniqueChildElement(joint, "origin");
		String originValueString = jointOrigin.getAttribute("xyz");
		System.out.printf("Origin value is %s\n", originValueString);
		
		// Scale joint origin
		SizableVector offset = new SizableVector(originValueString);
		double originalLength = offset.length();
		double newLength = originalLength * (1 + percent);
		
		scaleJointOffsetElement(joint, newLength);
	}
	
	/** Scale joint offset to a specific length
	 * 
	 * @param jointName Name of the joint whose offset should be scaled
	 * @param scaleFactor Multiplier to use
	 */
	public void setLinkLength(String linkName, double newLength) {
		String childJoint = getChild(linkName);		
		if (childJoint == null) {
			throw new IllegalArgumentException("Unable to find a parent for " + linkName);
		}

		Element joint = getJointByName(childJoint);
		scaleJointOffsetElement(joint, newLength);
	}
	
	public void scaleJointOffsetElement(Element joint, double newLength) {

		Element jointOrigin = getUniqueChildElement(joint, "origin");
		String originValueString = jointOrigin.getAttribute("xyz");
		System.out.printf("Origin value is %s\n", originValueString);
		
		// Scale joint origin
		SizableVector offset = new SizableVector(originValueString);
		System.out.printf("Offset is %s\n", offset.toString());
		double originalLength = offset.length();
		double scaleFactor = newLength/originalLength;
		System.out.printf("Original length: %f, Scale factor is %f\n",originalLength, scaleFactor);
		offset.scale(scaleFactor);
		jointOrigin.setAttribute("xyz", offset.toString());
		
		// Scale parent link inertial center of mass offset
		String parentLinkName = getUniqueChildElement(joint, "parent").getAttribute("link");
		Element parentLink = getLinkByName(parentLinkName);
		Element inertial = getUniqueChildElement(parentLink, "inertial");
		Element inertialCoM = getUniqueChildElement(inertial, "origin");
		originValueString = inertialCoM.getAttribute("xyz");
		offset = new SizableVector(originValueString);
		offset.scale(scaleFactor);
		inertialCoM.setAttribute("xyz", offset.toString());
		
		// Scale inertial matrix
		Element inertialMatrix = getUniqueChildElement(inertial, "inertia");
		SizableVector matrix = new SizableVector(6);
		matrix.set(0, Double.valueOf(inertialMatrix.getAttribute("ixx")));
		matrix.set(1, Double.valueOf(inertialMatrix.getAttribute("ixy")));
		matrix.set(2, Double.valueOf(inertialMatrix.getAttribute("ixz")));
		matrix.set(3, Double.valueOf(inertialMatrix.getAttribute("iyy")));
		matrix.set(4, Double.valueOf(inertialMatrix.getAttribute("iyz")));
		matrix.set(5, Double.valueOf(inertialMatrix.getAttribute("izz")));
		matrix.scale(scaleFactor*scaleFactor); // Assumption: inertia is proportional to the square of the length
		
		inertialMatrix.setAttribute("ixx", String.valueOf(matrix.get(0)));
		inertialMatrix.setAttribute("ixy", String.valueOf(matrix.get(1)));
		inertialMatrix.setAttribute("ixz", String.valueOf(matrix.get(2)));
		inertialMatrix.setAttribute("iyy", String.valueOf(matrix.get(3)));		
		inertialMatrix.setAttribute("iyz", String.valueOf(matrix.get(4)));
		inertialMatrix.setAttribute("izz", String.valueOf(matrix.get(5)));
		
		// Scale visual mesh
		scaleMesh(parentLink, scaleFactor, "visual/geometry/mesh");
		
		// Scale collision mesh
		scaleMesh(parentLink, scaleFactor, "collision/geometry/mesh");
	}
	

	/* Test code */
	public static void main(String[] args) {
		try {
			UrdfDomManipulator parser = new UrdfDomManipulator("/home/mark/git/ihmc-open-robotics-software/valkyrie/src/main/resources/models/val_description/sdf/valkyrie_sim_ihmc.urdf");
			parser.scaleLinkByPercent("rightKneePitchLink", 0.4);
			parser.scaleLinkByPercent("leftKneePitchLink", 0.4);
			parser.scaleLinkByPercent("rightHipPitchLink", 0.4);
			parser.scaleLinkByPercent("leftHipPitchLink", 0.4);
			parser.writeToFile("/tmp/valkyrie_sim_mod.urdf");
			
		} catch (ParserConfigurationException | SAXException | IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}

