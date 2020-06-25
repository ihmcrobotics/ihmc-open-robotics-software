package us.ihmc.valkyrie.testsupport;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
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

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

public class DomParser {
	private Document document;
	private Element root;
	private HashMap<String, Element> links;
	private HashMap<String, Element> joints;
	private HashMap<String, String> parentOf = new HashMap<String, String>();
	private HashMap<String, String> childOf = new HashMap<String, String>();
	
	protected XPath xPath;
	
	public DomParser(String file) throws ParserConfigurationException, SAXException, IOException {
		DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
		DocumentBuilder builder = factory.newDocumentBuilder();

		document = builder.parse(new File(file));
		root = document.getDocumentElement();
		links = new HashMap<String, Element>();
		joints = new HashMap<String, Element>();
		xPath = XPathFactory.newInstance().newXPath();
	}
	
	public Element getRoot() {
		return root;
	}
	
	protected void fillJointMap(Element parent, String tag) {
		fillMap(parent, tag, joints);
	}
	
	protected void fillLinkMap(Element parent, String tag) {
		fillMap(parent, tag, links);
	}
	
	protected int getNumberOfJoints() {
		return joints.size();
	}
	
	protected int getNumberOfLinks() {
		return links.size();
	}
	
	protected HashMap<String, Element> getJoints() {
		return joints;
	}
	
	protected HashMap<String, Element> getLinks() {
		return links;
	}
	
	public void addParentChildRelationship(String parent, String child) {
		childOf.put(parent, child);
		parentOf.put(child, parent);
	}
	
	public String getParent(String child) {
		return parentOf.getOrDefault(child, null);
	}
	
	public String getChild(String parent) {
		return childOf.getOrDefault(parent, null);
	}	
	
	/**
	 * Quick access to joint elements by name.
	 * 
	 * @param jointName The name of the joint
	 * @return Joint element or null if the jointName is not found
	 */
	public Element getJointByName(String jointName) {
		return joints.getOrDefault(jointName, null);
	}	
	
	/**
	 * Quick access to link elements by name.
	 * 
	 * @param linkName The name of the link
	 * @return Link element or null if the linkName is not found
	 */
	public Element getLinkByName(String linkName) {
		return links.getOrDefault(linkName, null);
	}	
	
	/**
	 * Creates in a map of name to element for quick access. Elements must be direct children of the root node.
	 * 
	 * @param Name of the tag to search for (e.g. link, joint)
	 * @param HashMap to fill in with the results
	 */
	private void fillMap(Element parent, String tag, HashMap<String, Element> map)
	{
		// Annoyingly, both joint nodes and joints within transmissions have a tag of "joint" so
		// it's not sufficient to get nodes by element name. Instead, we grab nodes that are 
		// direct children of the root node, then filter out what we need.
		NodeList elements = getDirectChildrenOfType(parent, tag);
		for (int index = 0; index < elements.getLength(); index++) {
			Element element;
            try {
			    element = (Element) elements.item(index);
			} catch (ClassCastException e) {
            	continue;
            }

			if (! element.hasAttribute("name")) {
				System.err.printf("Missing name for %s\n", tag);
			} else {
				String name = element.getAttribute("name");
				if (map.containsKey(name)) {
					System.err.printf("Duplicate %s with name %s\n", tag, name);
				} else {
					System.out.printf("Adding %s %s\n", tag, name);
					map.put(name, element);
				}
			}
		}		
	}
	
	public Element getUniqueChildElement(Element parent, String name) {
		NodeList filteredChildren = getDirectChildrenOfType(parent, name);
		assert(filteredChildren != null);
		
		if (filteredChildren.getLength() != 1) {
			System.out.printf("Did not find a unique %s\n", name);
			return null;
		}
		return (Element) filteredChildren.item(0);			
	}
	
	public NodeList getDirectChildrenOfType(Element element, String typeName) {
		assert(element != null);

		String expression = String.format("child::%s", typeName);
		NodeList filteredChildren = null;
		try {
			filteredChildren = (NodeList) xPath.compile(expression).evaluate(element, XPathConstants.NODESET);
		} catch (XPathExpressionException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return filteredChildren;
		
	}
	
	/**
	 * Output the current content of the DOM document to a file
	 * 
	 * @param fileName Path to the file
	 */
	public void writeToFile(String fileName) {
		TransformerFactory transformerFactory = TransformerFactory.newInstance();
		Transformer transformer = null;
		try {
			transformer = transformerFactory.newTransformer();
			transformer.setOutputProperty(OutputKeys.INDENT, "yes");
			transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");			
		} catch (TransformerConfigurationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("Unable to create new transformer");
			return;
		}
		DOMSource source = new DOMSource(document);
		FileWriter writer;
		try {
			writer = new FileWriter(new File(fileName));
			StreamResult result = new StreamResult(writer);
			
			transformer.transform(source, result);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("Unable to create file writer");
		} catch (TransformerException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("Unable to transform source document");
		}
	}
		
}
