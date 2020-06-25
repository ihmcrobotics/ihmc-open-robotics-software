package us.ihmc.valkyrie.testsupport;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import javax.xml.parsers.ParserConfigurationException;

import org.xml.sax.SAXException;

public class ValkyrieDomParser extends DomParser {
	
	public ValkyrieDomParser(String file) throws ParserConfigurationException, SAXException, IOException {
		super(file);
	}
	
	// Enum for mesh scaling. This should correspond to the mesh scaling parameter in the URDF, i.e.
	// X = index 0, Y = index 1, Z = index 2
	enum Scale_Direction {
		SCALE_X(0),
		SCALE_Y(1),
		SCALE_Z(2);
		
		public int index;
		
		private Scale_Direction(int index) {
			this.index = index;
		}
	}
	
	// Mapping from links to scaling direction. We don't want to uniformly scale the mesh because causes
	// issues with overlapping collision meshes and in any case looks odd. Instead, we'll stretch the mesh
	// in the "length" direction, whatever that is. This map indicates the direction of stretch.
	private static Map<String, Scale_Direction> scaleDirection;
	static {
		scaleDirection = new HashMap<>();
		scaleDirection.put("rightHipPitchLink", Scale_Direction.SCALE_Y);
		scaleDirection.put("leftHipPitchLink", Scale_Direction.SCALE_Z);
		scaleDirection.put("rightKneePitchLink", Scale_Direction.SCALE_Z);
		scaleDirection.put("leftKneePitchLink", Scale_Direction.SCALE_Z);		
	}
	
	public Scale_Direction getScaleDirection(String linkName) {
		if (scaleDirection.containsKey(linkName)) {
			return scaleDirection.get(linkName);
		} else {
			return null;
	    }
	}
}
