package us.ihmc.valkyrie.testsupport;

import java.io.IOException;
import javax.xml.parsers.ParserConfigurationException;

import org.xml.sax.SAXException;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties;

public class ModifiableValkyriePhysicalProperties extends ValkyriePhysicalProperties {
	
	private double modifiedShinLength, modifiedThighLength;
	private boolean valuesSet;
	
	public ModifiableValkyriePhysicalProperties(ModifiableValkyrieRobotConfig config)
	{
		super(config.getGlobalSizeScale(), config.getGlobalMassScale());

		if (config.getModelFile() != null) {
			SDFSimpleParser sdfContent = null;
    		try {
				sdfContent = new SDFSimpleParser(config.getModelFile());
			} catch (ParserConfigurationException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (SAXException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    		modifiedThighLength = sdfContent.getLinkLength("rightHipPitchLink");
    		modifiedShinLength  = sdfContent.getLinkLength("rightKneePitchLink");
    		System.out.printf("Modified Shin is %f  Modified Thigh is %f\n", modifiedShinLength, modifiedThighLength);
		} else {
    		modifiedThighLength = super.getThighLength();
	    	modifiedShinLength = super.getShinLength();
		}
	}
	
	private double getJointOffsetLength(GeneralizedSDFRobotModel model, String joint)
	{
		SDFJointHolder jointHolder = model.getJointHolder("rightKneePitch");
		Vector3D jointOffset = jointHolder.getOffsetFromParentJoint();
		return jointOffset.length();
		
	}
	
	@Override
	public double getShinLength()
	{
		return modifiedShinLength;
	} 
	
	@Override
	public double getThighLength()
	{
		return modifiedThighLength;
	}
	
	@Override
	public double getLegLength()
	{
		return getShinLength() + getThighLength();
	}
	
}
