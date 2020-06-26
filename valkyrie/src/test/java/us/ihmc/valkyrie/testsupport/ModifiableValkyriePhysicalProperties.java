package us.ihmc.valkyrie.testsupport;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.Map;

import javax.xml.parsers.ParserConfigurationException;

import org.xml.sax.SAXException;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties;

public class ModifiableValkyriePhysicalProperties extends ValkyriePhysicalProperties {
	
	private double modifiedShinLength, modifiedThighLength;
	
	public ModifiableValkyriePhysicalProperties(ModifiableValkyrieRobotConfig config, SDFSimpleParser parser) {
		this(config);
		setValuesFromSDF(parser);
	}
	
	public ModifiableValkyriePhysicalProperties(ModifiableValkyrieRobotConfig config)
	{
		super(config.getGlobalSizeScale(), config.getGlobalMassScale());

   		modifiedThighLength = super.getThighLength();
    	modifiedShinLength = super.getShinLength();
    }	
	
	private void setValuesFromSDF(SDFSimpleParser parser) {
		modifiedShinLength = getModelSizeScale() * parser.getLinkLength("rightKneePitchLink");
		modifiedThighLength = getModelSizeScale() * parser.getLinkLength("rightHipPitchLink");
		System.out.printf("Modified Shin is %f  Modified Thigh is %f\n", modifiedShinLength, modifiedThighLength);
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
