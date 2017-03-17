package us.ihmc.idl.us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "Handshake" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class Handshake implements IDLStruct<Handshake>
{
    public Handshake()
    {
        	registry_ = new IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.YoRegistryDefinition> (1024, us.ihmc.idl.us.ihmc.robotDataLogger.YoRegistryDefinition.class, new us.ihmc.idl.us.ihmc.robotDataLogger.YoRegistryDefinitionPubSubType());

        	variable_ = new IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.YoVariableDefinition> (32767, us.ihmc.idl.us.ihmc.robotDataLogger.YoVariableDefinition.class, new us.ihmc.idl.us.ihmc.robotDataLogger.YoVariableDefinitionPubSubType());

        	joint_ = new IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.JointDefinition> (255, us.ihmc.idl.us.ihmc.robotDataLogger.JointDefinition.class, new us.ihmc.idl.us.ihmc.robotDataLogger.JointDefinitionPubSubType());

        	graphic_object_ = new IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessage> (4096, us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessage.class, new us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessagePubSubType());

        	artifact_ = new IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessage> (4096, us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessage.class, new us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessagePubSubType());


        
        
    }
    @Override
    public void set(Handshake other)
    {
        	dt_ = other.dt_;
        	registry_.set(other.registry_);variable_.set(other.variable_);joint_.set(other.joint_);graphic_object_.set(other.graphic_object_);artifact_.set(other.artifact_);
    }

    public void setDt(double dt)
    {
        dt_ = dt;
    }

    public double getDt()
    {
        return dt_;
    }

        

    public IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.YoRegistryDefinition>  getRegistry()
    {
        return registry_;
    }

        

    public IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.YoVariableDefinition>  getVariable()
    {
        return variable_;
    }

        

    public IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.JointDefinition>  getJoint()
    {
        return joint_;
    }

        

    public IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessage>  getGraphic_object()
    {
        return graphic_object_;
    }

        

    public IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessage>  getArtifact()
    {
        return artifact_;
    }

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 1024; ++a)
	    {
	        current_alignment += us.ihmc.idl.us.ihmc.robotDataLogger.YoRegistryDefinition.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 32767; ++a)
	    {
	        current_alignment += us.ihmc.idl.us.ihmc.robotDataLogger.YoVariableDefinition.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 255; ++a)
	    {
	        current_alignment += us.ihmc.idl.us.ihmc.robotDataLogger.JointDefinition.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 4096; ++a)
	    {
	        current_alignment += us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessage.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 4096; ++a)
	    {
	        current_alignment += us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessage.getMaxCdrSerializedSize(current_alignment);}

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(Handshake data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(Handshake data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getRegistry().size(); ++a)
	    {
	        current_alignment += us.ihmc.idl.us.ihmc.robotDataLogger.YoRegistryDefinition.getCdrSerializedSize(data.getRegistry().get(a), current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getVariable().size(); ++a)
	    {
	        current_alignment += us.ihmc.idl.us.ihmc.robotDataLogger.YoVariableDefinition.getCdrSerializedSize(data.getVariable().get(a), current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getJoint().size(); ++a)
	    {
	        current_alignment += us.ihmc.idl.us.ihmc.robotDataLogger.JointDefinition.getCdrSerializedSize(data.getJoint().get(a), current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getGraphic_object().size(); ++a)
	    {
	        current_alignment += us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessage.getCdrSerializedSize(data.getGraphic_object().get(a), current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getArtifact().size(); ++a)
	    {
	        current_alignment += us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessage.getCdrSerializedSize(data.getArtifact().get(a), current_alignment);}

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_6(dt_);

	    if(registry_.size() <= 1024)
	    cdr.write_type_e(registry_);else
	        throw new RuntimeException("registry field exceeds the maximum length");

	    if(variable_.size() <= 32767)
	    cdr.write_type_e(variable_);else
	        throw new RuntimeException("variable field exceeds the maximum length");

	    if(joint_.size() <= 255)
	    cdr.write_type_e(joint_);else
	        throw new RuntimeException("joint field exceeds the maximum length");

	    if(graphic_object_.size() <= 4096)
	    cdr.write_type_e(graphic_object_);else
	        throw new RuntimeException("graphic_object field exceeds the maximum length");

	    if(artifact_.size() <= 4096)
	    cdr.write_type_e(artifact_);else
	        throw new RuntimeException("artifact field exceeds the maximum length");
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	dt_ = cdr.read_type_6();	

	    	cdr.read_type_e(registry_);	

	    	cdr.read_type_e(variable_);	

	    	cdr.read_type_e(joint_);	

	    	cdr.read_type_e(graphic_object_);	

	    	cdr.read_type_e(artifact_);	
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof Handshake)) return false;
        Handshake otherMyClass = (Handshake)other;
        boolean returnedValue = true;

        returnedValue &= this.dt_ == otherMyClass.dt_;

                
        returnedValue &= this.registry_.equals(otherMyClass.registry_);
                
        returnedValue &= this.variable_.equals(otherMyClass.variable_);
                
        returnedValue &= this.joint_.equals(otherMyClass.joint_);
                
        returnedValue &= this.graphic_object_.equals(otherMyClass.graphic_object_);
                
        returnedValue &= this.artifact_.equals(otherMyClass.artifact_);
                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("Handshake {");
        builder.append("dt=");
        builder.append(this.dt_);

                builder.append(", ");
        builder.append("registry=");
        builder.append(this.registry_);

                builder.append(", ");
        builder.append("variable=");
        builder.append(this.variable_);

                builder.append(", ");
        builder.append("joint=");
        builder.append(this.joint_);

                builder.append(", ");
        builder.append("graphic_object=");
        builder.append(this.graphic_object_);

                builder.append(", ");
        builder.append("artifact=");
        builder.append(this.artifact_);

                
        builder.append("}");
		return builder.toString();
    }

    private double dt_; 
    private IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.YoRegistryDefinition>  registry_; 
    private IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.YoVariableDefinition>  variable_; 
    private IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.JointDefinition>  joint_; 
    private IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessage>  graphic_object_; 
    private IDLSequence.Object<us.ihmc.idl.us.ihmc.robotDataLogger.DynamicGraphicMessage>  artifact_; 

}