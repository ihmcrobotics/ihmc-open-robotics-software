package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
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
        	registries_ = new IDLSequence.Object<us.ihmc.robotDataLogger.YoRegistryDefinition> (1024, us.ihmc.robotDataLogger.YoRegistryDefinition.class, new us.ihmc.robotDataLogger.YoRegistryDefinitionPubSubType());

        	variables_ = new IDLSequence.Object<us.ihmc.robotDataLogger.YoVariableDefinition> (32767, us.ihmc.robotDataLogger.YoVariableDefinition.class, new us.ihmc.robotDataLogger.YoVariableDefinitionPubSubType());

        	joints_ = new IDLSequence.Object<us.ihmc.robotDataLogger.JointDefinition> (128, us.ihmc.robotDataLogger.JointDefinition.class, new us.ihmc.robotDataLogger.JointDefinitionPubSubType());

        	graphicObjects_ = new IDLSequence.Object<us.ihmc.robotDataLogger.GraphicObjectMessage> (2048, us.ihmc.robotDataLogger.GraphicObjectMessage.class, new us.ihmc.robotDataLogger.GraphicObjectMessagePubSubType());

        	artifacts_ = new IDLSequence.Object<us.ihmc.robotDataLogger.GraphicObjectMessage> (2048, us.ihmc.robotDataLogger.GraphicObjectMessage.class, new us.ihmc.robotDataLogger.GraphicObjectMessagePubSubType());

        	enumTypes_ = new IDLSequence.Object<us.ihmc.robotDataLogger.EnumType> (1024, us.ihmc.robotDataLogger.EnumType.class, new us.ihmc.robotDataLogger.EnumTypePubSubType());

        	summary_ = new us.ihmc.robotDataLogger.Summary();
        
        
    }
    @Override
    public void set(Handshake other)
    {
        	dt_ = other.dt_;
        	registries_.set(other.registries_);variables_.set(other.variables_);joints_.set(other.joints_);graphicObjects_.set(other.graphicObjects_);artifacts_.set(other.artifacts_);enumTypes_.set(other.enumTypes_);summary_.set(other.summary_);
    }

    public void setDt(double dt)
    {
        dt_ = dt;
    }

    public double getDt()
    {
        return dt_;
    }

        

    public IDLSequence.Object<us.ihmc.robotDataLogger.YoRegistryDefinition>  getRegistries()
    {
        return registries_;
    }

        

    public IDLSequence.Object<us.ihmc.robotDataLogger.YoVariableDefinition>  getVariables()
    {
        return variables_;
    }

        

    public IDLSequence.Object<us.ihmc.robotDataLogger.JointDefinition>  getJoints()
    {
        return joints_;
    }

        

    public IDLSequence.Object<us.ihmc.robotDataLogger.GraphicObjectMessage>  getGraphicObjects()
    {
        return graphicObjects_;
    }

        

    public IDLSequence.Object<us.ihmc.robotDataLogger.GraphicObjectMessage>  getArtifacts()
    {
        return artifacts_;
    }

        

    public IDLSequence.Object<us.ihmc.robotDataLogger.EnumType>  getEnumTypes()
    {
        return enumTypes_;
    }

        

    public us.ihmc.robotDataLogger.Summary getSummary()
    {
        return summary_;
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
	        current_alignment += us.ihmc.robotDataLogger.YoRegistryDefinition.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 32767; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.YoVariableDefinition.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 128; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.JointDefinition.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 2048; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.GraphicObjectMessage.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 2048; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.GraphicObjectMessage.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 1024; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.EnumType.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.Summary.getMaxCdrSerializedSize(current_alignment);
	
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
	    for(int a = 0; a < data.getRegistries().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.YoRegistryDefinition.getCdrSerializedSize(data.getRegistries().get(a), current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getVariables().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.YoVariableDefinition.getCdrSerializedSize(data.getVariables().get(a), current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getJoints().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.JointDefinition.getCdrSerializedSize(data.getJoints().get(a), current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getGraphicObjects().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.GraphicObjectMessage.getCdrSerializedSize(data.getGraphicObjects().get(a), current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getArtifacts().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.GraphicObjectMessage.getCdrSerializedSize(data.getArtifacts().get(a), current_alignment);}

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getEnumTypes().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.EnumType.getCdrSerializedSize(data.getEnumTypes().get(a), current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.Summary.getCdrSerializedSize(data.getSummary(), current_alignment);
	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_6(dt_);

	    if(registries_.size() <= 1024)
	    cdr.write_type_e(registries_);else
	        throw new RuntimeException("registries field exceeds the maximum length");

	    if(variables_.size() <= 32767)
	    cdr.write_type_e(variables_);else
	        throw new RuntimeException("variables field exceeds the maximum length");

	    if(joints_.size() <= 128)
	    cdr.write_type_e(joints_);else
	        throw new RuntimeException("joints field exceeds the maximum length");

	    if(graphicObjects_.size() <= 2048)
	    cdr.write_type_e(graphicObjects_);else
	        throw new RuntimeException("graphicObjects field exceeds the maximum length");

	    if(artifacts_.size() <= 2048)
	    cdr.write_type_e(artifacts_);else
	        throw new RuntimeException("artifacts field exceeds the maximum length");

	    if(enumTypes_.size() <= 1024)
	    cdr.write_type_e(enumTypes_);else
	        throw new RuntimeException("enumTypes field exceeds the maximum length");

	    cdr.write_type_a(summary_);
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	dt_ = cdr.read_type_6();	

	    	cdr.read_type_e(registries_);	

	    	cdr.read_type_e(variables_);	

	    	cdr.read_type_e(joints_);	

	    	cdr.read_type_e(graphicObjects_);	

	    	cdr.read_type_e(artifacts_);	

	    	cdr.read_type_e(enumTypes_);	

	    	cdr.read_type_a(summary_);	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_6("dt", dt_);
			    
			    ser.write_type_e("registries", registries_);
			    
			    ser.write_type_e("variables", variables_);
			    
			    ser.write_type_e("joints", joints_);
			    
			    ser.write_type_e("graphicObjects", graphicObjects_);
			    
			    ser.write_type_e("artifacts", artifacts_);
			    
			    ser.write_type_e("enumTypes", enumTypes_);
			    
			    ser.write_type_a("summary", summary_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			dt_ = ser.read_type_6("dt");	
	    	    
	    			ser.read_type_e("registries", registries_);	
	    	    
	    			ser.read_type_e("variables", variables_);	
	    	    
	    			ser.read_type_e("joints", joints_);	
	    	    
	    			ser.read_type_e("graphicObjects", graphicObjects_);	
	    	    
	    			ser.read_type_e("artifacts", artifacts_);	
	    	    
	    			ser.read_type_e("enumTypes", enumTypes_);	
	    	    
	    			ser.read_type_a("summary", summary_);	
	    	    
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

                
        returnedValue &= this.registries_.equals(otherMyClass.registries_);
                
        returnedValue &= this.variables_.equals(otherMyClass.variables_);
                
        returnedValue &= this.joints_.equals(otherMyClass.joints_);
                
        returnedValue &= this.graphicObjects_.equals(otherMyClass.graphicObjects_);
                
        returnedValue &= this.artifacts_.equals(otherMyClass.artifacts_);
                
        returnedValue &= this.enumTypes_.equals(otherMyClass.enumTypes_);
                
        returnedValue &= this.summary_.equals(otherMyClass.summary_);
                

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
        builder.append("registries=");
        builder.append(this.registries_);

                builder.append(", ");
        builder.append("variables=");
        builder.append(this.variables_);

                builder.append(", ");
        builder.append("joints=");
        builder.append(this.joints_);

                builder.append(", ");
        builder.append("graphicObjects=");
        builder.append(this.graphicObjects_);

                builder.append(", ");
        builder.append("artifacts=");
        builder.append(this.artifacts_);

                builder.append(", ");
        builder.append("enumTypes=");
        builder.append(this.enumTypes_);

                builder.append(", ");
        builder.append("summary=");
        builder.append(this.summary_);

                
        builder.append("}");
		return builder.toString();
    }

    private double dt_; 
    private IDLSequence.Object<us.ihmc.robotDataLogger.YoRegistryDefinition>  registries_; 
    private IDLSequence.Object<us.ihmc.robotDataLogger.YoVariableDefinition>  variables_; 
    private IDLSequence.Object<us.ihmc.robotDataLogger.JointDefinition>  joints_; 
    private IDLSequence.Object<us.ihmc.robotDataLogger.GraphicObjectMessage>  graphicObjects_; 
    private IDLSequence.Object<us.ihmc.robotDataLogger.GraphicObjectMessage>  artifacts_; 
    private IDLSequence.Object<us.ihmc.robotDataLogger.EnumType>  enumTypes_; 
    private us.ihmc.robotDataLogger.Summary summary_; 

}