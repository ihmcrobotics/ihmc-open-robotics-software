package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "Handshake" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class Handshake
{
    public Handshake()
    {
        	registries_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.YoRegistryDefinition> (1024, us.ihmc.robotDataLogger.YoRegistryDefinition.class, new us.ihmc.robotDataLogger.YoRegistryDefinitionPubSubType());

        	variables_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.YoVariableDefinition> (32767, us.ihmc.robotDataLogger.YoVariableDefinition.class, new us.ihmc.robotDataLogger.YoVariableDefinitionPubSubType());

        	joints_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.JointDefinition> (128, us.ihmc.robotDataLogger.JointDefinition.class, new us.ihmc.robotDataLogger.JointDefinitionPubSubType());

        	graphicObjects_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.GraphicObjectMessage> (2048, us.ihmc.robotDataLogger.GraphicObjectMessage.class, new us.ihmc.robotDataLogger.GraphicObjectMessagePubSubType());

        	artifacts_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.GraphicObjectMessage> (2048, us.ihmc.robotDataLogger.GraphicObjectMessage.class, new us.ihmc.robotDataLogger.GraphicObjectMessagePubSubType());

        	enumTypes_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.EnumType> (1024, us.ihmc.robotDataLogger.EnumType.class, new us.ihmc.robotDataLogger.EnumTypePubSubType());

        	summary_ = new us.ihmc.robotDataLogger.Summary();
        
        
    }

    public void set(Handshake other)
    {
        	dt_ = other.dt_;
            registries_.set(other.registries_);	variables_.set(other.variables_);	joints_.set(other.joints_);	graphicObjects_.set(other.graphicObjects_);	artifacts_.set(other.artifacts_);	enumTypes_.set(other.enumTypes_);	us.ihmc.robotDataLogger.SummaryPubSubType.staticCopy(other.summary_, summary_);
    }

    public void setDt(double dt)
    {
        dt_ = dt;
    }

    public double getDt()
    {
        return dt_;
    }

        

    public us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.YoRegistryDefinition>  getRegistries()
    {
        return registries_;
    }

        

    public us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.YoVariableDefinition>  getVariables()
    {
        return variables_;
    }

        

    public us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.JointDefinition>  getJoints()
    {
        return joints_;
    }

        

    public us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.GraphicObjectMessage>  getGraphicObjects()
    {
        return graphicObjects_;
    }

        

    public us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.GraphicObjectMessage>  getArtifacts()
    {
        return artifacts_;
    }

        

    public us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.EnumType>  getEnumTypes()
    {
        return enumTypes_;
    }

        

    public us.ihmc.robotDataLogger.Summary getSummary()
    {
        return summary_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
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
    public java.lang.String toString()
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
    private us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.YoRegistryDefinition>  registries_; 
    private us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.YoVariableDefinition>  variables_; 
    private us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.JointDefinition>  joints_; 
    private us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.GraphicObjectMessage>  graphicObjects_; 
    private us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.GraphicObjectMessage>  artifacts_; 
    private us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.EnumType>  enumTypes_; 
    private us.ihmc.robotDataLogger.Summary summary_; 

}