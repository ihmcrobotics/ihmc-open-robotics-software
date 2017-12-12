package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "Summary" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class Summary
{
    public Summary()
    {
        	summaryTriggerVariable_ = new java.lang.StringBuilder(1024); 
        	summarizedVariables_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (128, "type_d");           
        
    }

    public void set(Summary other)
    {
        	createSummary_ = other.createSummary_;
        	summaryTriggerVariable_.setLength(0);
        	summaryTriggerVariable_.append(other.summaryTriggerVariable_);
            summarizedVariables_.set(other.summarizedVariables_);	
    }

    public void setCreateSummary(boolean createSummary)
    {
        createSummary_ = createSummary;
    }

    public boolean getCreateSummary()
    {
        return createSummary_;
    }

        
        public void setSummaryTriggerVariable(String summaryTriggerVariable)
        {
        	summaryTriggerVariable_.setLength(0);
        	summaryTriggerVariable_.append(summaryTriggerVariable);
        }
        
        public java.lang.String getSummaryTriggerVariableAsString()
        {
        	return getSummaryTriggerVariable().toString();
        }

    public java.lang.StringBuilder getSummaryTriggerVariable()
    {
        return summaryTriggerVariable_;
    }

        

    public us.ihmc.idl.IDLSequence.StringBuilderHolder  getSummarizedVariables()
    {
        return summarizedVariables_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof Summary)) return false;
        Summary otherMyClass = (Summary)other;
        boolean returnedValue = true;

        returnedValue &= this.createSummary_ == otherMyClass.createSummary_;

                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.summaryTriggerVariable_, otherMyClass.summaryTriggerVariable_);
                
        returnedValue &= this.summarizedVariables_.equals(otherMyClass.summarizedVariables_);
                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("Summary {");
        builder.append("createSummary=");
        builder.append(this.createSummary_);

                builder.append(", ");
        builder.append("summaryTriggerVariable=");
        builder.append(this.summaryTriggerVariable_);

                builder.append(", ");
        builder.append("summarizedVariables=");
        builder.append(this.summarizedVariables_);

                
        builder.append("}");
		return builder.toString();
    }

    private boolean createSummary_; 
    private java.lang.StringBuilder summaryTriggerVariable_; 
    private us.ihmc.idl.IDLSequence.StringBuilderHolder  summarizedVariables_; 

}