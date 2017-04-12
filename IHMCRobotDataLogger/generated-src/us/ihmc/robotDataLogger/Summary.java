package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "Summary" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class Summary implements IDLStruct<Summary>
{
    public Summary()
    {
        	summaryTriggerVariable_ = new StringBuilder(1024); 
        	summarizedVariables_ = new IDLSequence.StringBuilderHolder (128, "type_d");           
        
    }
    @Override
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
        
        public String getSummaryTriggerVariableAsString()
        {
        	return getSummaryTriggerVariable().toString();
        }

    public StringBuilder getSummaryTriggerVariable()
    {
        return summaryTriggerVariable_;
    }

        

    public IDLSequence.StringBuilderHolder  getSummarizedVariables()
    {
        return summarizedVariables_;
    }

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 1024 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 128; ++a)
	    {
	        current_alignment += 4 + CDR.alignment(current_alignment, 4) + 1024 + 1;
	    }
	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(Summary data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(Summary data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getSummaryTriggerVariable().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getSummarizedVariables().size(); ++a)
	    {
	        current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getSummarizedVariables().get(a).length() + 1;
	    }
	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_7(createSummary_);

	    if(summaryTriggerVariable_.length() <= 1024)
	    cdr.write_type_d(summaryTriggerVariable_);else
	        throw new RuntimeException("summaryTriggerVariable field exceeds the maximum length");

	    if(summarizedVariables_.size() <= 128)
	    cdr.write_type_e(summarizedVariables_);else
	        throw new RuntimeException("summarizedVariables field exceeds the maximum length");
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	createSummary_ = cdr.read_type_7();	

	    	cdr.read_type_d(summaryTriggerVariable_);	

	    	cdr.read_type_e(summarizedVariables_);	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_7("createSummary", createSummary_);
			    
			    ser.write_type_d("summaryTriggerVariable", summaryTriggerVariable_);
			    
			    ser.write_type_e("summarizedVariables", summarizedVariables_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			createSummary_ = ser.read_type_7("createSummary");	
	    	    
	    			ser.read_type_d("summaryTriggerVariable", summaryTriggerVariable_);	
	    	    
	    			ser.read_type_e("summarizedVariables", summarizedVariables_);	
	    	    
	}

    @Override
    public boolean equals(Object other)
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
    public String toString()
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
    private StringBuilder summaryTriggerVariable_; 
    private IDLSequence.StringBuilderHolder  summarizedVariables_; 

}