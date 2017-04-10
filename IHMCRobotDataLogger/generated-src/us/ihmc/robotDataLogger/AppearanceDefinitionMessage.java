package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "AppearanceDefinitionMessage" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class AppearanceDefinitionMessage implements IDLStruct<AppearanceDefinitionMessage>
{
    public AppearanceDefinitionMessage()
    {
        
        
    }
    @Override
    public void set(AppearanceDefinitionMessage other)
    {
        	r_ = other.r_;
        	g_ = other.g_;
        	b_ = other.b_;
        	transparency_ = other.transparency_;

    }

    public void setR(double r)
    {
        r_ = r;
    }

    public double getR()
    {
        return r_;
    }

        
    public void setG(double g)
    {
        g_ = g;
    }

    public double getG()
    {
        return g_;
    }

        
    public void setB(double b)
    {
        b_ = b;
    }

    public double getB()
    {
        return b_;
    }

        
    public void setTransparency(double transparency)
    {
        transparency_ = transparency;
    }

    public double getTransparency()
    {
        return transparency_;
    }

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(AppearanceDefinitionMessage data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(AppearanceDefinitionMessage data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	    current_alignment += 8 + CDR.alignment(current_alignment, 8);

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    cdr.write_type_6(r_);

	    cdr.write_type_6(g_);

	    cdr.write_type_6(b_);

	    cdr.write_type_6(transparency_);
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	r_ = cdr.read_type_6();	

	    	g_ = cdr.read_type_6();	

	    	b_ = cdr.read_type_6();	

	    	transparency_ = cdr.read_type_6();	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_6("r", r_);
			    
			    ser.write_type_6("g", g_);
			    
			    ser.write_type_6("b", b_);
			    
			    ser.write_type_6("transparency", transparency_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			r_ = ser.read_type_6("r");	
	    	    
	    			g_ = ser.read_type_6("g");	
	    	    
	    			b_ = ser.read_type_6("b");	
	    	    
	    			transparency_ = ser.read_type_6("transparency");	
	    	    
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof AppearanceDefinitionMessage)) return false;
        AppearanceDefinitionMessage otherMyClass = (AppearanceDefinitionMessage)other;
        boolean returnedValue = true;

        returnedValue &= this.r_ == otherMyClass.r_;

                
        returnedValue &= this.g_ == otherMyClass.g_;

                
        returnedValue &= this.b_ == otherMyClass.b_;

                
        returnedValue &= this.transparency_ == otherMyClass.transparency_;

                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("AppearanceDefinitionMessage {");
        builder.append("r=");
        builder.append(this.r_);

                builder.append(", ");
        builder.append("g=");
        builder.append(this.g_);

                builder.append(", ");
        builder.append("b=");
        builder.append(this.b_);

                builder.append(", ");
        builder.append("transparency=");
        builder.append(this.transparency_);

                
        builder.append("}");
		return builder.toString();
    }

    private double r_; 
    private double g_; 
    private double b_; 
    private double transparency_; 

}