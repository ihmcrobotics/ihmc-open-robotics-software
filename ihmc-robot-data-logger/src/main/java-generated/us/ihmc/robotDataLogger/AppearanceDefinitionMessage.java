package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "AppearanceDefinitionMessage" defined in Handshake.idl. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class AppearanceDefinitionMessage
{
    public AppearanceDefinitionMessage()
    {
        
        
    }

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

        




    @Override
    public boolean equals(java.lang.Object other)
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
    public java.lang.String toString()
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