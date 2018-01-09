package us.ihmc.robotDataLogger;
/**
* 
* Definition of the class "Video" defined in LogProperties.idl. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class Video
{
    public Video()
    {
        
        
    }

    public void set(Video other)
    {
        	hasTimebase_ = other.hasTimebase_;

    }

    public void setHasTimebase(boolean hasTimebase)
    {
        hasTimebase_ = hasTimebase;
    }

    public boolean getHasTimebase()
    {
        return hasTimebase_;
    }

        




    @Override
    public boolean equals(java.lang.Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof Video)) return false;
        Video otherMyClass = (Video)other;
        boolean returnedValue = true;

        returnedValue &= this.hasTimebase_ == otherMyClass.hasTimebase_;

                

        return returnedValue;
    }
    
     @Override
    public java.lang.String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("Video {");
        builder.append("hasTimebase=");
        builder.append(this.hasTimebase_);

                
        builder.append("}");
		return builder.toString();
    }

    private boolean hasTimebase_; 

}