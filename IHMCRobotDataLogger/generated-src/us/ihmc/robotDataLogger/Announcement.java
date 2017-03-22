package us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.IDLStruct;
import java.util.Arrays;

/**
* 
* Definition of the class "Announcement" defined in Announcement.idl. 
*
* This file was automatically generated from Announcement.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Announcement.idl instead.
*
*/
public class Announcement implements IDLStruct<Announcement>
{
    public Announcement()
    {
        	identifier_ = new StringBuilder(255); 
        	dataIP_ = new byte[4];
        	name_ = new StringBuilder(255); 
        	cameras_ = new IDLSequence.Object<us.ihmc.robotDataLogger.CameraAnnouncement> (127, us.ihmc.robotDataLogger.CameraAnnouncement.class, new us.ihmc.robotDataLogger.CameraAnnouncementPubSubType());

        	modelFileDescription_ = new us.ihmc.robotDataLogger.ModelFileDescription();
        
        
    }
    @Override
    public void set(Announcement other)
    {
        	identifier_.setLength(0);
        	identifier_.append(other.identifier_);
        	for(int b = 0; b < dataIP_.length; ++b)
        	{
        	    	dataIP_[b] = other.dataIP_[b];	

        	}
        	dataPort_ = other.dataPort_;
        	name_.setLength(0);
        	name_.append(other.name_);
        	cameras_.set(other.cameras_);modelFileDescription_.set(other.modelFileDescription_);log_ = other.log_;

    }

        public void setIdentifier(String identifier)
        {
        	identifier_.setLength(0);
        	identifier_.append(identifier);
        }
        
        public String getIdentifierAsString()
        {
        	return getIdentifier().toString();
        }

    public StringBuilder getIdentifier()
    {
        return identifier_;
    }

        

    public byte[] getDataIP()
    {
        return dataIP_;
    }

        
    public void setDataPort(short dataPort)
    {
        dataPort_ = dataPort;
    }

    public short getDataPort()
    {
        return dataPort_;
    }

        
        public void setName(String name)
        {
        	name_.setLength(0);
        	name_.append(name);
        }
        
        public String getNameAsString()
        {
        	return getName().toString();
        }

    public StringBuilder getName()
    {
        return name_;
    }

        

    public IDLSequence.Object<us.ihmc.robotDataLogger.CameraAnnouncement>  getCameras()
    {
        return cameras_;
    }

        

    public us.ihmc.robotDataLogger.ModelFileDescription getModelFileDescription()
    {
        return modelFileDescription_;
    }

        
    public void setLog(boolean log)
    {
        log_ = log;
    }

    public boolean getLog()
    {
        return log_;
    }

        


	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += ((4) * 1) + CDR.alignment(current_alignment, 1);

	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 127; ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.CameraAnnouncement.getMaxCdrSerializedSize(current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.ModelFileDescription.getMaxCdrSerializedSize(current_alignment);
	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(Announcement data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(Announcement data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getIdentifier().length() + 1;

	    current_alignment += ((4) * 1) + CDR.alignment(current_alignment, 1);
	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getCameras().size(); ++a)
	    {
	        current_alignment += us.ihmc.robotDataLogger.CameraAnnouncement.getCdrSerializedSize(data.getCameras().get(a), current_alignment);}

	    current_alignment += us.ihmc.robotDataLogger.ModelFileDescription.getCdrSerializedSize(data.getModelFileDescription(), current_alignment);
	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    if(identifier_.length() <= 255)
	    cdr.write_type_d(identifier_);else
	        throw new RuntimeException("identifier field exceeds the maximum length");

	    for(int a = 0; a < dataIP_.length; ++a)
	    {
	        	cdr.write_type_9(dataIP_[a]);	
	    }

	    cdr.write_type_1(dataPort_);

	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");

	    if(cameras_.size() <= 127)
	    cdr.write_type_e(cameras_);else
	        throw new RuntimeException("cameras field exceeds the maximum length");

	    cdr.write_type_a(modelFileDescription_);

	    cdr.write_type_7(log_);
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	cdr.read_type_d(identifier_);	

	    	for(int a = 0; a < dataIP_.length; ++a)
	    	{
	    	    	dataIP_[a] = cdr.read_type_9();	
	    	}
	    	

	    	dataPort_ = cdr.read_type_1();	

	    	cdr.read_type_d(name_);	

	    	cdr.read_type_e(cameras_);	

	    	cdr.read_type_a(modelFileDescription_);	

	    	log_ = cdr.read_type_7();	
	}
	
	@Override
	public final void serialize(InterchangeSerializer ser)
	{
			    ser.write_type_d("identifier", identifier_);
			    
			    ser.write_type_f("dataIP", dataIP_);
			    
			    ser.write_type_1("dataPort", dataPort_);
			    
			    ser.write_type_d("name", name_);
			    
			    ser.write_type_e("cameras", cameras_);
			    
			    ser.write_type_a("modelFileDescription", modelFileDescription_);
			    
			    ser.write_type_7("log", log_);
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser)
	{
	    			ser.read_type_d("identifier", identifier_);	
	    	    
	    			ser.read_type_f("dataIP", dataIP_);	
	    	    
	    			dataPort_ = ser.read_type_1("dataPort");	
	    	    
	    			ser.read_type_d("name", name_);	
	    	    
	    			ser.read_type_e("cameras", cameras_);	
	    	    
	    			ser.read_type_a("modelFileDescription", modelFileDescription_);	
	    	    
	    			log_ = ser.read_type_7("log");	
	    	    
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof Announcement)) return false;
        Announcement otherMyClass = (Announcement)other;
        boolean returnedValue = true;

        returnedValue &= us.ihmc.idl.IDLTools.equals(this.identifier_, otherMyClass.identifier_);
                
                	for(int c = 0; c < dataIP_.length; ++c)
                	{
                	    returnedValue &= this.dataIP_[c] == otherMyClass.dataIP_[c];

                	}        
        returnedValue &= this.dataPort_ == otherMyClass.dataPort_;

                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= this.cameras_.equals(otherMyClass.cameras_);
                
        returnedValue &= this.modelFileDescription_.equals(otherMyClass.modelFileDescription_);
                
        returnedValue &= this.log_ == otherMyClass.log_;

                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("Announcement {");
        builder.append("identifier=");
        builder.append(this.identifier_);

                builder.append(", ");
        builder.append("dataIP=");
        builder.append(Arrays.toString(this.dataIP_));

                builder.append(", ");
        builder.append("dataPort=");
        builder.append(this.dataPort_);

                builder.append(", ");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("cameras=");
        builder.append(this.cameras_);

                builder.append(", ");
        builder.append("modelFileDescription=");
        builder.append(this.modelFileDescription_);

                builder.append(", ");
        builder.append("log=");
        builder.append(this.log_);

                
        builder.append("}");
		return builder.toString();
    }

    private StringBuilder identifier_; 
    private byte[] dataIP_; 
    private short dataPort_; 
    private StringBuilder name_; 
    private IDLSequence.Object<us.ihmc.robotDataLogger.CameraAnnouncement>  cameras_; 
    private us.ihmc.robotDataLogger.ModelFileDescription modelFileDescription_; 
    private boolean log_; 

}