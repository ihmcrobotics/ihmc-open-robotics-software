// Copyright 2017 Florida Institute for Human And Machine Cognition (IHMC)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
package us.ihmc.idl.us.ihmc.robotDataLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.idl.CDR;
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
        	guid_ = new byte[16];
        	dataIP_ = new byte[4];
        	cameras_ = new IDLSequence.Byte (127, "type_9");
        	name_ = new StringBuilder(255); 
        
        
    }
    @Override
    public void set(Announcement other)
    {
        	for(int b = 0; b < guid_.length; ++b)
        	{
        	    	guid_[b] = other.guid_[b];	

        	}
        	for(int d = 0; d < dataIP_.length; ++d)
        	{
        	    	dataIP_[d] = other.dataIP_[d];	

        	}
        	dataPort_ = other.dataPort_;
        	cameras_.set(other.cameras_);name_.setLength(0);
        	name_.append(other.name_);
        	log_ = other.log_;

    }


    public byte[] getGuid()
    {
        return guid_;
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

        

    public IDLSequence.Byte  getCameras()
    {
        return cameras_;
    }

        

    public StringBuilder getName()
    {
        return name_;
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
	            
	    current_alignment += ((16) * 1) + CDR.alignment(current_alignment, 1);

	    current_alignment += ((4) * 1) + CDR.alignment(current_alignment, 1);

	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    current_alignment += (127 * 1) + CDR.alignment(current_alignment, 1);


	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

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
	            
	    current_alignment += ((16) * 1) + CDR.alignment(current_alignment, 1);
	    current_alignment += ((4) * 1) + CDR.alignment(current_alignment, 1);
	    current_alignment += 2 + CDR.alignment(current_alignment, 2);

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    current_alignment += (data.getCameras().size() * 1) + CDR.alignment(current_alignment, 1);


	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 1 + CDR.alignment(current_alignment, 1);

	
	    return current_alignment - initial_alignment;
	}
	
	@Override
	public final void serialize(CDR cdr)
	{


	    for(int a = 0; a < guid_.length; ++a)
	    {
	        	cdr.write_type_9(guid_[a]);	
	    }

	    for(int a = 0; a < dataIP_.length; ++a)
	    {
	        	cdr.write_type_9(dataIP_[a]);	
	    }

	    cdr.write_type_1(dataPort_);

	    if(cameras_.size() <= 127)
	    cdr.write_type_e(cameras_);else
	        throw new RuntimeException("cameras field exceeds the maximum length");

	    if(name_.length() <= 255)
	    cdr.write_type_d(name_);else
	        throw new RuntimeException("name field exceeds the maximum length");

	    cdr.write_type_7(log_);
	}
	
	@Override
	public final void deserialize(CDR cdr)
	{

	    	for(int a = 0; a < guid_.length; ++a)
	    	{
	    	    	guid_[a] = cdr.read_type_9();	
	    	}
	    	

	    	for(int a = 0; a < dataIP_.length; ++a)
	    	{
	    	    	dataIP_[a] = cdr.read_type_9();	
	    	}
	    	

	    	dataPort_ = cdr.read_type_1();	

	    	cdr.read_type_e(cameras_);	

	    	cdr.read_type_d(name_);	

	    	log_ = cdr.read_type_7();	
	}

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof Announcement)) return false;
        Announcement otherMyClass = (Announcement)other;
        boolean returnedValue = true;

                	for(int c = 0; c < guid_.length; ++c)
                	{
                	    returnedValue &= this.guid_[c] == otherMyClass.guid_[c];

                	}        
                	for(int e = 0; e < dataIP_.length; ++e)
                	{
                	    returnedValue &= this.dataIP_[e] == otherMyClass.dataIP_[e];

                	}        
        returnedValue &= this.dataPort_ == otherMyClass.dataPort_;

                
        returnedValue &= this.cameras_.equals(otherMyClass.cameras_);
                
        returnedValue &= us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_);
                
        returnedValue &= this.log_ == otherMyClass.log_;

                

        return returnedValue;
    }
    
     @Override
    public String toString()
    {
		StringBuilder builder = new StringBuilder();
		
      	builder.append("Announcement {");
        builder.append("guid=");
        builder.append(Arrays.toString(this.guid_));

                builder.append(", ");
        builder.append("dataIP=");
        builder.append(Arrays.toString(this.dataIP_));

                builder.append(", ");
        builder.append("dataPort=");
        builder.append(this.dataPort_);

                builder.append(", ");
        builder.append("cameras=");
        builder.append(this.cameras_);

                builder.append(", ");
        builder.append("name=");
        builder.append(this.name_);

                builder.append(", ");
        builder.append("log=");
        builder.append(this.log_);

                
        builder.append("}");
		return builder.toString();
    }

    private byte[] guid_; 
    private byte[] dataIP_; 
    private short dataPort_; 
    private IDLSequence.Byte  cameras_; 
    private StringBuilder name_; 
    private boolean log_; 

}