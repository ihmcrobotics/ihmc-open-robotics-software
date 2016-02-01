package us.ihmc.acsell.treadmill;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import us.ihmc.tools.inputDevices.joystick.Joystick;



public class TreadmillSerialManager {
	
	public static class Commands
	{
		public static final int START_BELT = 0xA1;
		public static final int STOP_BELT = 0xAA;
		public static final int SET_SPEED = 0xA3;
		public static final int GET_SPEED = 0xC1;
	}
	public static class Acknowledgements
	{
		public static final int START_BELT = 0xB1;
		public static final int STOP_BELT = 0xBA;
		public static final int SET_SPEED = 0xB3;
		public static final int GET_SPEED = 0xD1;
	}

	private boolean isConnected = false;
	public boolean isConnected() {return isConnected;}
	private OutputStream outputStream;
	public OutputStream getSerialOutputStream() {return outputStream;}
	
	public TreadmillSerialManager( String portName )
	{
		try {
			connect(portName);

		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
    OutputStream connect ( String portName ) throws Exception
    {
    	OutputStream out = null;
        CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
        if ( portIdentifier.isCurrentlyOwned() )
        {
            System.out.println("Error: Port is currently in use");
        }
        else
        {
            CommPort commPort = portIdentifier.open(this.getClass().getName(),2000);
            
            if ( commPort instanceof SerialPort )
            {
                SerialPort serialPort = (SerialPort) commPort;
                serialPort.setSerialPortParams(4800,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
                
                InputStream in = serialPort.getInputStream();
                outputStream = serialPort.getOutputStream();
                
                (new Thread(new SerialReader(in,outputStream))).start();
                isConnected = true;
                //(new Thread(new SerialWriter(out, in))).start();
            }
            else
            {
            	isConnected = false;
                System.out.println("Error: Only serial ports are used by the treadmill.");
            }
        }
        return out;
    }
	
    static void listPorts()
    {
        java.util.Enumeration<CommPortIdentifier> portEnum = CommPortIdentifier.getPortIdentifiers();
        while ( portEnum.hasMoreElements() ) 
        {
            CommPortIdentifier portIdentifier = portEnum.nextElement();
            System.out.println(portIdentifier.getName()  +  " - " +  getPortTypeName(portIdentifier.getPortType()) );
        }        
    }
    
    static String getPortTypeName ( int portType )
    {
        switch ( portType )
        {
            case CommPortIdentifier.PORT_I2C:
                return "I2C";
            case CommPortIdentifier.PORT_PARALLEL:
                return "Parallel";
            case CommPortIdentifier.PORT_RAW:
                return "Raw";
            case CommPortIdentifier.PORT_RS485:
                return "RS485";
            case CommPortIdentifier.PORT_SERIAL:
                return "Serial";
            default:
                return "unknown type";
        }
    }
    
    
    
    public static void main(String[] args)
    {       
    	listPorts();
    	System.out.println("Connecting to Treadmill");
    	TreadmillSerialManager tsm = new TreadmillSerialManager("/dev/ttyS0");      
        setupJoyStick(tsm.getSerialOutputStream());

    }

   public static void setupJoyStick(final OutputStream out)
   {
      Joystick joystickUpdater;
      try
      {
         joystickUpdater = new Joystick();
         joystickUpdater.addJoystickEventListener(new TreadmillJoystickEventListener(out));
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

   }
    
    
    /** */
    public static class SerialReader implements Runnable 
    {
        final InputStream in;
        final OutputStream out;
        final boolean debug = false;
        
        public SerialReader ( InputStream in, OutputStream out )
        {
            this.in = in;
            this.out = out;
        }
        
        public void run ()
        {
            byte[] buffer = new byte[1024];
            int len = -1;

            try
            {
                while ( ( len = this.in.read(buffer)) > -1 )
                {
                	processResponse(buffer,len);
                }
            }
            catch ( IOException e )
            {
                e.printStackTrace();
            }            
        }
        
        final protected static char[] hexArray = "0123456789ABCDEF".toCharArray();
        public static String bytesToHex(byte[] bytes, int len) {
            char[] hexChars = new char[len*2];
            for ( int j = 0; j < len; j++ ) {
                int v = bytes[j] & 0xFF;
                hexChars[j * 2] = hexArray[v >>> 4];
                hexChars[j * 2 + 1] = hexArray[v & 0x0F];
            }
            return new String(hexChars);
        }
        
        private static int getResponseCode(byte[] bytes, int len)
        {
        	return bytes[0] & 0xFF;        	
        }
        
        private void processResponse(byte[] bytes, int len) throws IOException
        {
        	if(debug) System.out.print(bytesToHex(bytes,len) + ": ");	
        	switch (getResponseCode(bytes,len))
        	{
        		case Acknowledgements.START_BELT:
        			if(debug) System.out.println("Acknowledged Start Request");
        			break;
        		case Acknowledgements.STOP_BELT:
        			if(debug) System.out.println("Acknowledged Stop Request");
        			break;
        		case Acknowledgements.SET_SPEED:
        			out.write(Commands.GET_SPEED);
        			if(debug) System.out.println("Acknowledged Set Speed Request");        			
        			break;
            	case Acknowledgements.GET_SPEED:
        			if(debug) System.out.println("Acknowledged Get Speed Request");
        			break;
        		default:
        			if(debug) System.out.println("Current Speed = " + processSpeed(bytes, len));
        			
        	}
        }
        
        private int processSpeed(byte[] bytes, int len)
        {
        	int[] powersOfTen = {1000,100,10,1};
        	int speed = 0;
        	System.out.println(len);
        	if(len==4)
                for ( int j = 0; j < len; j++ ) {
                    speed += (bytes[j] & 0x0F)*powersOfTen[j];
                }
        	return speed;
        }
    }
    
}
