package us.ihmc.Robotiq;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Arrays;

public class RobotiqHandInterface
{
//	private static final byte INITAILIZING = 0b00100000;
//	private static final byte INITAILIZED = 0b00110000;
//	private static final byte INIT_MODE_STATUS = 0b00110000;
	
	
	private ModbusTCPConnection connection;
	
	private byte initializedStatus;
	private byte operationMode;
	private byte commandedStatus;
	private byte curlStatus;
	private byte scissorStatus;
	private byte[] data = new byte[32];
	private byte[] response = new byte[32];
	private byte[] speed = new byte[4];
	private byte[] force = new byte[4];
	private byte[] position = new byte[4];
	private int faultCounter = 0;
	
	
	RobotiqHandInterface()
	{
		
		try
		{
			connection = new ModbusTCPConnection(RobotiqHandParameters.RIGHT_HAND_ADDRESS,
												 RobotiqHandParameters.PORT);
		} 
		catch (UnknownHostException e)
		{
			e.printStackTrace();
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}
	
	RobotiqHandInterface(String address)
	{
		try
		{
			connection = new ModbusTCPConnection(address,RobotiqHandParameters.PORT);
		}
		catch (UnknownHostException e)
		{
			e.printStackTrace();
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}
	
	public void initialize() throws InterruptedException
	{
		byte[] status;
		status = this.getStatus();
		
		if(((status[2] & RobotiqHandParameters.ACTIVATE_HAND) == RobotiqHandParameters.ACTIVATE_HAND) || (status[4] != 0x00))
		{
			this.reset();
			return;
		}
		
		for(int counter = 0; counter < 32; counter++)
		{
			data[counter] = 0;
			response[counter] = 0;
		}
		for(int counter = 0; counter < 4; counter++)
		{
			speed[counter] = 0;
			force[counter] = 0;
			position[counter] = 0;
		}
		
		position[0] = RobotiqHandParameters.FULLY_OPEN;
		speed[0] = (byte) (RobotiqHandParameters.MAX_SPEED & 0xF0);
		force[0] = RobotiqHandParameters.MAX_FORCE/2;
		
		initializedStatus = RobotiqHandParameters.ACTIVATE_HAND;
		operationMode = RobotiqHandParameters.NORMAL_MODE;
		commandedStatus = RobotiqHandParameters.IGNORE_POSITION_COMMANDS;
		curlStatus = RobotiqHandParameters.CONCURRENT_CURL;
		scissorStatus = RobotiqHandParameters.CONCURRENT_SCISSOR;
		
		do
		{
			sendRequest(RobotiqHandParameters.SET_REGISTERS,
					RobotiqHandParameters.REGISTER_START,
					(byte)(initializedStatus | operationMode | commandedStatus),
					(byte)(curlStatus | scissorStatus),
					(byte)0x00,	//reserved byte
					(byte)position[0], //all finger control
					(byte)speed[0],
					(byte)force[0]);
			
			do
			{
				Thread.sleep(1000);
				status = this.getStatus();
			}while(((status[2] & 0b00110000) == 0b00010000) && (status[4] == 0x00)); //check/wait while activation is in progress
			
			if(status[4] != 0x00) //checking for errors
			{
				faultCounter++;
				if(faultCounter < 3)
					this.reset();
				else
				{
					System.err.println("Unable to initalize hand");
					printErrors(status[4]);
					return;
				}
			}
			
		}while ((status[2] & 0b00110000) != 0b00110000); //check to see if activation was successful, else resend command	
		
		faultCounter = 0; //reset fault counter
	}

	/**
	 * @param status
	 */
	private void printErrors(byte status)
	{
		switch(status) //print correct fault
		{
		case 0x05: System.err.println("Priority Fault: Action delayed, activation must be completed prior to action."); break;
		case 0x06: System.err.println("Priority Fault: Action delayed, mode change must be completed prior to action."); break;
		case 0x07: System.err.println("Priority Fault: The activation bit must be set prior to action."); break;
		case 0x09: System.err.println("Minor Fault: Communication is not ready (may be booting)."); break;
		case 0x0A: System.err.println("Minor Fault: Changing mode fault, interferences detected on scissor axis (less then 20s)."); break;
		case 0x0B: System.err.println("Minor Fault: Automatic release in progress."); break;
		case 0x0D: System.err.println("Major Fault: Action fault, verify that no interference or other error occured."); break;
		case 0x0E: System.err.println("Major Fault: Changing mode fault, interferences detected on scissor axis (more then 20s)."); break;
		case 0x0F: System.err.println("Major Fault: Automatic release completed. Reset and activation required."); break;
		}
	}
	
	public void reset() throws InterruptedException
	{
		initializedStatus &= ~RobotiqHandParameters.ACTIVATE_HAND; //reset activation status
		commandedStatus = RobotiqHandParameters.IGNORE_POSITION_COMMANDS;
		byte[] status;
		do
		{
			sendRequest(RobotiqHandParameters.SET_REGISTERS,
					RobotiqHandParameters.REGISTER_START,
					(byte)(initializedStatus | operationMode | commandedStatus),
					(byte)(curlStatus | scissorStatus));
		
			Thread.sleep(100);
			status = this.getStatus();
		}while((status[2] & 0x01) != 0x00); //check until reset
		initialize();
	}
	
	/* WARNING:
	 * Since Modbus considers a register to be a 16-bit word instead of a byte like the Robotiq Hand does,
	 * there must be an even number of registers given in dataRegisters when writing, otherwise the last register
	 * will be truncated to maintain compatibility and register integrity
	 */
	private byte[] sendRequest(byte functionCode, int startRegister, byte ... dataRegisters)
	{
		int dataLength = 0;
		data[0] = functionCode;
		data[1] = (byte)(startRegister >> 8);
		data[2] = (byte)startRegister;
		if(functionCode == RobotiqHandParameters.READ_REGISTERS)
		{
			data[3] = (byte)(dataRegisters[0] >> 8);
			data[4] = (byte)(dataRegisters[0]);
			dataLength = 5;
		}
		else
		{
			data[3] = (byte)((dataRegisters.length/2) >> 8);
			data[4] = (byte)(dataRegisters.length/2);
			data[5] = (byte)(dataRegisters.length); //number of bytes to follow
			int counter;
			for(counter = 0; counter < dataRegisters.length; counter++)
			{
				data[counter+6] = dataRegisters[counter];
			}
			dataLength = counter+6;
		}
		try {
			return connection.transcieve(RobotiqHandParameters.UNIT_ID, Arrays.copyOfRange(data, 0, dataLength));
		} catch (IOException e) {
			System.err.println("Unable to send Modbus request");
			e.printStackTrace();
		}
		return null;
	}

	private void sendMotionRequest() {
		byte[] request = new byte[32];
		int dataLength;
		Arrays.fill(request, (byte)0x00);
		request[0] = (byte)(initializedStatus | operationMode | RobotiqHandParameters.COMMAND_HAND_POSITONS);
		request[1] = (byte)(curlStatus | scissorStatus);
		//update finger a/all fingers
		request[2] = 0x00; //reserved byte
		request[3] = position[0];
		request[4] = speed[0];
		request[5] = force[0];
		dataLength = 6;
		if(curlStatus == RobotiqHandParameters.INDIVIDUAL_CURL)
		{
			//update finger b
			request[6] = position[1];
			request[7] = speed[1];
			request[8] = force[1];
			//update finger c
			request[9] = position[2];
			request[10] = speed[2];
			request[11] = force[2];
			dataLength = 12;
		}		
		if(operationMode == RobotiqHandParameters.SCISSOR_MODE)
		{
			request[12] = force[2];
			request[13] = position[3];
			request[14] = speed[3];
			request[15] = force[3];
			dataLength = 16;
		}
		sendRequest(RobotiqHandParameters.SET_REGISTERS, RobotiqHandParameters.REGISTER_START, Arrays.copyOfRange(request,0,dataLength));
		
	}
	
	public void open() throws InterruptedException
	{
		if(operationMode != RobotiqHandParameters.SCISSOR_MODE)
		{
			position[0] = RobotiqHandParameters.FULLY_OPEN; // finger A/all fingers
			if(curlStatus == RobotiqHandParameters.INDIVIDUAL_CURL)
			{
				position[1] = RobotiqHandParameters.FULLY_OPEN; // finger B
				position[2] = RobotiqHandParameters.FULLY_OPEN; //finger C
			}
		}
		else
		{
			position[3] = RobotiqHandParameters.FULLY_OPEN;
		}
		sendMotionRequest();
		byte[] status;
		do
		{
			Thread.sleep(200);
			status = this.getStatus();
		}while((status[2] & 0b11000000) == 0b00000000);
	}
	public void close() throws InterruptedException
	{
		if(operationMode != RobotiqHandParameters.SCISSOR_MODE)
		{
			position[0] = RobotiqHandParameters.FULLY_CLOSED; // finger A/all fingers
			force[0] = RobotiqHandParameters.MAX_FORCE/2;
			if(curlStatus == RobotiqHandParameters.INDIVIDUAL_CURL)
			{
				position[1] = RobotiqHandParameters.FULLY_CLOSED; // finger B
				force[1] = RobotiqHandParameters.MAX_FORCE/2;
				position[2] = RobotiqHandParameters.FULLY_CLOSED; //finger C
				force[2] = RobotiqHandParameters.MAX_FORCE/2;
			}
		}
		else
		{
			position[3] = RobotiqHandParameters.FULLY_CLOSED;
			force[3] = RobotiqHandParameters.MAX_FORCE/2;
		}
		sendMotionRequest();
		byte[] status;
		do
		{
			Thread.sleep(200);
			status = this.getStatus();
		}while(((status[3] & 0b00000011) == 0b00000000) ||
			   ((status[3] & 0b00001100) == 0b00000000) ||
			   ((status[3] & 0b00110000) == 0b00000000));
	}
	
	public void crush() throws InterruptedException
	{
		if(operationMode != RobotiqHandParameters.SCISSOR_MODE)
		{
			position[0] = RobotiqHandParameters.FULLY_CLOSED; // finger A/all fingers
			force[0] = RobotiqHandParameters.MAX_FORCE;
			if(curlStatus == RobotiqHandParameters.INDIVIDUAL_CURL)
			{
				position[1] = RobotiqHandParameters.FULLY_CLOSED; // finger B
				force[1] = RobotiqHandParameters.MAX_FORCE;
				position[2] = RobotiqHandParameters.FULLY_CLOSED; //finger C
				force[2] = RobotiqHandParameters.MAX_FORCE;
			}
		}
		else
		{
			position[3] = RobotiqHandParameters.FULLY_CLOSED;
			force[3] = RobotiqHandParameters.MAX_FORCE;
		}
		sendMotionRequest();
		byte[] status;
		do
		{
			Thread.sleep(200);
			status = this.getStatus();
		}while(((status[3] & 0b00000011) == 0b00000000) ||
			   ((status[3] & 0b00001100) == 0b00000000) ||
			   ((status[3] & 0b00110000) == 0b00000000));
	}
	
	public void shutdown()
	{
		try
		{
			connection.close();
		} catch
		(IOException e)
		{
			e.printStackTrace();
		}
	}
	public byte[] getStatus()
	{
		return sendRequest(RobotiqHandParameters.READ_REGISTERS,
						   RobotiqHandParameters.REGISTER_START,
						   (byte)0x08);
	}
}

