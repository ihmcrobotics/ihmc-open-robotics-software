package us.ihmc.Robotiq;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Arrays;

public class RobotiqHandInterface
{
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
	
	public void initialize()
	{
		if(initializedStatus == RobotiqHandParameters.ACTIVATE_HAND)
		{
			this.reset();
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
		
		position[0] = RobotiqHandParameters.FULLY_CLOSED;
		speed[0] = RobotiqHandParameters.MAX_SPEED/2;
		force[0] = RobotiqHandParameters.MAX_FORCE/2;
		
		initializedStatus = RobotiqHandParameters.ACTIVATE_HAND;
		operationMode = RobotiqHandParameters.NORMAL_MODE;
		commandedStatus = RobotiqHandParameters.IGNORE_POSITION_COMMANDS;
		curlStatus = RobotiqHandParameters.CONCURRENT_CURL;
		scissorStatus = RobotiqHandParameters.CONCURRENT_SCISSOR;
		
		sendRequest(RobotiqHandParameters.SET_REGISTERS,
					RobotiqHandParameters.REGISTER_START,
					(byte)(initializedStatus | operationMode | commandedStatus),
					(byte)(curlStatus | scissorStatus),
					(byte)0x00,	//reserved byte
					(byte)position[0],
					(byte)speed[0],
					(byte)force[0]);
		//TODO:check until fully activated, then set to commanded positions
	}
	
	public void reset()
	{
		initializedStatus &= ~RobotiqHandParameters.ACTIVATE_HAND; //reset activation status
		commandedStatus = RobotiqHandParameters.IGNORE_POSITION_COMMANDS;
		sendRequest(RobotiqHandParameters.SET_REGISTERS,
				RobotiqHandParameters.REGISTER_START,
				(byte)(initializedStatus | operationMode | commandedStatus));
		//TODO:check until reset
		initialize();
	}
	
	private void sendRequest(byte functionCode, int startRegister, byte ... dataRegisters)
	{
		int dataLength = 0;
		data[0] = (byte)(startRegister >> 8);
		data[1] = (byte)startRegister;
		if(functionCode == RobotiqHandParameters.READ_REGISTERS)
		{
			data[2] = (byte)(dataRegisters[0] >> 8);
			data[3] = (byte)dataRegisters[0];
			dataLength = 4;
		}
		else
		{
			data[2] = (byte)(dataRegisters.length * 2);
			int counter;
			for(counter = 0; counter < dataRegisters.length; counter++)
			{
				data[counter+3] = dataRegisters[counter];
			}
			data[counter+3] = (byte)(dataRegisters.length >> 8);
			data[counter+3+1] = (byte)dataRegisters.length;
			dataLength = counter+3+1+1;
		}
		try {
			connection.transcieve(RobotiqHandParameters.UNIT_ID, functionCode, Arrays.copyOfRange(data, 0, dataLength));
		} catch (IOException e) {
			System.err.println("Unable to send Modbus request");
			e.printStackTrace();
		}
	}

	private void sendMotionRequest() {
		byte[] request = new byte[16];
		int dataLength = 0;
		int startAddress;
		if(operationMode != RobotiqHandParameters.SCISSOR_MODE)
		{
			startAddress = 0x0003; //register 3
			//update finger a
			request[0] = position[0];
			request[1] = speed[0];
			request[2] = force[0];
			dataLength += 3;
			if(curlStatus == RobotiqHandParameters.INDIVIDUAL_CURL)
			{
				//update finger b
				request[3] = position[1];
				request[4] = speed[1];
				request[5] = force[1];
				//update finger c
				request[6] = position[2];
				request[7] = speed[2];
				request[8] = force[2];
				dataLength += 6;
			}
		}
		else
		{
			startAddress = 0x000C; //register 12
			request[0] = position[3];
			request[1] = speed[3];
			request[2] = force[3];
			dataLength += 3;
		}
		sendRequest(RobotiqHandParameters.SET_REGISTERS, startAddress, Arrays.copyOfRange(request,0,dataLength));
	}
	
	public void open()
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
	}
	public void close()
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
	}
	
	public void crush()
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
}

