package us.ihmc.robotiq.communication;

import java.io.BufferedInputStream;
import java.io.FilterInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Arrays;

import us.ihmc.robotiq.RobotiqHandParameters;
import us.ihmc.utilities.ThreadTools;

public class ModbusTCPConnection
{
	private static final int HEADER_LENGTH = 7; // header length in bytes
	
	private Socket connection;
	private OutputStream outStream;
	private FilterInputStream inStream;
	private byte[] outBuffer = new byte[32];
	private byte[] inBuffer = new byte[32];
	private int packetCounter;
	
	// TODO: this came for Stack Overflow
	public static String bytesToHexString(byte[] bytes)
	{
		char[] hexArray = "0123456789ABCDEF".toCharArray();
		char[] hexChars = new char[bytes.length * 2];
		for(int i = 0; i < bytes.length; i++)
		{
			int hex = bytes[i] & 0xFF;
			hexChars[i * 2] = hexArray[hex >>> 4];
			hexChars[i * 2 + 1] = hexArray[hex & 0x0F];
		}
		
		return new String(hexChars);
	}
	
	public ModbusTCPConnection(String ip_address, int port) throws UnknownHostException, IOException
	{
		connection = new Socket(ip_address,port);
		outStream = connection.getOutputStream();
		inStream = new BufferedInputStream(connection.getInputStream());
		connection.setTcpNoDelay(true);
		packetCounter = 0;
	}
	
	public ModbusTCPConnection(String ip_address) throws UnknownHostException, IOException
	{
		this(ip_address, 502);
	}
	
	public void transcieve(int unitID, byte[] data) throws IOException
	{
		transcieve((byte)unitID, data);
	}
	
	public byte[] transcieve(byte unitID, byte[] data) throws IOException
	{		
		/*
		 *Header:
		 *Transaction ID: 2 bytes
		 *Protocol Identifier: 2 bytes
		 *Packet Length: 2 bytes
		 *Unit ID: 1 byte
		 *
		 *Data:
		 *Function Code: 1 byte
		 *Address of first register: 2 bytes
		 *Byte Count: 1 byte
		 *Application Data: Up to 1449 Bytes
		*/
		packetCounter++;
		if(packetCounter > 0xFFFF)	//cycle packet counter to maintain a semblance of knowledge about how many packets were sent
			packetCounter = 0;
		
		byte[] packetLength = new byte[2];
		packetLength[0] = (byte)((1 + data.length) >> 8);	//bit shifting to align integer to byte stream
		packetLength[1] = (byte)(1 + data.length);
		
		byte[] transactionID = new byte[2];
		transactionID[0] = (byte)(packetCounter >> 8);		//bit shifting to align integer to byte stream
		transactionID[1] = (byte)packetCounter;
		
		outBuffer[0] = transactionID[0];
		outBuffer[1] = transactionID[1];	//transactionID being used as packet counter
		outBuffer[2] = 0x00;
		outBuffer[3] = 0x00;				//Defining protocol as Modbus (0x0000)
		outBuffer[4] = packetLength[0];
		outBuffer[5] = packetLength[1];		//send 
		outBuffer[6] = unitID;
		
		for(int counter = 0; counter < data.length; counter++)
		{
			outBuffer[HEADER_LENGTH + counter] = data[counter];
		}
		
		int outBytes = HEADER_LENGTH + data.length; 
		outStream.write(outBuffer, 0, outBytes); //request
		outStream.flush();
		
		int inBytes = inStream.read(inBuffer, 0, 32); //reply
		
//		if(inBytes < 9)
//		{
//			if(inBytes == 0)
//			{
//				//unexpected close of connection
//				throw new ModbusException("Connection appears to have closed unexpectedly");
//			}
//			else
//			{
//				//response too short
//				throw new ModbusResponseTooShortException(unitID);
//			}
//		}
		
		System.out.println(bytesToHexString(outBuffer));
		System.out.println(bytesToHexString(inBuffer));
		
		return Arrays.copyOfRange(inBuffer, HEADER_LENGTH, inBytes); //return the reply with the proper length (removes header)
		
	}
	
	public byte[] sendLiteral(byte[] data, int length) throws IOException
	{
//		for(byte b : data)
//		{
//			b = (byte)(b & 0xFF);
//		}
		
		outStream.write(data, 0, length); //request
		outStream.flush();
		inStream.read(inBuffer, 0, 32); //reply
		return inBuffer;
	}
	
	public void close() throws IOException
	{
		connection.close();
	}
	
	public boolean testConnection()
	{
		try
		{
			outStream.write('\n');
			outStream.flush();
			inStream.read();
			return true;
		}
		catch(IOException e)
		{
			return false;
		}
	}
	
	public class ModbusException extends IOException
	{
		ModbusException()
		{
			super("Error with Modbus Connection at " + connection.getLocalSocketAddress());
		}
		ModbusException(String message)
		{
			super(message);
		}
	}
	
	public class ModbusResponseTooShortException extends ModbusException
	{
		public ModbusResponseTooShortException()
		{
			super("Response from a device on " + connection.getLocalSocketAddress() + " was too short.");
		}
		
		public ModbusResponseTooShortException(byte unitID)
		{
			super("Response from a device on " + connection.getLocalSocketAddress() + " ID: " + unitID + " was too short.");
		}
		
		public ModbusResponseTooShortException(String message)
		{
			super(message);
		}
	}
}
