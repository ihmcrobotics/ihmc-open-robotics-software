package us.ihmc.robotiq.communication;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;
import java.util.Arrays;

public class ModbusTCPConnection
{
	private static final int HEADER_LENGTH = 7; // header length in bytes
	
	private final Object lock = new Object();
	
	private Socket connection;
	private OutputStream outStream;
	private InputStream inStream;
	private byte[] outBuffer = new byte[32];
	private byte[] inBuffer = new byte[32];
	private int packetCounter;
	
	public ModbusTCPConnection(String ipAddress) throws UnknownHostException, IOException
	{
		this(ipAddress, 502);
	}
	
	public ModbusTCPConnection(String ipAddress, int port) throws UnknownHostException, IOException
	{
		setupConnectionFields(ipAddress, port);
		
		packetCounter = 0;
	}
	
	public void setupConnectionFields(String ipAddress, int port) throws IOException
	{
		connection = new Socket();
		connection.connect(new InetSocketAddress(ipAddress, port), 200);
		outStream = connection.getOutputStream();
		inStream = connection.getInputStream();
		connection.setSoTimeout(1000);
	}
	
	public byte[] transcieve(byte unitID, byte[] data) throws IOException
	{
		synchronized(lock)
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
			outStream.write(outBuffer, 0, outBytes);

			int inBytes = 0;
			int readAttempts = 0;
			while(inBytes < HEADER_LENGTH && readAttempts < 5)
			{
				int bytesToRead = inBuffer.length - inBytes;
				inBytes += inStream.read(inBuffer, inBytes, bytesToRead);
				readAttempts++;
			}
			
//			int inBytes = inStream.read(inBuffer, 0, 32);
			
			return Arrays.copyOfRange(inBuffer, HEADER_LENGTH, inBytes); //return the reply with the proper length (removes header)
		}
	}
	
	public byte[] sendLiteral(byte[] data, int length) throws IOException
	{
		synchronized (lock)
		{
			outStream.write(data, 0, length);
			outStream.flush();
			inStream.read(inBuffer, 0, 32);
			
			return inBuffer;
		}
	}
	
	public void close() throws IOException
	{
		connection.close();
	}
	
	public boolean testConnection()
	{
		byte[] ping = new byte[]
				{
				// DATA
				// Function code
				0x04,
				// Address of first register to write to
				0x00, 0x00,
				// Number of registers to read from
				0x00, 0x08
				};
		
		synchronized (lock)
		{
			try
			{
				transcieve((byte)0x02, ping);
			}
			catch (SocketTimeoutException e)
			{
				return false;
			}
			catch (IOException e)
			{
				return false;
			}
		}
		
		return true;
	}
}
