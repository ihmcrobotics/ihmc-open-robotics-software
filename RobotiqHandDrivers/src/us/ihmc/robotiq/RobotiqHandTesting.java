package us.ihmc.robotiq;

import java.io.IOException;
import java.net.UnknownHostException;

import us.ihmc.robotiq.communication.ModbusTCPConnection;

public class RobotiqHandTesting
{
	private static final long WAIT_TIME_MS = 60000;
	
	public static void main(String[] args) throws Exception
	{
		//testing the robot hand interface
		RobotiqHandInterface rightHand = new RobotiqHandInterface();
		
		long startTime = System.currentTimeMillis();
		
		do
		{
			if(!rightHand.isReady())
			{
//				System.out.println("starting connection");
//				rightHand.connect();
//				System.out.println("connected");
						
				System.out.println("Sending Activation Request");
				rightHand.initialize();
				System.out.println("Activation Request Sent");
						
//				System.out.println("Resetting Hand");
//				rightHand.reset();
//				System.out.println("Hand Reset");
			}
			
			long timeDifference = System.currentTimeMillis() - startTime;
			
			if(timeDifference > WAIT_TIME_MS)
				break;
		}
		while (true);
		
		System.out.println("Shutting Down Hand");
		rightHand.shutdown();
		System.out.println("Hand Shut Down");
	}

	/**
	 * @param hand
	 * @throws InterruptedException
	 */
	private static void testCycles(RobotiqHandInterface hand, int cycles) throws InterruptedException
	{
		System.out.println();
		System.out.println("Starting test cycles: " + cycles + " cycles");
		int counter;
		for(counter = 0; counter < cycles; counter++)
		{
			System.out.println("Cycle: " + (counter + 1));
			System.out.println("Closing Hand");
			hand.close();
			System.out.println("Hand Closed");
			Thread.sleep(1000);
			
			System.out.println("Opening Hand");
			hand.open();
			System.out.println("Hand Open");
			Thread.sleep(500);
			
			System.out.println();
		}
		System.out.println("Close cycles completed: " + counter);
		System.out.println();
		for(counter = 0; counter < cycles; counter++)
		{
			System.out.println("Cycle: " + (counter + 1));
			System.out.println("Crushing with Hand");
			hand.crush();
			System.out.println("Hand Crushing");
			Thread.sleep(500);
			
			System.out.println("Opening Hand");
			hand.open();
			System.out.println("Hand Open");
			Thread.sleep(500);
			
			System.out.println();
		}
		System.out.println("Crush cycles completed: " + counter);
		System.out.println();
	}
	
	private static void testModbusTCP() throws InterruptedException, UnknownHostException, IOException
	{
		//testing of the underlying connection
		ModbusTCPConnection connection;
		System.out.println("starting connection");
		connection = new ModbusTCPConnection("192.168.1.13", 502);
		System.out.println("connected");
		Thread.sleep(5000);
//		byte[] activationRequest = {0x03,(byte)0xE8,0x00,0x03,0x06,0x01,0x00,0x00,0x00,0x00,0x00};
		byte[] activationRequest = {0x10, 0x00,0x00,0x00,0x03,0x06,0x01,0x00,0x00,0x00,0x00,0x00};
//		byte[] dataLiteral = {0x33, (byte)0x9A, 0x00, 0x00, 0x00, 0x0D, 0x02, 0x10, 0x03, (byte)0xE8, 0x00, 0x03, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
//		byte[] dataLiteral = {0x33, (byte)0x9A, 0x00, 0x00, 0x00, 0x0D, 0x02, 0x10, 0x00, 0x00, 0x00, 0x03, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
		
//		System.out.println("sending data literal");
//		connection.sendLiteral(dataLiteral, dataLiteral.length);
		System.out.println("Sending Activation Request");
		connection.transcieve(0x02, activationRequest);
		System.out.println("activation data");
		
		Thread.sleep(20000);
		
		byte[] resetRequest = {0x10, 0x00,0x00,0x00,0x03,0x06,0x00,0x00,0x00,0x00,0x00,0x00};
		System.out.println("Sending reset");
		connection.transcieve(0x02, resetRequest);
		System.out.println("Reset Sent");
		
		Thread.sleep(5000);
		
		System.out.println("closing socket");
		connection.close();
		System.out.println("socket closed");
	}
}
