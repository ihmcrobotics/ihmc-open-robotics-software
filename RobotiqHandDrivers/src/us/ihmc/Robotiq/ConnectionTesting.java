package us.ihmc.Robotiq;

import java.io.IOException;

public class ConnectionTesting
{
	public static void main(String[] args) throws IOException, InterruptedException
	{
		
//		ModbusTCPConnection connection;
//		
//		System.out.println("starting connection");
//		
//		connection = new ModbusTCPConnection("192.168.1.13", 0x02);
//		
//		System.out.println("connected");
//		
//		Thread.sleep(5000);
//		
////		byte[] activationRequest = {0x03,(byte)0xE8,0x00,0x03,0x06,0x01,0x00,0x00,0x00,0x00,0x00};
//		byte[] activationRequest = {0x00,0x00,0x00,0x03,0x06,0x01,0x00,0x00,0x00,0x00,0x00};
////		byte[] dataLiteral = {0x33, (byte)0x9A, 0x00, 0x00, 0x00, 0x0D, 0x02, 0x10, 0x03, (byte)0xE8, 0x00, 0x03, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
////		byte[] dataLiteral = {0x33, (byte)0x9A, 0x00, 0x00, 0x00, 0x0D, 0x02, 0x10, 0x00, 0x00, 0x00, 0x03, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
//		
////		System.out.println("sending data literal");
////		connection.sendLiteral(dataLiteral, dataLiteral.length);
//		System.out.println("Sending Activation Request");
//		connection.transcieve(0x02, 0x10, activationRequest);
//		System.out.println("activation data");
//		
//		Thread.sleep(20000);
//		
//		byte[] resetRequest = {0x00,0x00,0x00,0x03,0x06,0x00,0x00,0x00,0x00,0x00,0x00};
//		System.out.println("Sending reset");
//		connection.transcieve(0x02, 0x10, resetRequest);
//		System.out.println("Reset Sent");
//		
//		Thread.sleep(5000);
//		
//		System.out.println("closing socket");
//		connection.close();
//		System.out.println("socket closed");
		
		System.out.println("starting connection");
		RobotiqHandInterface rightHand = new RobotiqHandInterface();
		System.out.println("connected");
		Thread.sleep(3000);
		System.out.println("Sending Activation Request");
		rightHand.initialize();
		System.out.println("Activation Request Sent");
		Thread.sleep(17000);
		System.out.println("Resetting Hand");
		rightHand.reset();
		System.out.println("Hand Reset");
		Thread.sleep(20000);
		System.out.println("Closing Hand");
		rightHand.close();
		System.out.println("Hand Closed");
		Thread.sleep(7000);
		System.out.println("Shutting Down Hand");
		rightHand.shutdown();
		System.out.println("Hand Shut Down");
	}
}
