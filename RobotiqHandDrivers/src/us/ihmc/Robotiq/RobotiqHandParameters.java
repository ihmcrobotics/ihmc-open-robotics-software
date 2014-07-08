package us.ihmc.Robotiq;

public final class RobotiqHandParameters
{
	//Parameters for use with the Robotiq 3-Finger Gripper with ModbusTCP interface
	static final int PORT = 502;
	static final String RIGHT_HAND_ADDRESS = "192.168.1.13";
	static final String LEFT_HAND_ADDRESS = "192.168.1.12"; //placeholder
	static final byte UNIT_ID = 0x02;
	static final byte MAX_SPEED = (byte) 0xFF;
	static final byte MAX_FORCE = (byte) 0xFF;
	static final byte FULLY_OPEN = 0x00;
	static final byte MIN_SPEED = 0x00;
	static final byte MIN_FORCE = 0x00;
	static final byte FULLY_CLOSED = (byte)0xFF;
}
