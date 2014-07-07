package us.ihmc.Robotiq;

public final class RobotiqHandParameters
{
	//Parameters for use with the Robotiq 3-Finger Gripper with ModbusTCP interface
	static final int PORT = 502;
	static final String RIGHT_HAND_ADDRESS = "192.168.1.13";
	static final String LEFT_HAND_ADDRESS = "192.168.1.12"; //not sure yet
	static final byte UNIT_ID = 0x02;
	static final byte SET_REGISTERS = 0x10;
	static final byte READ_REGISTERS = 0x04;
	static final int REGISTER_START = 0x0000;
	static final byte ACTIVATE_HAND = 0x01;
	static final byte COMMAND_HAND_POSITONS = 0b1000;
	static final byte IGNORE_POSITION_COMMANDS = 0b0000;
	static final byte NORMAL_MODE = 0b0000;
	static final byte WIDE_MODE = 0b0010;
	static final byte PINCH_MODE = 0b0100;
	static final byte SCISSOR_MODE = 0b0110;
	static final byte MODE_RESET_MASK = 0b1001;
	static final byte CONCURRENT_CURL = 0b0000;
	static final byte CONCURRENT_SCISSOR = 0b0000;
	static final byte INDIVIDUAL_CURL = 0b0100;
	static final byte INDIVIDUAL_SCISSOR = 0b1000;
	static final byte MAX_SPEED = (byte) 0xFF;
	static final byte MAX_FORCE = (byte) 0xFF;
	static final byte FULLY_OPEN = 0x00;
	static final byte MIN_SPEED = 0x00;
	static final byte MIN_FORCE = 0x00;
	static final byte FULLY_CLOSED = (byte)0xFF;
}
