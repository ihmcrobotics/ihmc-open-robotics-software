package us.ihmc.robotiq.data;

import java.util.Arrays;

import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandSensorData;
import us.ihmc.robotiq.RobotiqHandInterface;

public class RobotiqHandSensorData implements HandSensorData
{
	static final int FINGER_A = 0;
	static final int FINGER_B = 1;
	static final int FINGER_C = 2;
	static final int SCISSOR = 3;
	private static final int[] FINGERS = {FINGER_A, FINGER_B, FINGER_C};
	private static final int MAX_POSITION_VALUE = 0xFF;
	
	static final int IN_MOTION = 0;
	static final int STOPPED_SOME_SHORT = 1;
	static final int STOPPED_ALL_SHORT = 2;
	static final int STOPPED_AT_REQUESTED = 3;
	
	static final int STOPPED_SHORT_OPEN = 1;
	static final int STOPPED_SHORT_CLOSED = 2;

	static final int BASIC_MODE = 0;
	static final int PINCH_MODE = 1;
	static final int WIDE_MODE = 2;
	static final int SCISSOR_MODE = 3;
	
	static final int IN_RESET = 0;
	static final int ACTIVATING = 1;
	static final int CHANGING_MODE = 2;
	static final int CHANGE_COMPLETE = 3;
	
//	private static enum GripperStatus
//	{
//		IN_MOTION (0),
//		STOPPED_SOME_SHORT (1),
//		STOPPED_ALL_SHORT (2),
//		STOPPED_AT_REQUESTED (3);
//		
//		private int value;
//		GripperStatus(int value)
//		{
//			this.value = value;
//		}
//	}
//	
//	private static enum OperationMode
//	{
//		IN_MOTION (0),
//		STOPPED_SOME_SHORT (1),
//		STOPPED_ALL_SHORT (2),
//		STOPPED_AT_REQUESTED (3);
//		
//		private int value;
//		OperationMode(int value)
//		{
//			this.value = value;
//		}
//	}
	private boolean activated;
	private char operationMode;
	private boolean motionRequests;
	private char modeStatus;
	private char gripperStatus;
	private char[] objectDetection = new char[4];
	private byte error;
	private char[] position = new char[4];
	private char[] requestedPosition = new char[4];
	private char[] current = new char[4];
	
	public RobotiqHandSensorData(byte[] data)
	{
		if(data != null) update(data);
	}
	
	public void update(byte[] data)
	{
		activated = (data[0] == 0) ? false : true;
		operationMode = (char) data[1];
		motionRequests = (data[2] == 0) ? false : true;
		modeStatus = (char) data[3];
		gripperStatus = (char) data[4];
		int counter;
		for(counter = 0; counter < 4; counter++)
		{
			objectDetection[counter] = (char) data[5 + counter];
		}
		for(counter = 0; counter < 4; counter++)
		{
			position[counter] = (char) (0xFF & data[9 + counter]);
		}
		for(counter = 0; counter < 4; counter++)
		{
			requestedPosition[counter] = (char) (0xFF & data[13 + counter]);
		}
		for(counter = 0; counter < 4; counter++)
		{
			current[counter] = (char) (0xFF & data[17 + counter]);
		}
	}
	
	public boolean isActivated()
	{
		return activated;
	}
	
	public char getOperationMode()
	{
		return operationMode;
	}
	
	public boolean isRequestingMotion()
	{
		return motionRequests;
	}
	
	public char getModeStatus()
	{
		return modeStatus;
	}
	
	public char getGripperStatus()
	{
		return gripperStatus;
	}
	
	public boolean isInMotion()
	{
		return !(gripperStatus == IN_MOTION);
	}
	
	public boolean hasStoppedShort()
	{
		return !(gripperStatus == IN_MOTION) && !(gripperStatus == STOPPED_AT_REQUESTED);
	}
	
	public boolean hasError()
	{
		return error != 0x00;
	}
	
	public void printError()
	{
		switch(error)
		{
		case 0x00: System.err.println("No Error");
		case 0x05: System.err.println("Priority Fault: Action delayed, activation must be completed prior to action."); break;
		case 0x06: System.err.println("Priority Fault: Action delayed, mode change must be completed prior to action."); break;
		case 0x07: System.err.println("Priority Fault: The activation bit must be set prior to action."); break;
		case 0x09: System.err.println("Minor Fault: Communication is not ready (may be booting)."); break;
		case 0x0A: System.err.println("Minor Fault: Changing mode fault, interferences detected on scissor axis (less then 20s)."); break;
		case 0x0B: System.err.println("Minor Fault: Automatic release in progress."); break;
		case 0x0D: System.err.println("Major Fault: Action fault, verify that no interference or other error occurred."); break;
		case 0x0E: System.err.println("Major Fault: Changing mode fault, interferences detected on scissor axis (more then 20s)."); break;
		case 0x0F: System.err.println("Major Fault: Automatic release completed. Reset and activation required."); break;
		}
	}
	
	public char[] getFingerStatus()
	{
		return objectDetection;
	}
	
	public char[] getFingerCurrents()
	{
		return current;
	}
	
	public char[] getRequestedFingerPositions()
	{
		return requestedPosition;
	}
	
	public char[] getFingerPositions()
	{
		return null;
	}
	
	double[] temp = new double[2];
	public double[][] getFingerJointAngles()
	{
		double[][] fingerJointAngles = new double[4][];
		
		for(int finger : FINGERS)
		{
			temp[0] = (double)position[finger] * 62.5  / MAX_POSITION_VALUE;
			temp[1] = (double)position[finger] * 90.0 / MAX_POSITION_VALUE;
			fingerJointAngles[finger] = Arrays.copyOf(temp, 2);
		}
		temp[0] = (double)position[SCISSOR] * 32 / MAX_POSITION_VALUE;
		fingerJointAngles[SCISSOR] = Arrays.copyOf(temp, 1);
		return fingerJointAngles;
	}

}
