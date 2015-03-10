package us.ihmc.robotiq.data;

import java.util.Arrays;

import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandSensorData;

public class RobotiqHandSensorData implements HandSensorData
{
	static final int FINGER_A = 0;
	static final int FINGER_B = 1;
	static final int FINGER_C = 2;
	static final int SCISSOR = 3;
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
	private int operationMode;
	private boolean motionRequests;
	private byte modeStatus;
	private byte gripperStatus;
	private int[] objectDetection = new int[4];
	private byte error;
	private int[] position = new int[4];
	private int[] requestedPosition = new int[4];
	private int[] current = new int[4];
	
	public void update(byte[] data)
	{
		activated = (data[0] == 0) ? false : true;
		operationMode = (int) data[1];
		motionRequests = (data[2] == 0) ? false : true;
		modeStatus = (byte) data[3];
		gripperStatus = (byte) data[4];
		int counter;
		for(counter = 0; counter < 4; counter++)
		{
			objectDetection[counter] = (int) data[5 + counter];
		}
		for(counter = 0; counter < 4; counter++)
		{
			position[counter] = (int) (0xFF & data[11 + counter * 3]);
		}
		for(counter = 0; counter < 4; counter++)
		{
			requestedPosition[counter] = (int) (0xFF & data[10 + counter * 3]);
		}
		for(counter = 0; counter < 4; counter++)
		{
			current[counter] = (int) (0xFF & data[12 + counter * 3]);
		}
	}
	
	public boolean isActivated()
	{
		return activated;
	}
	
	public int getOperationMode()
	{
		return operationMode;
	}
	
	public boolean isRequestingMotion()
	{
		return motionRequests;
	}
	
	public byte getModeStatus()
	{
		return modeStatus;
	}
	
	public byte getGripperStatus()
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
	
	public boolean objectDetected()
	{
		boolean ret = true;
		
		for(int i : objectDetection)
		{
			ret &= i == 1 || i == 2;
		}
		
		return ret;
	}
	
	public boolean isInitializing()
	{
		return (modeStatus == ACTIVATING);
	}
	
	public boolean hasCompletedAction()
	{
		return (modeStatus == CHANGE_COMPLETE);
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
	
	public int[] getFingerStatus()
	{
		return objectDetection;
	}
	
	public int[] getFingerCurrents()
	{
		return current;
	}
	
	public int[] getRequestedFingerPositions()
	{
		return requestedPosition;
	}
	
	public int[] getFingerPositions()
	{
		return position;
	}
	
	double[] temp = new double[4];
	public double[][] getFingerJointAngles()
	{
		double[][] fingerJointAngles = new double[3][];
		//TODO: verify SCISSOR sign
		temp[0] = ((4.0/45) - (double)position[SCISSOR] * (8.0/45) / MAX_POSITION_VALUE) * Math.PI; //32 degrees
		temp[1] = (double)position[FINGER_A] * (25.0/72) * Math.PI  / MAX_POSITION_VALUE; //62.5 degrees
		temp[2] = (double)position[FINGER_A] * (0.5) * Math.PI / MAX_POSITION_VALUE; //90 degrees
		temp[3] = 0.0;
		fingerJointAngles[0] = Arrays.copyOf(temp, 4);
		
		temp[0] = -((4.0/45) - (double)position[SCISSOR] * (8.0/45) / MAX_POSITION_VALUE) * Math.PI; //32 degrees
		temp[1] = (double)position[FINGER_B] * (25.0/72) * Math.PI  / MAX_POSITION_VALUE; //62.5 degrees
		temp[2] = (double)position[FINGER_B] * (0.5) * Math.PI / MAX_POSITION_VALUE; //90 degrees
		temp[3] = 0.0;
		fingerJointAngles[1] = Arrays.copyOf(temp, 4);
		
		temp[0] = (double)position[FINGER_C] * (25.0/72) * Math.PI  / MAX_POSITION_VALUE; //62.5 degrees
		temp[1] = (double)position[FINGER_C] * (0.5) * Math.PI / MAX_POSITION_VALUE; //90 degrees
		temp[2] = 0.0;
		fingerJointAngles[2] = Arrays.copyOf(temp, 3);
		
		return fingerJointAngles;
	}

}
