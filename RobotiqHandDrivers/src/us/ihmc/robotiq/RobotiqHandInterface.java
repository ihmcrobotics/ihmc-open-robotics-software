package us.ihmc.robotiq;

import java.io.IOException;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Arrays;

import us.ihmc.robotiq.communication.ModbusTCPConnection;
import us.ihmc.robotiq.communication.ModbusTCPConnection.ModbusException;
import us.ihmc.robotiq.communication.ModbusTCPConnection.ModbusResponseTooShortException;
import us.ihmc.robotiq.data.RobotiqHandSensorData;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.robotSide.RobotSide;

/* GENERAL INFO
 * (This information assumes right hand for convention)
 * Finger A = Thumb
 * Finger B = Index Finger
 * Finger C = Middle Finger
 * Scissor = Motion between Fingers B & C
 */

public final class RobotiqHandInterface
{	
	/*---LOCATION DATA---*/
	private static final int REGISTER_START = 0x0000;
	
	
	/*---FUNCTION CODES---*/
	private static final byte SET_REGISTERS = 0x10;
	private static final byte READ_REGISTERS = 0x04;
	
	
	/*---CONTROL PARAMETERS---*/
	public static final byte MAX_SPEED = (byte) 0xFF;
	public static final byte MAX_FORCE = (byte) 0xFF;
	public static final byte FULLY_OPEN = 0x00;
	public static final byte MIN_SPEED = 0x00;
	public static final byte MIN_FORCE = 0x00;
	public static final byte FULLY_CLOSED = (byte)0xFF;
	public static final byte DEFAULT_SPEED = (byte)(MAX_SPEED & 11000000);  //about 3/4 speed
	public static final byte SCISSOR_POSITION_BASIC = 120; //approximate scissor position during basic grip
	
	
	/*---DATA INDICES---*/
	//status response index (from hand start register) 
	private static final int FUNCTION_CODE = 0;
	private static final int DATA_BYTES = 1;
	private static final int GRIPPER_STATUS = 2;
	private static final int OBJECT_STATUS = 3;
	private static final int FAULT_STATUS = 4;
	private static final int FINGER_A_REQUESTED_POSITION = 5;
	private static final int FINGER_A_POSITION = 6;
	private static final int FINGER_A_CURRENT = 7;
	private static final int FINGER_B_REQUESTED_POSITION = 8;
	private static final int FINGER_B_POSITION = 9;
	private static final int FINGER_B_CURRENT = 10;
	private static final int FINGER_C_REQUESTED_POSITION = 11;
	private static final int FINGER_C_POSITION = 12;
	private static final int FINGER_C_CURRENT = 13;
	private static final int SCISSOR_REQUESTED_POSITION = 14;
	private static final int SCISSOR_POSITION = 15;
	private static final int SCISSOR_CURRENT = 16;
	
	//control parameter index
	private static final int FINGER_A = 0;
	private static final int FINGER_B = 1;
	private static final int FINGER_C = 2;
	private static final int SCISSOR = 3;
	
	/*---FUNCTIONALITY REGISTER BITMASKS---*/
	//byte 0 (action request)
	private static final byte INITIALIZATON_MASK =		0b00000001;
	private static final byte OPERATION_MODE_MASK =		0b00000110;
	private static final byte GO_TO_REQUESTED_MASK =	0b00001000;
	private static final byte AUTOMATIC_RELEASE_MASK = 	0b00010000;
	//last three bits reserved
	
	//byte 1 (gripper options 1)
	private static final byte GLOVE_MODE_MASK = 				0b00000001;
	//bit 1 reserved
	private static final byte INDIVIDUAL_FINGER_CONTROL_MASK = 	0b00000100;
	private static final byte INDIVIDUAL_SCISSOR_CONTROL_MASK =	0b00001000;
	//bits 4 to 7 reserved
	
	//bytes 2 to 14 do not require masks
	
	
	/*---STATUS REGISTER BITMASKS---*/
	//byte 0 (gripper status 
	/*
	private static final byte INITIALIZATON_MASK =		0b00000001; //same as functionality mask
	private static final byte OPERATION_MODE_MASK =		0b00000110; //same as functionality mask
	private static final byte GO_TO_REQUESTED_MASK =	0b00001000; //same as functionality mask
	 */
	private static final byte INIT_MODE_STATUS_MASK =	 0b00110000;
	private static final byte MOTION_STATUS_MASK = (byte)0b11000000;
	
	//byte 1 (object status)
	private static final byte OBJECT_DETECTION_A_MASK = 	  0b00000011;
	private static final byte OBJECT_DETECTION_B_MASK = 	  0b00001100;
	private static final byte OBJECT_DETECTION_C_MASK =       0b00110000;
	private static final byte OBJECT_DETECTION_S_MASK =	(byte)0b11000000;
	
	//bytes 2 to 14 do not require masks
	
	
	/*---FUNCTIONALITY REGISTER VALUES---*/
	//byte 0 (action request)
	//bit 0
	private static final byte RESET = 			0b00000000;
	private static final byte INITIALIZE = 		0b00000001;
	//bits 1 and 2
	private static final byte BASIC_MODE = 		0b00000000;
	private static final byte PINCH_MODE = 		0b00000010;
	private static final byte WIDE_MODE = 		0b00000100;
	private static final byte SCISSOR_MODE = 	0b00000110; 
	//bit 3
	private static final byte STANDBY = 		0b00000000;
	private static final byte GO_TO_REQUESTED =	0b00001000;
	//bit 4
	private static final byte AUTO_RELEASE_DISABLED =	0b00010000;
	private static final byte AUTO_RELEASE_ENABLED =	0b00010000;
	//bits 5 to 7 reserved
	
	//byte 1 (gripper options 1)
	//bit 0
	private static final byte GLOVE_MODE_DISABLED = 	0b00000000;
	private static final byte GLOVE_MODE_ENABLED = 		0b00000001;
	//bit 1 reserved
	//bit 2
	private static final byte CONCURRENT_FINGER_CONTROL = 	0b00000000;
	private static final byte INDIVIDUAL_FINGER_CONTROL = 	0b00000100;
	//bit 3
	private static final byte CONCURRENT_SCISSOR_CONTROL = 	0b00000000;
	private static final byte INDIVIDUAL_SCISSOR_CONTROL = 	0b00001000;
	//bits 4 to 7 reserved
	
	//byte 2 (gripper options 2)
	//byte reserved
	
	//byte 3 (position request finger A)
	/* Value:
	 * 0x00 (fully open) to 0xFF (fully closed)
	 */
	
	//byte 4 (grasping speed of finger A)
	/* Value:
	 * 0x00 (minimum speed) to 0xFF (maximum speed)
	 */
	
	//byte 5 (gripping force of finger A)
	/* Value:
	 * 0x00 (minimum force) to 0xFF (maximum force)
	 */
	
	//byte 6 (position request finger B)
	/* Value:
	 * 0x00 (fully open) to 0xFF (fully closed)
	 */
	
	//byte 7 (grasping speed of finger B)
	/* Value:
	 * 0x00 (minimum speed) to 0xFF (maximum speed)
	 */
	
	//byte 8 (gripping force of finger B)
	/* Value:
	 * 0x00 (minimum force) to 0xFF (maximum force)
	 */
	
	//byte 9 (position request finger C)
	/* Value:
	 * 0x00 (fully open) to 0xFF (fully closed)
	 */
	
	//byte 10 (grasping speed of finger C)
	/* Value:
	 * 0x00 (minimum speed) to 0xFF (maximum speed)
	 */
	
	//byte 11 (gripping force of finger C)
	/* Value:
	 * 0x00 (minimum force) to 0xFF (maximum force)
	 */
	
	//byte 12 (position request scissor
	/* Value:
	 * 0x00 (fully open) to 0xFF (fully closed)
	 */
	
	//byte 13 (grasping speed of scissor)
	/* Value:
	 * 0x00 (minimum speed) to 0xFF (maximum speed)
	 */
	
	//byte 14 (gripping force of scissor)
	/* Value:
	 * 0x00 (minimum force) to 0xFF (maximum force)
	 */
	
	
	/*---STATUS REGISTER VALUES---*/
	//byte 0 (gripper status)
	//bit 0
	/*
	private static final byte RESET = 		0b00000000;  //same as function value
	 */
	private static final byte INITIALIZED = 	0b00000001;
	//bits 1 and 2
	/*
	private static final byte BASIC_MODE = 		0b00000000;  //same as function value
	private static final byte WIDE_MODE = 		0b00000010;  //same as function value
	private static final byte PINCH_MODE = 		0b00000100;  //same as function value
	private static final byte SCISSOR_MODE = 	0b00000110;   //same as function value
	 */
	//bit 3
	/*
	private static final byte STANDBY = 			0b00000000;  //same as function value
	private static final byte GO_TO_REQUESTED =		0b00001000;  //same as function value
	 */
	//bits 4 and 5
	private static final byte RESET_STATE = 	0b00000000;
	private static final byte ACTIVATING = 		0b00010000;
	private static final byte CHANGING_MODE = 	0b00100000;
	private static final byte COMPLETED = 		0b00110000;
	//bits 6 and 7
	private static final byte IN_MOTION = 				0b00000000;
	private static final byte STOPPED_SOME_EARLY = 		0b01000000; //1 or 2 fingers stopped before desired position
	private static final byte STOPPED_ALL_EARLY = (byte)0b10000000; //all fingers stopped before the desired position
	private static final byte STOPPED_AT_DESIRED =(byte)0b11000000;
	
	//byte 1 (object status)
	//bits 0 and 1
	private static final byte A_IN_MOTION = 		0b00000000;
	private static final byte A_STOPPED_OPEN = 		0b00000001; //stopped from contact while opening
	private static final byte A_STOPPED_CLOSED = 	0b00000010; //stopped from contact while closing
	private static final byte A_AT_DESIRED = 		0b00000011;
	//bits 2 and 3
	private static final byte B_IN_MOTION = 		0b00000000;
	private static final byte B_STOPPED_OPEN = 		0b00000100; //stopped from contact while opening
	private static final byte B_STOPPED_CLOSED = 	0b00001000; //stopped from contact while closing
	private static final byte B_AT_DESIRED = 		0b00001100;
	//bits 4 and 5
	private static final byte C_IN_MOTION = 		0b00000000;
	private static final byte C_STOPPED_OPEN = 		0b00010000; //stopped from contact while opening
	private static final byte C_STOPPED_CLOSED = 	0b00100000; //stopped from contact while closing
	private static final byte C_AT_DESIRED = 		0b00110000;
	//bits 6 and 7
	private static final byte S_IN_MOTION = 		0b00000000;
	private static final byte S_STOPPED_OPEN = 		0b01000000; //stopped from contact while opening
	private static final byte S_STOPPED_CLOSED = 	(byte)0b10000000; //stopped from contact while closing
	private static final byte S_AT_DESIRED = 		(byte)0b11000000;
	
	//byte 2 (fault status)
	private static final byte NO_FAULT = 0x00;
	/* Fault Codes:
	 * 0x00: No Fault.
	 * 0x05: Priority Fault: Action delayed, activation must be completed prior to action.
	 * 0x06: Priority Fault: Action delayed, mode change must be completed prior to action.
	 * 0x07: Priority Fault: The activation bit must be set prior to action.
	 * 0x09: Minor Fault: Communication is not ready (may be booting).
	 * 0x0A: Minor Fault: Changing mode fault, interferences detected on scissor axis (less then 20s).
	 * 0x0B: Minor Fault: Automatic release in progress.
	 * 0x0D: Major Fault: Action fault, verify that no interference or other error occurred.
	 * 0x0E: Major Fault: Changing mode fault, interferences detected on scissor axis (more then 20s).
	 * 0x0F: Major Fault: Automatic release completed. Reset and activation required.
	 */
	
	//byte 3 (position request echo finger A)
	/* Value:
	 * 0x00 (fully open) to 0xFF (fully closed)
	 */
	
	//byte 4 (current position of finger A)
	/* Value:
	 * 0x00 (fully open) to 0xFF (fully closed)
	 */
	
	//byte 5 (current of finger A motor)
	/* Value:
	 * 0x00 to 0xFF (0.1*Current[mA])
	 */
	
	//byte 6 (position request echo finger B)
	/* Value:
	 * 0x00 (fully open) to 0xFF (fully closed)
	 */
	
	//byte 7 (current position of finger B)
	/* Value:
	 * 0x00 (fully open) to 0xFF (fully closed)
	 */
	
	//byte 8 (current of finger B motor)
	/* Value:
	 * 0x00 to 0xFF (0.1*Current[mA])
	 */
	
	//byte 9 (position request echo finger C)
	/* Value:
	 * 0x00 (fully open) to 0xFF (fully closed)
	 */
	
	//byte 10 (current position of finger C)
	/* Value:
	 * 0x00 (fully open) to 0xFF (fully closed)
	 */
	
	//byte 11 (current of finger C motor)
	/* Value:
	 * 0x00 to 0xFF (0.1*Current[mA])
	 */
	
	//byte 12 (position request echo scissor
	/* Value:
	 * 0x00 (fully open) to 0xFF (fully closed)
	 */
	
	//byte 13 (current position of scissor)
	/* Value:
	 * 0x00 (fully open) to 0xFF (fully closed)
	 */
	
	//byte 14 (current of scissor motor)
	/* Value:
	 * 0x00 to 0xFF (0.1*Current[mA])
	 */
	private static final int[] fingers = {FINGER_A, FINGER_B, FINGER_C, SCISSOR};
	private ModbusTCPConnection connection;
	private byte initializedStatus;
	private byte operationMode;
	private byte commandedStatus;
	private byte fingerControl;
	private byte scissorControl;
	private byte[] data = new byte[32]; //buffer extracted for efficiency
	private byte[] status;
	private byte[] speed = new byte[4];
	private byte[] force = new byte[4];
	private byte[] position = new byte[4];
	private int faultCounter = 0;
	private boolean connected = false;
	private String address;
	
	RobotiqHandInterface()
	{
		this(RobotiqHandParameters.LEFT_HAND_ADDRESS);
	}
	
	public RobotiqHandInterface(String address)
	{
		this.address = address;
	}
	
	public boolean connect()
	{
		try
		{
			connection = new ModbusTCPConnection(address,RobotiqHandParameters.PORT);
			connected = true;
		}
		catch(IOException e)
		{
			connected = false;
		}
		return connected;
	}
	
	public void initialize()
	{
	   if(!this.isConnected())
			this.connect();
			
		status = this.getStatus();
		
		if(status == null)
		   return;
		
		if(((status[GRIPPER_STATUS] & INITIALIZATON_MASK) == INITIALIZED) || (status[FAULT_STATUS] != NO_FAULT))
		{
			this.reset();
		}
		
		//initialize arrays
		Arrays.fill(position,(byte)0x00);
		Arrays.fill(speed,(byte)0x00);
		Arrays.fill(force,(byte)0x00);
		
		position[FINGER_A] = FULLY_OPEN;
		speed[FINGER_A] = DEFAULT_SPEED;
		force[FINGER_A] = MAX_FORCE/2;
		
		speed[FINGER_B] = DEFAULT_SPEED;
		force[FINGER_B] = MAX_FORCE/2;
		speed[FINGER_C] = DEFAULT_SPEED;
		force[FINGER_C] = MAX_FORCE/2;
		
		initializedStatus = INITIALIZED;
		operationMode = BASIC_MODE;
		commandedStatus = STANDBY;
		fingerControl = CONCURRENT_FINGER_CONTROL;
		scissorControl = CONCURRENT_SCISSOR_CONTROL;
		
		sendRequest(SET_REGISTERS,
				REGISTER_START,
				(byte)(initializedStatus | operationMode | commandedStatus),
				(byte)(fingerControl | scissorControl),
				(byte)0x00,	//reserved byte
				(byte)position[FINGER_A], //all finger control
				(byte)speed[FINGER_A],
				(byte)force[FINGER_A]);
		
//		do
//		{
//			sendRequest(SET_REGISTERS,
//						REGISTER_START,
//						(byte)(initializedStatus | operationMode | commandedStatus),
//						(byte)(fingerControl | scissorControl),
//						(byte)0x00,	//reserved byte
//						(byte)position[FINGER_A], //all finger control
//						(byte)speed[FINGER_A],
//						(byte)force[FINGER_A]);
//			do
//			{
//				ThreadTools.sleep(1000);
//				status = this.getStatus();
//			}while(((status[GRIPPER_STATUS] & INIT_MODE_STATUS_MASK) == ACTIVATING) && (status[FAULT_STATUS] == NO_FAULT)); //check/wait while activation is in progress
//			
//			if(status[FAULT_STATUS] != 0x00) //checking for errors
//			{
//				faultCounter++;
//				if(faultCounter < 3)
//					this.reset();
//				else
//				{
//					return;
//				}
//			}
//		}while ((status[GRIPPER_STATUS] & INIT_MODE_STATUS_MASK) != COMPLETED); //check to see if activation was successful, else resend command	
//		
//		faultCounter = 0; //reset fault counter
	}

	public void reset()
	{
		initializedStatus = RESET; //reset activation status
		commandedStatus = STANDBY;
		int errorCount = 0;
		do
		{
			if(errorCount > 5)
				throw new RuntimeException("Unable to reset hand. Unrecoverable error");
			sendRequest(SET_REGISTERS,
						REGISTER_START,
						(byte)(initializedStatus | operationMode | commandedStatus),
						(byte)(fingerControl | scissorControl));
		
			ThreadTools.sleep(200);
			status = this.getStatus();
			errorCount++;
		}while((status[GRIPPER_STATUS] & INITIALIZATON_MASK) != RESET); //check until reset
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

	public void open()
	{
		
		position[FINGER_A] = FULLY_OPEN; //all fingers
		if(fingerControl == INDIVIDUAL_FINGER_CONTROL)
		{
			position[FINGER_B] = FULLY_OPEN;
			position[FINGER_C] = FULLY_OPEN;
		}
		sendMotionRequest();
//		blockDuringMotion();
	}
	
	public void open(double percent)
	{
		position[FINGER_A] = (byte)((1-percent) * (0xFF & FULLY_CLOSED)); //all fingers
		if(fingerControl == INDIVIDUAL_FINGER_CONTROL)
		{
			position[FINGER_B] = (byte)((1-percent) * (0xFF & FULLY_CLOSED));
			position[FINGER_C] = (byte)((1-percent) * (0xFF & FULLY_CLOSED));
		}
		sendMotionRequest();
//		blockDuringMotion();
	}
	
	public void close()
	{
		if(operationMode == PINCH_MODE)
		{
			position[FINGER_A] = 120; //max closed position for pinch
			force[FINGER_A] = MAX_FORCE/2;
		}
		else
		{
			position[FINGER_A] = FULLY_CLOSED; // also all fingers
			force[FINGER_A] = MAX_FORCE/2;
			if(fingerControl == INDIVIDUAL_FINGER_CONTROL)
			{
				position[FINGER_B] = FULLY_CLOSED;
				force[FINGER_B] = MAX_FORCE/2;
				position[FINGER_C] = FULLY_CLOSED;
				force[FINGER_C] = MAX_FORCE/2;
			}
		}
		sendMotionRequest();
//		blockDuringMotion();
	}
	
	public void close(double percent) throws InterruptedException
	{
		if(operationMode == PINCH_MODE)
		{
			position[FINGER_A] = (byte)(percent * 120); //max closed position for pinch
			force[FINGER_A] = MAX_FORCE/2;
		}
		else
		{
			position[FINGER_A] = (byte)(percent * (0xFF & FULLY_CLOSED)); // also all fingers
			force[FINGER_A] = MAX_FORCE/2;
			if(fingerControl == INDIVIDUAL_FINGER_CONTROL)
			{
				position[FINGER_B] = (byte)(percent * (0xFF & FULLY_CLOSED));
				force[FINGER_B] = MAX_FORCE/2;
				position[FINGER_C] = (byte)(percent * (0xFF & FULLY_CLOSED));
				force[FINGER_C] = MAX_FORCE/2;
			}
		}
		sendMotionRequest();
//		blockDuringMotion();
	}
	
	public void crush()
	{
		if(operationMode == PINCH_MODE)
		{
			position[FINGER_A] = 120; //max closed position for pinch
			force[FINGER_A] = MAX_FORCE;
		}
		else
		{
			position[FINGER_A] = FULLY_CLOSED; // also all fingers
			force[FINGER_A] = MAX_FORCE;
			if(fingerControl == INDIVIDUAL_FINGER_CONTROL)
			{
				position[FINGER_B] = FULLY_CLOSED;
				force[FINGER_B] = MAX_FORCE;
				position[FINGER_C] = FULLY_CLOSED;
				force[FINGER_C] = MAX_FORCE;
			}
		}
		sendMotionRequest();
//		blockDuringMotion();
	}

	void blockDuringMotion()
	{
		do
		{
			ThreadTools.sleep(200);
			status = this.getStatus();
		}while((status[GRIPPER_STATUS] & MOTION_STATUS_MASK) == IN_MOTION);
	}
	
	public void stop()
	{
		status = getStatus();
		position[FINGER_A] = status[FINGER_A_POSITION];
		position[FINGER_B] = status[FINGER_B_POSITION];
		position[FINGER_C] = status[FINGER_C_POSITION];
		position[SCISSOR] = status[SCISSOR_POSITION];
		sendMotionRequest();
	}
	
	public void pinchGrip()
	{
		setGrip(PINCH_MODE);
	}
	
	public void normalGrip()
	{
		setGrip(BASIC_MODE);
	}
	
	public void wideGrip()
	{
		setGrip(WIDE_MODE);
	}
	
	public void scissorGrip()
	{
		setGrip(SCISSOR_MODE);
	}

	private void setGrip(byte mode)
	{
		if(operationMode != mode)
		{
			fingerControl = CONCURRENT_FINGER_CONTROL;
			scissorControl = CONCURRENT_SCISSOR_CONTROL;
			operationMode = mode;
			sendMotionRequest();
			blockDuringGripChange();
		}
	}

	private void blockDuringGripChange()
	{
		do
		{
			ThreadTools.sleep(50);
			status = this.getStatus();
		}while((status[GRIPPER_STATUS] & INIT_MODE_STATUS_MASK) == CHANGING_MODE);
	}
	
	public void setIndividualFinger(int finger, byte desiredForce, byte desiredPosition, byte desiredSpeed)
	{
		if(fingerControl == CONCURRENT_FINGER_CONTROL || scissorControl == CONCURRENT_SCISSOR_CONTROL)
			status = getStatus();
		if(fingerControl == CONCURRENT_FINGER_CONTROL)
		{
			position[FINGER_A] = status[FINGER_A_REQUESTED_POSITION];
			position[FINGER_B] = status[FINGER_B_REQUESTED_POSITION];
			position[FINGER_C] = status[FINGER_C_REQUESTED_POSITION];
			force[FINGER_B] = force[FINGER_A];
			force[FINGER_C] = force[FINGER_A];
			speed[FINGER_B] = speed[FINGER_A];
			speed[FINGER_C] = speed[FINGER_A];
			fingerControl = INDIVIDUAL_FINGER_CONTROL;
		}
		if(finger == SCISSOR)
		{
			position[SCISSOR] = status[SCISSOR_REQUESTED_POSITION];
			scissorControl = INDIVIDUAL_SCISSOR_CONTROL;
		}
		position[finger] = desiredPosition;
		speed[finger] = desiredSpeed;
		force[finger] = desiredForce;
	}
	
	public void velocityControl(int[] desiredSpeed, RobotSide side)
	{
		
		if(side == RobotSide.LEFT)
		{
			int temp = desiredSpeed[FINGER_C];
			desiredSpeed[FINGER_C] = desiredSpeed[FINGER_B];
			desiredSpeed[FINGER_B] = temp;
		}
		
		byte desiredPosition;
		
		for(int finger : fingers)
		{
			desiredPosition = (desiredSpeed[finger] > 0) ? FULLY_CLOSED : FULLY_OPEN;
			force[finger] = MAX_FORCE;
			speed[finger] = (byte)(desiredSpeed[finger] & 0xFF);
			position[finger] = desiredPosition;
		}
		
		fingerControl = INDIVIDUAL_FINGER_CONTROL;
		scissorControl = INDIVIDUAL_SCISSOR_CONTROL;
		sendMotionRequest();
	}
	
	public void positionControl(int desiredPosition[], RobotSide side)
	{
		if(side == RobotSide.LEFT)
		{
			int temp = desiredPosition[FINGER_C];
			desiredPosition[FINGER_C] = desiredPosition[FINGER_B];
			desiredPosition[FINGER_B] = temp;
		}
		
		for(int finger : fingers)
		{
			force[finger] = MAX_FORCE;
			speed[finger] = DEFAULT_SPEED;
			position[finger] = (byte)(desiredPosition[finger] & 0xFF);
		}
		
		fingerControl = INDIVIDUAL_FINGER_CONTROL;
		scissorControl = INDIVIDUAL_SCISSOR_CONTROL;
		sendMotionRequest();
	}
	
	public void hook(RobotSide side)
	{
		if(side == RobotSide.LEFT)
		{
			setIndividualFinger(FINGER_A, force[FINGER_A], FULLY_CLOSED, speed[FINGER_A]);
			setIndividualFinger(FINGER_B, force[FINGER_B], FULLY_CLOSED, speed[FINGER_B]);
			setIndividualFinger(FINGER_C, force[FINGER_C], FULLY_OPEN, speed[FINGER_C]); //index finger of left Hand
		}
		else if(side == RobotSide.RIGHT)
		{
			setIndividualFinger(FINGER_A, force[FINGER_A], FULLY_CLOSED, speed[FINGER_A]);
			setIndividualFinger(FINGER_B, force[FINGER_B], FULLY_OPEN, speed[FINGER_B]); //index finger of right Hand
			setIndividualFinger(FINGER_C, force[FINGER_C], FULLY_CLOSED, speed[FINGER_C]); 
		}
		else
		{
			setIndividualFinger(FINGER_A, force[FINGER_A], FULLY_OPEN, speed[FINGER_A]);
			setIndividualFinger(FINGER_B, force[FINGER_B], FULLY_CLOSED, speed[FINGER_B]);
			setIndividualFinger(FINGER_C, force[FINGER_C], FULLY_CLOSED, speed[FINGER_C]);
		}
		sendMotionRequest();
	}
	
	private float[] positions = new float[4]; //only used in this method and is extracted for efficiency
	public float[] positionStatus()
	{
		status = getStatus();
		positions[FINGER_A] = status[FINGER_A_POSITION];
		positions[FINGER_B] = status[FINGER_B_POSITION];
		positions[FINGER_C] = status[FINGER_C_POSITION];
		positions[SCISSOR] = status[SCISSOR_POSITION];
		for(float p : positions)
		{
			if(p < 0)
				p += 256; //rectify two's comp because Java doesn't have unsigned bytes
			p /= 255;
		}
		return positions;
	}
	
	private RobotiqHandSensorData handData = new RobotiqHandSensorData(null);
	public RobotiqHandSensorData getHandStatus()
	{
		do
		{
		   status = getStatus();
		}while(status.length < 22);
		
		data[0] = (byte) (status[GRIPPER_STATUS] & INITIALIZATON_MASK);
		data[1] = (byte) ((status[GRIPPER_STATUS] & OPERATION_MODE_MASK) >> 1);
		data[2] = (byte) ((status[GRIPPER_STATUS] & GO_TO_REQUESTED_MASK) >> 3);
		data[3] = (byte) ((status[GRIPPER_STATUS] & INIT_MODE_STATUS_MASK) >> 4);
		data[4] = (byte) ((status[GRIPPER_STATUS] & MOTION_STATUS_MASK) >> 6);
		data[5] = (byte) (status[OBJECT_STATUS] & OBJECT_DETECTION_A_MASK);
		data[6] = (byte) ((status[OBJECT_STATUS] & OBJECT_DETECTION_B_MASK) >> 2);
		data[7] = (byte) ((status[OBJECT_STATUS] & OBJECT_DETECTION_C_MASK) >> 4);
		data[8] = (byte) ((status[OBJECT_STATUS] & OBJECT_DETECTION_S_MASK) >> 6);
		data[9] = status[FAULT_STATUS];
		data[10] = status[FINGER_A_REQUESTED_POSITION];
		data[11] = status[FINGER_A_POSITION];
		data[12] = status[FINGER_A_CURRENT];
		data[13] = status[FINGER_B_REQUESTED_POSITION];
		data[14] = status[FINGER_B_POSITION];
		data[15] = status[FINGER_B_CURRENT];
		data[16] = status[FINGER_C_REQUESTED_POSITION];
		data[17] = status[FINGER_C_POSITION];
		data[18] = status[FINGER_C_CURRENT];
		data[19] = status[SCISSOR_REQUESTED_POSITION];
		data[20] = status[SCISSOR_POSITION];
		data[21] = status[SCISSOR_CURRENT];
		
		handData.update(data);
		
		return handData;
	}
	
	public boolean isReady()
	{
		status = getStatus();
		return (status[GRIPPER_STATUS] & INITIALIZATON_MASK) == INITIALIZED;
	}
	
	public boolean doneInitializing()
	{
		status = getStatus();
		return (status[GRIPPER_STATUS] & INIT_MODE_STATUS_MASK) != COMPLETED;
	}
	
	public boolean isConnected()
	{
		if(connected)
		{
		   connected = connection.testConnection();
		}
	   return connected;
	}
	
	public void doControl()
	{
		sendMotionRequest();
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
		if(functionCode == READ_REGISTERS)
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
		try
		{
			return connection.transcieve(RobotiqHandParameters.UNIT_ID, Arrays.copyOfRange(data, 0, dataLength));
		}
		catch (SocketException e)
		{
			connected = false;
			e.printStackTrace();
		}
		catch(ModbusResponseTooShortException e)
		{
			e.printStackTrace();
		}
		catch(ModbusException e)
		{
			e.printStackTrace();
		}
		catch (IOException e)
		{
			System.err.println("Unable to send Modbus request");
			e.printStackTrace();
		}
		return null; //should probably return something a bit more useful that won't break things
	}
	
	private void sendMotionRequest()
	{
		int dataLength;
		commandedStatus = GO_TO_REQUESTED;
		
		data[0] = (byte)(initializedStatus | operationMode | commandedStatus); //Whenever sending a motion request, the command hand positions bit (GO_TO_REQUESTED) must be sent
		data[1] = (byte)(fingerControl | scissorControl);
		data[2] = 0x00; //reserved byte
		//update finger a/all fingers
		data[3] = position[FINGER_A];
		data[4] = speed[FINGER_A];
		data[5] = force[FINGER_A];
		dataLength = 6;
		if(fingerControl == INDIVIDUAL_FINGER_CONTROL)
		{
			//update finger b
			data[6] = position[FINGER_B];
			data[7] = speed[FINGER_B];
			data[8] = force[FINGER_B];
			//update finger c
			data[9] = position[FINGER_C];
			data[10] = speed[FINGER_C];
			data[11] = force[FINGER_C];
			dataLength = 12;
		}
		if(scissorControl == INDIVIDUAL_SCISSOR_CONTROL)
		{
			data[12] = position[SCISSOR];
			data[13] = speed[SCISSOR];
			data[14] = force[SCISSOR];
			data[15] = 0x00; //included to send an even number of registers. (see the sendRequest() warning)
			dataLength = 16;
		}
		sendRequest(SET_REGISTERS, REGISTER_START, Arrays.copyOfRange(data,0,dataLength));
	}
	
	private byte[] getStatus() //gets the status of every register
	{
		status = sendRequest(READ_REGISTERS,
							 REGISTER_START,
							 (byte)0x08);
		return status;
	}
}

