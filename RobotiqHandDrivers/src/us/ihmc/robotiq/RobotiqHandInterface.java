package us.ihmc.robotiq;

import java.io.IOException;
import java.util.Arrays;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotiq.communication.ModbusTCPConnection;
import us.ihmc.robotiq.data.RobotiqHandSensorData;
import us.ihmc.tools.thread.ThreadTools;

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
	private final int REGISTER_START = 0x0000;
	
	
	/*---FUNCTION CODES---*/
	private final byte SET_REGISTERS = 0x10;
	private final byte READ_REGISTERS = 0x04;
	
	/*---CONTROL PARAMETER INDEX---*/
	private final int FINGER_A = 0;
	private final int FINGER_B = 1;
	private final int FINGER_C = 2;
	private final int SCISSOR = 3;
	
	
	/*---CONTROL PARAMETERS---*/
	public final byte MAX_SPEED = (byte) 0xFF;
	public final byte MIN_SPEED = 0x00;
	public final byte MAX_FORCE = (byte) 0xFF;
	public final byte DEFAULT_FORCE = (byte) 0x7F; //half of MAX_FORCE
	public final byte MIN_FORCE = 0x00;
	
	public final byte[] BASIC_MODE_FULLY_OPEN = new byte[]{(byte)0x00, (byte)0x00, (byte)0x00, (byte)0x8C};
	public final byte[] BASIC_MODE_FULLY_CLOSED = new byte[]{(byte)0xFF, (byte)0xFF, (byte)0xFF, (byte)0x8C};
	public final byte[] PINCH_MODE_FULLY_OPEN = new byte[]{(byte)0x00, (byte)0x00, (byte)0x00, (byte)0xDC};
	public final byte[] PINCH_MODE_FULLY_CLOSED = new byte[]{(byte)0x78, (byte)0x78, (byte)0x78, (byte)0xDC};
	public final byte[] WIDE_MODE_FULLY_OPEN = new byte[]{(byte)0x00, (byte)0x00, (byte)0x00, (byte)0x19};
	public final byte[] WIDE_MODE_FULLY_CLOSED = new byte[]{(byte)0xFF, (byte)0xFF, (byte)0xFF, (byte)0x19};
	public final byte[] SCISSOR_MODE_FULLY_OPEN = new byte[]{(byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00};
	public final byte[] SCISSOR_MODE_FULLY_CLOSED = new byte[]{(byte)0x00, (byte)0x00, (byte)0x00, (byte)0xFF};
	
	/*---DATA INDICES---*/
	//status response index (from hand start register) 
	private final int FUNCTION_CODE = 0;
	private final int DATA_BYTES = 1;
	private final int GRIPPER_STATUS = 2;
	private final int OBJECT_DETECTION = 3;
	private final int FAULT_STATUS = 4;
	private final int FINGER_A_REQUESTED_POSITION = 5;
	private final int FINGER_A_POSITION = 6;
	private final int FINGER_A_CURRENT = 7;
	private final int FINGER_B_REQUESTED_POSITION = 8;
	private final int FINGER_B_POSITION = 9;
	private final int FINGER_B_CURRENT = 10;
	private final int FINGER_C_REQUESTED_POSITION = 11;
	private final int FINGER_C_POSITION = 12;
	private final int FINGER_C_CURRENT = 13;
	private final int SCISSOR_REQUESTED_POSITION = 14;
	private final int SCISSOR_POSITION = 15;
	private final int SCISSOR_CURRENT = 16;
	
	/*---FUNCTIONALITY REGISTER BITMASKS---*/
	//byte 0 (action request)
	private final byte INITIALIZATON_MASK =		0b00000001;
	private final byte OPERATION_MODE_MASK =		0b00000110;
	private final byte GO_TO_REQUESTED_MASK =	0b00001000;
	private final byte AUTOMATIC_RELEASE_MASK = 	0b00010000;
	//last three bits reserved
	
	//byte 1 (gripper options 1)
	private final byte GLOVE_MODE_MASK = 				0b00000001;
	//bit 1 reserved
	private final byte INDIVIDUAL_FINGER_CONTROL_MASK = 	0b00000100;
	private final byte INDIVIDUAL_SCISSOR_CONTROL_MASK =	0b00001000;
	//bits 4 to 7 reserved
	
	//bytes 2 to 14 do not require masks
	
	
	/*---STATUS REGISTER BITMASKS---*/
	//byte 0 (gripper status 
	/*
//	private final byte INITIALIZATON_MASK =		0b00000001; //same as functionality mask
	private final byte OPERATION_MODE_MASK =		0b00000110; //same as functionality mask
	private final byte GO_TO_REQUESTED_MASK =	0b00001000; //same as functionality mask
	 */
	private final byte INIT_MODE_STATUS_MASK =	 0b00110000;
	private final byte MOTION_STATUS_MASK = (byte)0b11000000;
	
	//byte 1 (object status)
	private final byte OBJECT_DETECTION_A_MASK = 	  0b00000011;
	private final byte OBJECT_DETECTION_B_MASK = 	  0b00001100;
	private final byte OBJECT_DETECTION_C_MASK =       0b00110000;
	private final byte OBJECT_DETECTION_S_MASK =	(byte)0b11000000;
	
	//bytes 2 to 14 do not require masks
	
	
	/*---Robot output registers & functionalities (data going TO the hand)---*/
	//byte 0 (action request)
	//bit 0
	private final byte RESET = 			0b00000000;
	private final byte INITIALIZE = 		0b00000001;
	//bits 1 and 2
	private final byte BASIC_MODE = 		0b00000000;
	private final byte PINCH_MODE = 		0b00000010;
	private final byte WIDE_MODE = 		0b00000100;
	private final byte SCISSOR_MODE = 	0b00000110;
	//bit 3
	private final byte STANDBY = 		0b00000000;
	private final byte GO_TO_REQUESTED =	0b00001000;
	//bit 4
	private final byte AUTO_RELEASE_DISABLED =	0b00000000;
	private final byte AUTO_RELEASE_ENABLED =	0b00010000;
	//bits 5 to 7 reserved
	
	//byte 1 (gripper options 1)
	//bits 0 & 1 reserved
	//bit 2
//	private final byte CONCURRENT_FINGER_CONTROL = 	0b00000000;
	private final byte INDIVIDUAL_FINGER_CONTROL = 	0b00000100;
	//bit 3
	private final byte CONCURRENT_SCISSOR_CONTROL = 	0b00000000;
	private final byte INDIVIDUAL_SCISSOR_CONTROL = 	0b00001000;
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
	
	
	/*---Robot input registers & status (data coming FROM the hand)---*/
	//byte 0 (gripper status)
	//bit 0
	/*
	private final byte RESET = 		0b00000000;  //same as function value
	 */
	private final byte INITIALIZED = 	0b00000001;
	//bits 1 and 2
	/*
	private final byte BASIC_MODE = 		0b00000000;  //same as function value
	private final byte WIDE_MODE = 		0b00000010;  //same as function value
	private final byte PINCH_MODE = 		0b00000100;  //same as function value
	private final byte SCISSOR_MODE = 	0b00000110;   //same as function value
	 */
	//bit 3
	/*
	private final byte STANDBY = 			0b00000000;  //same as function value
	private final byte GO_TO_REQUESTED =		0b00001000;  //same as function value
	 */
	//bits 4 and 5
	private final byte RESET_STATE = 	0b00000000;
	private final byte ACTIVATING = 		0b00010000;
	private final byte CHANGING_MODE = 	0b00100000;
	private final byte COMPLETED = 		0b00110000;
	//bits 6 and 7
	private final byte IN_MOTION = 				0b00000000;
	private final byte STOPPED_SOME_EARLY = 		0b01000000; //1 or 2 fingers stopped before desired position
	private final byte STOPPED_ALL_EARLY = (byte)0b10000000; //all fingers stopped before the desired position
	private final byte STOPPED_AT_DESIRED =(byte)0b11000000;
	
	//byte 1 (object status)
	//bits 0 and 1
	private final byte A_IN_MOTION = 		0b00000000;
	private final byte A_STOPPED_OPEN = 		0b00000001; //stopped from contact while opening
	private final byte A_STOPPED_CLOSED = 	0b00000010; //stopped from contact while closing
	private final byte A_AT_DESIRED = 		0b00000011;
	//bits 2 and 3
	private final byte B_IN_MOTION = 		0b00000000;
	private final byte B_STOPPED_OPEN = 		0b00000100; //stopped from contact while opening
	private final byte B_STOPPED_CLOSED = 	0b00001000; //stopped from contact while closing
	private final byte B_AT_DESIRED = 		0b00001100;
	//bits 4 and 5
	private final byte C_IN_MOTION = 		0b00000000;
	private final byte C_STOPPED_OPEN = 		0b00010000; //stopped from contact while opening
	private final byte C_STOPPED_CLOSED = 	0b00100000; //stopped from contact while closing
	private final byte C_AT_DESIRED = 		0b00110000;
	//bits 6 and 7
	private final byte S_IN_MOTION = 		0b00000000;
	private final byte S_STOPPED_OPEN = 		0b01000000; //stopped from contact while opening
	private final byte S_STOPPED_CLOSED = 	(byte)0b10000000; //stopped from contact while closing
	private final byte S_AT_DESIRED = 		(byte)0b11000000;
	
	//byte 2 (fault status)
	private final byte NO_FAULT = 0x00;
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
	private final int[] fingers = {FINGER_A, FINGER_B, FINGER_C, SCISSOR};
	private ModbusTCPConnection connection;
	private byte initializedStatus;
	private byte operationMode;
	private byte commandedStatus;
	private byte fingerControl;
	private byte scissorControl;
	private byte[] dataFromHand = new byte[32]; //buffer extracted for efficiency
	private byte[] dataToSend = new byte[32]; //buffer extracted for efficiency
	private byte[] status;
	private byte[] speed = new byte[4];
	private byte[] force = new byte[4];
	private byte[] position = new byte[4];
//	private int faultCounter = 0;
	private boolean connected = false;
	private String address;
	
	private RobotiqHandSensorData handData = new RobotiqHandSensorData();
	
	public RobotiqHandInterface(String handIP)
	{
		this.address = handIP;
		
		if(connect())
		{
			reset();
			new Thread(new ConnectionListener(connection)).start();
		}
		
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
			System.out.println("RobotiqHandInterface failed to connect at " + address);
			connected = false;
		}
		return connected;
	}
	
	public void initialize()
	{
		if(!this.isConnected())
			this.connect();
		
		try
		{
			status = this.getStatus();
		}
		catch (RobotiqConnectionException e)
		{
//			e.printStackTrace();
			return;
		}
		
		if(((status[GRIPPER_STATUS] & INITIALIZE) == INITIALIZED) || (status[FAULT_STATUS] != NO_FAULT))
		{
			this.reset();
		}
		
		//initialize arrays
		Arrays.fill(position,(byte)0x00);
		Arrays.fill(speed,(byte)0x00);
		Arrays.fill(force,(byte)0x00);
		
		position[FINGER_A] = BASIC_MODE_FULLY_OPEN[FINGER_A];
		speed[FINGER_A] = MAX_SPEED;
		force[FINGER_A] = DEFAULT_FORCE;
		
		position[FINGER_B] = BASIC_MODE_FULLY_OPEN[FINGER_B];
		speed[FINGER_B] = MAX_SPEED;
		force[FINGER_B] = DEFAULT_FORCE;
		
		position[FINGER_C] = BASIC_MODE_FULLY_OPEN[FINGER_C];
		speed[FINGER_C] = MAX_SPEED;
		force[FINGER_C] = DEFAULT_FORCE;
		
		position[SCISSOR] = BASIC_MODE_FULLY_OPEN[SCISSOR];
		speed[SCISSOR] = MAX_SPEED;
		force[SCISSOR] = DEFAULT_FORCE;
		
		initializedStatus = INITIALIZED;
		operationMode = BASIC_MODE;
		commandedStatus = STANDBY;
		fingerControl = INDIVIDUAL_FINGER_CONTROL;
		scissorControl = INDIVIDUAL_SCISSOR_CONTROL;//CONCURRENT_SCISSOR_CONTROL;
		
		sendRequest(SET_REGISTERS,
				REGISTER_START,
				(byte)(initializedStatus | operationMode | commandedStatus),
				(byte)(fingerControl | scissorControl),
				(byte)0x00,	//reserved byte
				(byte)position[FINGER_A],
				(byte)speed[FINGER_A],
				(byte)force[FINGER_A],
				(byte)position[FINGER_B],
				(byte)speed[FINGER_B],
				(byte)force[FINGER_B],
				(byte)position[FINGER_C],
				(byte)speed[FINGER_C],
				(byte)force[FINGER_C],
				(byte)position[SCISSOR],
				(byte)speed[SCISSOR],
				(byte)force[SCISSOR]);
		
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
			try
			{
				status = this.getStatus();
			}
			catch (RobotiqConnectionException e)
			{
				e.printStackTrace();
			}
			errorCount++;
		} while(status == null || (status[GRIPPER_STATUS] & INITIALIZE) != RESET); //check until reset
	}
	
	public void standby()
	{
		commandedStatus = STANDBY;
		
		sendRequest(SET_REGISTERS,
				REGISTER_START,
				(byte)(initializedStatus | operationMode | commandedStatus),
				(byte)(fingerControl | scissorControl));
	}
	
	public void shutdown()
	{
		try
		{
			connection.close();
		}
		catch(IOException e)
		{
			e.printStackTrace();
		}
	}

	public void open()
	{
	   int operationMode = handData.getOperationMode();
	   
	   if(operationMode == RobotiqGraspMode.PINCH_MODE.ordinal())
	   {
	      position[FINGER_A] = PINCH_MODE_FULLY_OPEN[FINGER_A];
	      position[FINGER_B] = PINCH_MODE_FULLY_OPEN[FINGER_B];
	      position[FINGER_C] = PINCH_MODE_FULLY_OPEN[FINGER_C];
	      position[SCISSOR] = PINCH_MODE_FULLY_OPEN[SCISSOR];
	   }
	   else if(operationMode == RobotiqGraspMode.BASIC_MODE.ordinal())
	   {
	      position[FINGER_A] = BASIC_MODE_FULLY_OPEN[FINGER_A];
	      position[FINGER_B] = BASIC_MODE_FULLY_OPEN[FINGER_B];
	      position[FINGER_C] = BASIC_MODE_FULLY_OPEN[FINGER_C];
	      position[SCISSOR] = BASIC_MODE_FULLY_OPEN[SCISSOR];
	   }
	   else if(operationMode == RobotiqGraspMode.WIDE_MODE.ordinal())
	   {
		   position[FINGER_A] = WIDE_MODE_FULLY_OPEN[FINGER_A];
		   position[FINGER_B] = WIDE_MODE_FULLY_OPEN[FINGER_B];
		   position[FINGER_C] = WIDE_MODE_FULLY_OPEN[FINGER_C];
		   position[SCISSOR] = WIDE_MODE_FULLY_OPEN[SCISSOR];
	   }
	   else if(operationMode == RobotiqGraspMode.SCISSOR_MODE.ordinal())
	   {
		   position[FINGER_A] = SCISSOR_MODE_FULLY_OPEN[FINGER_A];
		   position[FINGER_B] = SCISSOR_MODE_FULLY_OPEN[FINGER_B];
		   position[FINGER_C] = SCISSOR_MODE_FULLY_OPEN[FINGER_C];
		   position[SCISSOR] = SCISSOR_MODE_FULLY_OPEN[SCISSOR];
	   }
	   
		speed[FINGER_A] = MAX_SPEED;
		force[FINGER_A] = DEFAULT_FORCE;
		speed[FINGER_B] = MAX_SPEED;
		force[FINGER_B] = DEFAULT_FORCE;
		speed[FINGER_C] = MAX_SPEED;
		force[FINGER_C] = DEFAULT_FORCE;
		speed[SCISSOR] = MAX_SPEED;
		force[SCISSOR] = DEFAULT_FORCE;
	   
		sendMotionRequest();
	}
	
	public void openFingers()
	{
	   //TODO
	}
	
	public void openThumb()
	{
	   //TODO
	}

	public void close()
	{
	   close(1.0, new int[]{FINGER_A, FINGER_B, FINGER_C, SCISSOR});
	}
	
	public void halfClose()
	{
	   close(0.15, new int[]{FINGER_A, FINGER_B, FINGER_C});
	}
	
	private void close(double percent, int[] fingersToClose)
	{
      int operationMode = handData.getOperationMode();
      
//      byte[] graspMode = null;
//      if(operationMode == RobotiqGraspMode.PINCH_MODE.ordinal())
//      {
//         graspMode = PINCH_MODE_FULLY_CLOSED;
//      }
//      else if(operationMode == RobotiqGraspMode.BASIC_MODE.ordinal())
//      {
//         graspMode = BASIC_MODE_FULLY_CLOSED;
//      }
//      else if(operationMode == RobotiqGraspMode.WIDE_MODE.ordinal())
//      {
//         graspMode = WIDE_MODE_FULLY_CLOSED;
//      }
//      else if(operationMode == RobotiqGraspMode.SCISSOR_MODE.ordinal())
//      {
//         graspMode = SCISSOR_MODE_FULLY_CLOSED;
//      }
//      
//      if(graspMode != null)
//      {
//         for(int i = 0; i < fingersToClose.length; i++)
//         {
//            position[fingersToClose[i]] = graspMode[fingersToClose[i]];
//            position[fingersToClose[i]] *= percent == 1.0 ? percent : (1.0 - percent) * 0xFF;
//            speed[fingersToClose[i]] = MAX_SPEED;
//            force[fingersToClose[i]] = DEFAULT_FORCE;
//         }
//         
//         sendMotionRequest();
//      }
      
      if(operationMode == RobotiqGraspMode.PINCH_MODE.ordinal())
      {
         position[FINGER_A] = PINCH_MODE_FULLY_CLOSED[FINGER_A];
         position[FINGER_B] = PINCH_MODE_FULLY_CLOSED[FINGER_B];
         position[FINGER_C] = PINCH_MODE_FULLY_CLOSED[FINGER_C];
         position[SCISSOR] = PINCH_MODE_FULLY_CLOSED[SCISSOR];
      }
      else if(operationMode == RobotiqGraspMode.BASIC_MODE.ordinal())
      {
         position[FINGER_A] = BASIC_MODE_FULLY_CLOSED[FINGER_A];
         position[FINGER_B] = BASIC_MODE_FULLY_CLOSED[FINGER_B];
         position[FINGER_C] = BASIC_MODE_FULLY_CLOSED[FINGER_C];
         position[SCISSOR] = BASIC_MODE_FULLY_CLOSED[SCISSOR];
      }
      else if(operationMode == RobotiqGraspMode.WIDE_MODE.ordinal())
      {
         position[FINGER_A] = WIDE_MODE_FULLY_CLOSED[FINGER_A];
         position[FINGER_B] = WIDE_MODE_FULLY_CLOSED[FINGER_B];
         position[FINGER_C] = WIDE_MODE_FULLY_CLOSED[FINGER_C];
         position[SCISSOR] = WIDE_MODE_FULLY_CLOSED[SCISSOR];
      }
      else if(operationMode == RobotiqGraspMode.SCISSOR_MODE.ordinal())
      {
         position[FINGER_A] = SCISSOR_MODE_FULLY_CLOSED[FINGER_A];
         position[FINGER_B] = SCISSOR_MODE_FULLY_CLOSED[FINGER_B];
         position[FINGER_C] = SCISSOR_MODE_FULLY_CLOSED[FINGER_C];
         position[SCISSOR] = SCISSOR_MODE_FULLY_CLOSED[SCISSOR];
      }

      percent = percent == 1.0 ? percent : (1.0 - percent) * 0xFF;
      
      position[FINGER_A] *= percent;
      position[FINGER_B] *= percent;
      position[FINGER_C] *= percent;
      position[SCISSOR] *= percent;
      
      speed[FINGER_A] = MAX_SPEED;
      force[FINGER_A] = MAX_FORCE;
      speed[FINGER_B] = MAX_SPEED;
      force[FINGER_B] = MAX_FORCE;
      speed[FINGER_C] = MAX_SPEED;
      force[FINGER_C] = MAX_FORCE;
      speed[SCISSOR] = MAX_SPEED;
      force[SCISSOR] = MAX_FORCE;
      
      sendMotionRequest();
	}
	
	public void closeFingers()
	{
	   //TODO
	}
	
	public void closeThumb()
	{
	   //TODO
	}
	
	public void crush()
	{
	   int operationMode = handData.getOperationMode();
      
      if(operationMode == RobotiqGraspMode.PINCH_MODE.ordinal())
      {
         position[FINGER_A] = PINCH_MODE_FULLY_CLOSED[FINGER_A];
         position[FINGER_B] = PINCH_MODE_FULLY_CLOSED[FINGER_B];
         position[FINGER_C] = PINCH_MODE_FULLY_CLOSED[FINGER_C];
      }
      else if(operationMode == RobotiqGraspMode.BASIC_MODE.ordinal())
      {
         position[FINGER_A] = BASIC_MODE_FULLY_CLOSED[FINGER_A];
         position[FINGER_B] = BASIC_MODE_FULLY_CLOSED[FINGER_B];
         position[FINGER_C] = BASIC_MODE_FULLY_CLOSED[FINGER_C];
      }
      else if(operationMode == RobotiqGraspMode.WIDE_MODE.ordinal())
      {
    	  position[FINGER_A] = WIDE_MODE_FULLY_CLOSED[FINGER_A];
    	  position[FINGER_B] = WIDE_MODE_FULLY_CLOSED[FINGER_B];
    	  position[FINGER_C] = WIDE_MODE_FULLY_CLOSED[FINGER_C];
      }
      
      speed[FINGER_A] = MAX_SPEED;
      force[FINGER_A] = MAX_FORCE;
      speed[FINGER_B] = MAX_SPEED;
      force[FINGER_B] = MAX_FORCE;
      speed[FINGER_C] = MAX_SPEED;
      force[FINGER_C] = MAX_FORCE;
      
      sendMotionRequest();
	}
	
	private void commandFingers(double percent, int []fingersToControl, byte[] fingerPositions)
	{
	   
	}

	public void blockDuringMotion()
	{
		do
		{
			ThreadTools.sleep(200);
			try
			{
				status = this.getStatus();
			}
			catch (RobotiqConnectionException e)
			{
				e.printStackTrace();
			}
		}
		while(status == null || (status[GRIPPER_STATUS] & MOTION_STATUS_MASK) == IN_MOTION);
	}
	
	public void stop()
	{
		try
		{
			status = getStatus();
		}
		catch (RobotiqConnectionException e)
		{
			e.printStackTrace();
			return;
		}
		
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
			try
			{
				status = this.getStatus();
			}
			catch (RobotiqConnectionException e)
			{
				e.printStackTrace();
			}
		}
		while(status == null || (status[GRIPPER_STATUS] & INIT_MODE_STATUS_MASK) == CHANGING_MODE);
	}
	
	public void setIndividualFinger(int finger, byte desiredForce, byte desiredPosition, byte desiredSpeed)
	{
		position[finger] = desiredPosition;
		speed[finger] = desiredSpeed;
		force[finger] = desiredForce;
	}
	
	public void velocityControl(double[] desiredSpeed, RobotSide side)
	{
		
		if(side == RobotSide.LEFT)
		{
			double temp = desiredSpeed[FINGER_C];
			desiredSpeed[FINGER_C] = desiredSpeed[FINGER_B];
			desiredSpeed[FINGER_B] = temp;
		}
		
		byte desiredPosition;
		
		for(int finger : fingers)
		{
			desiredSpeed[finger] = -(MAX_SPEED & 0xFF) + (2.0 * desiredSpeed[finger]) * (MAX_SPEED & 0xFF); // assuming range of [0,1] and mapping it to [-255,255]
			desiredPosition = (desiredSpeed[finger] > 0) ? (byte)0x00 : (byte)0xFF;
			force[finger] = MAX_FORCE;
			speed[finger] = (byte)((int)desiredSpeed[finger] & 0xFF);
			position[finger] = desiredPosition;
		}
		
		sendMotionRequest();
	}
	
	public void positionControl(double desiredPosition[], RobotSide side)
	{
		if(side == RobotSide.LEFT)
		{
			double temp = desiredPosition[FINGER_C];
			desiredPosition[FINGER_C] = desiredPosition[FINGER_B];
			desiredPosition[FINGER_B] = temp;
		}
		
		for(int finger : fingers)
		{
//			desiredPosition[finger] = 0x00 + desiredPosition[finger] * 0xFF;
			
			position[finger] = (byte)((int)desiredPosition[finger] & 0xFF);
			force[finger] = MAX_FORCE;
			speed[finger] = MAX_SPEED;
		}
		
		sendMotionRequest();
	}
	
	public void hook(RobotSide side)
	{
		if(side == RobotSide.LEFT)
		{
			setIndividualFinger(FINGER_A, force[FINGER_A], BASIC_MODE_FULLY_CLOSED[FINGER_A], speed[FINGER_A]);
			setIndividualFinger(FINGER_B, force[FINGER_B], BASIC_MODE_FULLY_CLOSED[FINGER_B], speed[FINGER_B]);
			setIndividualFinger(FINGER_C, force[FINGER_C], BASIC_MODE_FULLY_OPEN[FINGER_C], speed[FINGER_C]); //index finger of left Hand
		}
		else
		{
			setIndividualFinger(FINGER_A, force[FINGER_A], BASIC_MODE_FULLY_CLOSED[FINGER_A], speed[FINGER_A]);
			setIndividualFinger(FINGER_B, force[FINGER_B], BASIC_MODE_FULLY_OPEN[FINGER_B], speed[FINGER_B]); //index finger of right Hand
			setIndividualFinger(FINGER_C, force[FINGER_C], BASIC_MODE_FULLY_CLOSED[FINGER_C], speed[FINGER_C]); 
		}
		
		sendMotionRequest();
	}
	
	private float[] positions = new float[4]; //only used in this method and is extracted for efficiency
	public float[] positionStatus()
	{
		try
		{
			status = getStatus();
		}
		catch(RobotiqConnectionException e)
		{
			e.printStackTrace();
			return positions;
		}
		
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
	
	public RobotiqHandSensorData getHandStatus() throws IOException
	{
		int faultCount = 0;
		do
		{
			try
			{
				status = getStatus();
			}
			catch(RobotiqConnectionException e)
			{
				System.out.println("RobotiqHandInterface: Unable to read data from hand at " + address);
				faultCount++;
				ThreadTools.sleep(500);
			}
		}
		while(status == null && faultCount <= 5);
		
		if(status == null || faultCount > 5 || status.length < SCISSOR_CURRENT)
			throw new IOException("Unable to read hand status at " + address);
		
		dataFromHand[0] = (byte) (status[GRIPPER_STATUS] & INITIALIZATON_MASK);
		dataFromHand[1] = (byte) ((status[GRIPPER_STATUS] & OPERATION_MODE_MASK) >> 1);
		dataFromHand[2] = (byte) ((status[GRIPPER_STATUS] & GO_TO_REQUESTED_MASK) >> 3);
		dataFromHand[3] = (byte) ((status[GRIPPER_STATUS] & INIT_MODE_STATUS_MASK) >> 4);
		dataFromHand[4] = (byte) ((status[GRIPPER_STATUS] & MOTION_STATUS_MASK) >> 6);
		dataFromHand[5] = (byte) (status[OBJECT_DETECTION] & OBJECT_DETECTION_A_MASK);
		dataFromHand[6] = (byte) ((status[OBJECT_DETECTION] & OBJECT_DETECTION_B_MASK) >> 2);
		dataFromHand[7] = (byte) ((status[OBJECT_DETECTION] & OBJECT_DETECTION_C_MASK) >> 4);
		dataFromHand[8] = (byte) ((status[OBJECT_DETECTION] & OBJECT_DETECTION_S_MASK) >> 6);
		dataFromHand[9] = status[FAULT_STATUS];
		dataFromHand[10] = status[FINGER_A_REQUESTED_POSITION];
		dataFromHand[11] = status[FINGER_A_POSITION];
		dataFromHand[12] = status[FINGER_A_CURRENT];
		dataFromHand[13] = status[FINGER_B_REQUESTED_POSITION];
		dataFromHand[14] = status[FINGER_B_POSITION];
		dataFromHand[15] = status[FINGER_B_CURRENT];
		dataFromHand[16] = status[FINGER_C_REQUESTED_POSITION];
		dataFromHand[17] = status[FINGER_C_POSITION];
		dataFromHand[18] = status[FINGER_C_CURRENT];
		dataFromHand[19] = status[SCISSOR_REQUESTED_POSITION];
		dataFromHand[20] = status[SCISSOR_POSITION];
		dataFromHand[21] = status[SCISSOR_CURRENT];
		
		handData.update(dataFromHand, isConnected());
		
		return handData;
	}
	
	public boolean isReady()
	{
		try
		{
			status = getStatus();
		}
		catch(RobotiqConnectionException e)
		{
			e.printStackTrace();
			return false;
		}
		
		return (status[GRIPPER_STATUS] & INITIALIZE) == INITIALIZED;
	}
	
	public boolean doneInitializing()
	{
		try
		{
			status = getStatus();
		}
		catch(RobotiqConnectionException e)
		{
			e.printStackTrace();
			return false;
		}
		
		return (status[GRIPPER_STATUS] & INIT_MODE_STATUS_MASK) != COMPLETED;
	}
	
	public boolean isConnected()
	{
		if(connection != null)
		{
		   connected = connection.testConnection();
		}
		
		return connected;
	}
	
	public void doControl()
	{
		if(handData.objectDetected())
			standby();
		else
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
		dataToSend[0] = functionCode;
		dataToSend[1] = (byte)(startRegister >> 8);
		dataToSend[2] = (byte)startRegister;
		if(functionCode == READ_REGISTERS)
		{
			dataToSend[3] = (byte)(dataRegisters[0] >> 8);
			dataToSend[4] = (byte)(dataRegisters[0]);
			dataLength = 5;
		}
		else
		{
			dataToSend[3] = (byte)((dataRegisters.length/2) >> 8);
			dataToSend[4] = (byte)(dataRegisters.length/2);
			dataToSend[5] = (byte)(dataRegisters.length); //number of bytes to follow
			int counter;
			for(counter = 0; counter < dataRegisters.length; counter++)
			{
				dataToSend[counter+6] = dataRegisters[counter];
			}
			dataLength = counter+6;
		}
		
		try
		{
			byte[] dataToSendCopy = new byte[dataLength];
			for(int i = 0; i < dataLength; i++)
			{
				dataToSendCopy[i] = dataToSend[i];
			}
			return connection.transcieve(RobotiqHandParameters.UNIT_ID, dataToSendCopy);
		}
		catch (IOException e)
		{
			connected = false;
			System.err.println("RobotiqHandInterface: Unable to send Modbus request to Robotiq hand");
		}
		catch(NullPointerException e)
		{
			connected = false;
			System.err.println("RobotiqHandInterface: Robotiq connection is null");
		}
		
		return null;
	}
	
	private void sendMotionRequest()
	{
		int dataLength;
		commandedStatus = GO_TO_REQUESTED;
		
		dataToSend[0] = (byte)(initializedStatus | operationMode | commandedStatus); //Whenever sending a motion request, the command hand positions bit (GO_TO_REQUESTED) must be sent
		dataToSend[1] = (byte)(fingerControl | scissorControl);
		dataToSend[2] = 0x00; //reserved byte
		//update finger a/all fingers
		dataToSend[3] = position[FINGER_A];
		dataToSend[4] = speed[FINGER_A];
		dataToSend[5] = force[FINGER_A];
		
		//update finger b
		dataToSend[6] = position[FINGER_B];
		dataToSend[7] = speed[FINGER_B];
		dataToSend[8] = force[FINGER_B];
		
		//update finger c
		dataToSend[9] = position[FINGER_C];
		dataToSend[10] = speed[FINGER_C];
		dataToSend[11] = force[FINGER_C];
		
		//update scissor
		dataToSend[12] = position[SCISSOR];
		dataToSend[13] = speed[SCISSOR];
		dataToSend[14] = force[SCISSOR];
		dataToSend[15] = 0x00; //included to send an even number of registers. (see the sendRequest() warning)
		dataLength = 16;
		
//		System.out.println("Sending:");
//		System.out.println(position[FINGER_A]);
//		System.out.println(position[FINGER_B]);
//		System.out.println(position[FINGER_C]);
//		System.out.println(position[SCISSOR]);

		byte[] dataToSendCopy = new byte[dataLength];
		for(int i = 0; i < dataLength; i++)
		{
			dataToSendCopy[i] = dataToSend[i];
		}
		sendRequest(SET_REGISTERS, REGISTER_START, dataToSendCopy);
	}
	
	private byte[] getStatus() throws RobotiqConnectionException
	{
		status = sendRequest(READ_REGISTERS,
							 REGISTER_START,
							 (byte)0x08);
		
		if(status == null)
		{
			throw new RobotiqConnectionException("Error occurred communicating with Robotiq hand");
		}
		
		return status;
	}
	
	class ConnectionListener implements Runnable
	{
		public ModbusTCPConnection connection;
		
		public ConnectionListener(ModbusTCPConnection connection)
		{
			this.connection = connection;
		}
		
		@Override
		public void run()
		{
			while(true)
			{
				while(!connection.testConnection())
				{
					try
					{
						connection.setupConnectionFields(address, RobotiqHandParameters.PORT);
					}
					catch (IOException e)
					{
						System.out.println("RobotiqHandInterface: lost connection at " + address);
						System.out.println("Attempting to reconnect...");
						initializedStatus = 0x00;
					}
					
					ThreadTools.sleep(200);
				}
				
				ThreadTools.sleep(1000);
			}
		}
	}
	
	class RobotiqConnectionException extends Exception
	{
		private static final long serialVersionUID = -8733823573411561362L;

		public RobotiqConnectionException(String msg)
		{
			super(msg);
		}
	}
}

