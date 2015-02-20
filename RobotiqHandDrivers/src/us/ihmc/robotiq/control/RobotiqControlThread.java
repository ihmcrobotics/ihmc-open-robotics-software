package us.ihmc.robotiq.control;

import java.io.IOException;

import us.ihmc.commonWalkingControlModules.packetConsumers.FingerStateProvider;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.communication.packets.manipulation.ManualHandControlPacket;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandJointAngleCommunicator;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.ManualHandControlProvider;
import us.ihmc.robotiq.RobotiqHandInterface;
import us.ihmc.robotiq.data.RobotiqHandSensorData;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.robotSide.RobotSide;

class RobotiqControlThread implements Runnable
	{
      private final RobotSide robotSide;
		private final RobotiqHandInterface robotiqHand;
		private final FingerStateProvider fingerStateProvider;
		private final ManualHandControlProvider manualHandControlProvider;
		private final HandJointAngleCommunicator jointAngleCommunicator;
		private int errorCount = 0;
		private RobotiqHandSensorData handStatus;
		private final PacketCommunicator packetCommunicator;
		
		public RobotiqControlThread(RobotiqControlThreadManager robotiqControlThreadManager, RobotSide robotSide,
		      FingerStateProvider fingerStateProvider, ManualHandControlProvider manualHandControlProvider, PacketCommunicator packetCommunicator)
		{
         this.robotSide = robotSide;
			this.fingerStateProvider = fingerStateProvider;
			this.manualHandControlProvider = manualHandControlProvider;
			robotiqHand = new RobotiqHandInterface(robotSide);
			this.packetCommunicator = packetCommunicator;
			jointAngleCommunicator = new HandJointAngleCommunicator(robotSide, packetCommunicator);
		}

		public void connect()
		{
			robotiqHand.connect();
			ThreadTools.sleep(500);
			while(!robotiqHand.isConnected())
			{
				System.out.println(robotSide.toString() + " Hand is not connected");
			   if(++errorCount > 2)
				{
					System.out.println("Unable to connect " + robotSide.toString() + " Hand");
					return;
				}
				ThreadTools.sleep(2000);
			}
			
			initialize();
		}
		
		public void initialize()
		{
			errorCount = 0;
			int faultCounter = 0;
			do
			{	
				if(++errorCount > 3)
				{
					System.out.println("Unable to initalize " + robotSide.toString() + " Hand. Uncorrectable Fault has occurred");
					return;
				}
				do
				{
					robotiqHand.initialize();
					boolean initialized = true;
					do
					{
						ThreadTools.sleep(100);
						try
						{
							updateHandData();
						}
						catch (IOException e)
						{
							e.printStackTrace();
							initialized = false;
						}
					}
					while(handStatus.isInitializing() && !handStatus.hasError() && !initialized);
					
					if(handStatus.hasError())
					{
						handStatus.printError();
						faultCounter++;
						if(faultCounter < 3)
							robotiqHand.reset();
						else
						{
							break;
						}
					}
					
				}
				while(handStatus.hasCompletedAction());
				
				ThreadTools.sleep(100);
			}
			while(!robotiqHand.isReady());
			
			System.out.println(robotSide.toString() + " Hand Set Up");
		}
		
		private void updateHandData() throws IOException
		{
			handStatus = robotiqHand.getHandStatus();
			jointAngleCommunicator.updateHandAngles(handStatus);
			jointAngleCommunicator.write();
		}
		
		public void run()
		{
			while(packetCommunicator.isConnected())
			{
				if(robotiqHand.isConnected()) //status to UI and keep alive packet
				{
					robotiqHand.doControl();
					
					try
					{
						updateHandData();
					}
					catch (IOException e)
					{
						e.printStackTrace();
						continue;
					}
					
					if(handStatus.hasError())
					   handStatus.printError();

//					System.out.println("Current:");
//					System.out.println(handStatus.getFingerPositions()[0]);
//					System.out.println(handStatus.getFingerPositions()[1]);
//					System.out.println(handStatus.getFingerPositions()[2]);
				}
				
				if(fingerStateProvider.isNewFingerStateAvailable())
				{
					FingerStatePacket packet = fingerStateProvider.pullPacket();
					FingerState state = packet.getFingerState();
					if(!robotiqHand.isConnected())
					{
						if(state.equals(FingerState.CALIBRATE))
						{
							this.initialize();
						}
						else
						{
							System.out.println(robotSide.toString() + " Hand Not Connected");
							continue;
						}
					}
					
					switch(state)
					{
					case CALIBRATE: robotiqHand.initialize(); break;
					case STOP: robotiqHand.stop(); break;
					case OPEN: robotiqHand.open(); break;
					case CLOSE: robotiqHand.close(); break;
					case CRUSH: robotiqHand.crush(); break;
					case HOOK: robotiqHand.hook(robotSide); break;
					case BASIC_GRIP: robotiqHand.normalGrip(); break;
					case PINCH_GRIP: robotiqHand.pinchGrip(); break;
					case WIDE_GRIP: robotiqHand.wideGrip(); break;
					case SCISSOR_GRIP: robotiqHand.scissorGrip(); break;
					case RESET:
						{
							robotiqHand.reset();
							initialize();
						} break;
					default: break;
					}
				}

				if(manualHandControlProvider.isNewPacketAvailable()) // send manual hand control packet to hand
				{
					ManualHandControlPacket packet = manualHandControlProvider.pullPacket();
					if(!robotiqHand.isConnected() || !packet.getRobotSide().equals(robotSide))
						continue;
					if(packet.getControlType() == ManualHandControlPacket.POSITION)
					{
						robotiqHand.positionControl(packet.getCommands(ManualHandControlPacket.HandType.ROBOTIQ), this.robotSide);
					}
					else if(packet.getControlType() == ManualHandControlPacket.VELOCITY)
					{
						robotiqHand.velocityControl(packet.getCommands(ManualHandControlPacket.HandType.ROBOTIQ), this.robotSide);
					}
				}
				
				ThreadTools.sleep(50);
			}
		}
	}