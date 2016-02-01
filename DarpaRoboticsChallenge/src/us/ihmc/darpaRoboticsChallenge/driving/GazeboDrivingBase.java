package us.ihmc.darpaRoboticsChallenge.driving;

import java.awt.Transparency;
import java.awt.color.ColorSpace;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.ComponentColorModel;
import java.awt.image.DataBuffer;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import rosgraph_msgs.Clock;
import sensor_msgs.CompressedImage;
import std_msgs.Int8;
import us.ihmc.tools.inputDevices.keyboard.linux.RepeatingReleasedEventsFixer;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.utilities.ros.RosTools;

/**
 * Modified from {@link us.ihmc.atlas.AtlasGazeboDrivingInterface}.
 *
 * @author Peter Abeles
 */
public abstract class GazeboDrivingBase extends AbstractNodeMain
{
	private static final boolean COLOR_IMAGE = true;
	private static final String STEREO_NAMESPACE = "/multisense/camera/";
   private static final String IMAGE = "image_raw" + (COLOR_IMAGE ? "/compressed" : "/compressed");

	private static boolean RECORD = false;

   protected Subscriber<CompressedImage> leftEyeImageSubscriber, rightEyeImageSubscriber;
	private BufferedImage leftEyeImage, rightEyeImage;

   protected Subscriber<std_msgs.Float64> steeringWheelStateSubscriber, handBrakeStateSubscriber, gasPedalStateSubscriber, brakePedalStateSubscriber;

   protected Publisher<Int8> directionPublisher;
   Int8 directionCommand;

	protected Publisher<std_msgs.Float64> steeringWheelCommandPublisher, gasPedalCommandPublisher, brakePedalCommandPublisher, handBrakeCommandPublisher;
	private std_msgs.Float64 steeringWheelCommand, gasPedalCommand, brakePedalCommand, handBrakeCommand;

	private Publisher<geometry_msgs.Pose> teleportInToCarPublisher, teleportOutOfCarPublisher;
	private geometry_msgs.Pose teleportInToCarPose, teleportOutOfCarPose;

   protected Subscriber<Clock> timeSubscriber;
   protected final org.ros.message.Time simulationTime = new org.ros.message.Time();

	private ColorSpace colorSpace;
	private ColorModel colorModel;

	private ConnectedNode connectedNode;

	private double desiredHandBrake, desiredFootBrake;

	public GazeboDrivingBase()
	{
		new RepeatingReleasedEventsFixer().install();

		colorSpace = (COLOR_IMAGE ? ColorSpace.getInstance(ColorSpace.CS_sRGB) : ColorSpace.getInstance(ColorSpace.CS_GRAY));
		colorModel = new ComponentColorModel(colorSpace, false, false, Transparency.OPAQUE, DataBuffer.TYPE_BYTE);
	}

	public GraphName getDefaultNodeName()
	{
		return GraphName.of("darpaRoboticsChallenge/DRCGazeboDrivingInterface");
	}

	public void onStart(ConnectedNode connectedNode)
	{
		this.connectedNode = connectedNode;

		setupSubscribers(connectedNode);

		setupPublishers(connectedNode);

		setupMessages();

//      placeAtlasInCar();

		setUpListeners();

	}

	protected abstract void handleStateSteeringWheel( std_msgs.Float64 message );
	protected abstract void handleHandBrakeState(std_msgs.Float64 message);
	protected abstract void handleStateGasPedal( std_msgs.Float64 message );
	protected abstract void handleStateBreakPedal( std_msgs.Float64 message );

	protected abstract void handleImageLeft( CompressedImage message );
	protected abstract void handleImageRight( CompressedImage message );

	private void setUpListeners()
	{
		steeringWheelStateSubscriber.addMessageListener(new MessageListener<std_msgs.Float64>()
		{
			public void onNewMessage(std_msgs.Float64 message)
			{
				handleStateSteeringWheel(message);
			}
		});

		handBrakeStateSubscriber.addMessageListener(new MessageListener<std_msgs.Float64>()
		{

			public void onNewMessage(std_msgs.Float64 message)
			{
				handleHandBrakeState(message);
			}
		});

		gasPedalStateSubscriber.addMessageListener(new MessageListener<std_msgs.Float64>()
		{
			public void onNewMessage(std_msgs.Float64 message)
			{
				handleStateGasPedal(message);
			}
		});

		brakePedalStateSubscriber.addMessageListener(new MessageListener<std_msgs.Float64>()
		{
			public void onNewMessage(std_msgs.Float64 message)
			{
				handleStateBreakPedal(message);
			}
		});

		leftEyeImageSubscriber.addMessageListener(new MessageListener<CompressedImage>()
		{
			public void onNewMessage(CompressedImage message)
			{
				handleImageLeft(message);
			}
		});

		rightEyeImageSubscriber.addMessageListener(new MessageListener<CompressedImage>()
		{
			public void onNewMessage(CompressedImage message)
			{
				handleImageRight(message);
			}
		});


	}

	private void setupPublishers(ConnectedNode connectedNode)
	{
		steeringWheelCommandPublisher = connectedNode.newPublisher("/drc_vehicle/hand_wheel/cmd", std_msgs.Float64._TYPE);
		handBrakeCommandPublisher = connectedNode.newPublisher("/drc_vehicle/hand_brake/cmd", std_msgs.Float64._TYPE);
		gasPedalCommandPublisher = connectedNode.newPublisher("/drc_vehicle/gas_pedal/cmd", std_msgs.Float64._TYPE);
		brakePedalCommandPublisher = connectedNode.newPublisher("/drc_vehicle/brake_pedal/cmd", std_msgs.Float64._TYPE);

		teleportInToCarPublisher = connectedNode.newPublisher("/drc_world/robot_enter_car", geometry_msgs.Pose._TYPE);
		teleportOutOfCarPublisher = connectedNode.newPublisher("/drc_world/robot_exit_car", geometry_msgs.Pose._TYPE);

      directionPublisher = connectedNode.newPublisher("/drc_vehicle/direction/cmd", Int8._TYPE);
	}

	protected void setupSubscribers(ConnectedNode connectedNode)
	{
		leftEyeImageSubscriber = connectedNode.newSubscriber(STEREO_NAMESPACE + "left/" + IMAGE, CompressedImage._TYPE);
		rightEyeImageSubscriber = connectedNode.newSubscriber(STEREO_NAMESPACE + "right/" + IMAGE, CompressedImage._TYPE);

		steeringWheelStateSubscriber = connectedNode.newSubscriber("/drc_vehicle/hand_wheel/state", std_msgs.Float64._TYPE);
		handBrakeStateSubscriber = connectedNode.newSubscriber("/drc_vehicle/hand_brake/state", std_msgs.Float64._TYPE);
		gasPedalStateSubscriber = connectedNode.newSubscriber("/drc_vehicle/gas_pedal/state", std_msgs.Float64._TYPE);
		brakePedalStateSubscriber = connectedNode.newSubscriber("/drc_vehicle/brake_pedal/state", std_msgs.Float64._TYPE);

      timeSubscriber = connectedNode.newSubscriber("/clock",rosgraph_msgs.Clock._TYPE);
	}

   /**
    * Adds a listener for simulation time
    */
   public void startSimulationTime() {
      timeSubscriber.addMessageListener(new MessageListener<Clock>()
      {
         public void onNewMessage(Clock message)
         {
            synchronized ( simulationTime ) {
               simulationTime.secs = message.getClock().secs;
               simulationTime.nsecs = message.getClock().nsecs;
            }
         }
      });
   }

	private void setupMessages()
	{
		setUpSteeringWheelMessage();
		setUpGasPedalMessage();
		setUpBrakePedalMessage();
		setUpHandBrakeMessage();

		setupEnterCarPoseMessage();
		setupExitCarPoseMessage();
	}

   private void setUpDirectionMessage()
   {
      directionCommand = directionPublisher.newMessage();
      directionCommand.setData((byte)1);
   }

	private void setUpHandBrakeMessage()
	{
		handBrakeCommand = handBrakeCommandPublisher.newMessage();
		handBrakeCommand.setData(0);
	}

	private void setUpBrakePedalMessage()
	{
		brakePedalCommand = brakePedalCommandPublisher.newMessage();
		brakePedalCommand.setData(0);
	}

	private void setUpGasPedalMessage()
	{
		gasPedalCommand = gasPedalCommandPublisher.newMessage();
		gasPedalCommand.setData(0);
	}

	private void setUpSteeringWheelMessage()
	{
		steeringWheelCommand = steeringWheelCommandPublisher.newMessage();
		steeringWheelCommand.setData(0);
	}

	private void setupEnterCarPoseMessage()
	{
		teleportInToCarPose = teleportInToCarPublisher.newMessage();

		teleportInToCarPose.getPosition().setX(0);
		teleportInToCarPose.getPosition().setY(0);
		teleportInToCarPose.getPosition().setZ(0);
		teleportInToCarPose.getOrientation().setW(0);
		teleportInToCarPose.getOrientation().setX(0);
		teleportInToCarPose.getOrientation().setY(0);
		teleportInToCarPose.getOrientation().setZ(0);
	}

	private void setupExitCarPoseMessage()
	{
		teleportOutOfCarPose = teleportOutOfCarPublisher.newMessage();

		teleportOutOfCarPose.getPosition().setX(0);
		teleportOutOfCarPose.getPosition().setY(0);
		teleportOutOfCarPose.getPosition().setZ(0);
		teleportOutOfCarPose.getOrientation().setW(0);
		teleportOutOfCarPose.getOrientation().setX(0);
		teleportOutOfCarPose.getOrientation().setY(0);
		teleportOutOfCarPose.getOrientation().setZ(0);
	}

	protected void placeAtlasInCar()
	{
		teleportInToCarPublisher.setLatchMode(true);
		teleportInToCarPublisher.publish(teleportInToCarPose);
		ThreadTools.sleep(3000);
	}

	protected void removeAtlasFromCar()
	{
		teleportOutOfCarPublisher.setLatchMode(true);
		teleportOutOfCarPublisher.publish(teleportOutOfCarPose);
		ThreadTools.sleep(3000);
	}


	protected void processHandBreak( double val ) {
		handBrakeCommand.setData(val);
		handBrakeCommandPublisher.publish(handBrakeCommand);
	}


	protected void processFootBreak( boolean engaged ) {
		desiredFootBrake = (engaged ? 1 : 0);
		brakePedalCommand.setData(desiredFootBrake);
		brakePedalCommandPublisher.publish(brakePedalCommand);
	}

	protected void processSteeringAngle( double angle )
	{
		if (angle > Math.PI)
         angle = Math.PI;
      else if( angle < -Math.PI )
         angle = -Math.PI;

      System.out.println("  requesting steering angle "+angle);
		steeringWheelCommand.setData(angle);
		steeringWheelCommandPublisher.publish(steeringWheelCommand);
	}

	protected void processSpeed( double speed )
	{
		gasPedalCommand.setData(speed);
		gasPedalCommandPublisher.publish(gasPedalCommand);
	}


	protected BufferedImage bufferedImageFromRosMessage(CompressedImage imageMessage)
	{
		return RosTools.bufferedImageFromRosMessageJpeg(colorModel, imageMessage);
	}
}
