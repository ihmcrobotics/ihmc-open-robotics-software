package us.ihmc.darpaRoboticsChallenge.driving;

import org.ros.RosRun;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import sensor_msgs.Image;
import std_msgs.Float64;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.keyboardAndMouse.RepeatingReleasedEventsFixer;

import java.awt.*;
import java.awt.color.ColorSpace;
import java.awt.image.*;
import java.net.URISyntaxException;

/**
 * Modified from {@link us.ihmc.darpaRoboticsChallenge.DRCGazeboDrivingInterface}.
 *
 * @author Peter Abeles
 */
public abstract class GazeboDrivingBase extends AbstractNodeMain
{
	private static final boolean COLOR_IMAGE = true;
	private static final String STEREO_NAMESPACE = "/multisense_sl/camera/";
	private static final String IMAGE = "image_rect" + (COLOR_IMAGE ? "_color" : "");

	private static boolean RECORD = false;

	private Subscriber<Image> leftEyeImageSubscriber, rightEyeImageSubscriber;
	private BufferedImage leftEyeImage, rightEyeImage;

	private Subscriber<Float64> steeringWheelStateSubscriber, handBrakeStateSubscriber, gasPedalStateSubscriber, brakePedalStateSubscriber;

	private Publisher<Float64> steeringWheelCommandPublisher, gasPedalCommandPublisher, brakePedalCommandPublisher, handBrakeCommandPublisher;
	private Float64 steeringWheelCommand, gasPedalCommand, brakePedalCommand, handBrakeCommand;

	private Publisher<geometry_msgs.Pose> teleportInToCarPublisher, teleportOutOfCarPublisher;
	private geometry_msgs.Pose teleportInToCarPose, teleportOutOfCarPose;


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

		setUpCameraSubscriberListeners();

		setUpVehicleStateSubscriberListeners();
	}

	protected abstract void handleStateSteeringWheel( Float64 message );
	protected abstract void handleStateBreak( Float64 message );
	protected abstract void handleStateGasPedal( Float64 message );
	protected abstract void handleStateBreakPedal( Float64 message );

	protected abstract void handleImageLeft( Image message );
	protected abstract void handleImageRight( Image message );

	private void setUpVehicleStateSubscriberListeners()
	{
		steeringWheelStateSubscriber.addMessageListener(new MessageListener<Float64>()
		{
			public void onNewMessage(Float64 message)
			{
				handleStateSteeringWheel(message);
			}
		});

		handBrakeStateSubscriber.addMessageListener(new MessageListener<Float64>()
		{

			public void onNewMessage(Float64 message)
			{
				handleStateBreak(message);
			}
		});

		gasPedalStateSubscriber.addMessageListener(new MessageListener<Float64>()
		{
			public void onNewMessage(Float64 message)
			{
				handleStateGasPedal(message);
			}
		});

		brakePedalStateSubscriber.addMessageListener(new MessageListener<Float64>()
		{
			public void onNewMessage(Float64 message)
			{
				handleStateBreakPedal(message);
			}
		});
	}


	private void setUpCameraSubscriberListeners()
	{
		leftEyeImageSubscriber.addMessageListener(new MessageListener<Image>()
		{
			public void onNewMessage(Image message)
			{
				handleImageLeft(message);
			}
		});

		rightEyeImageSubscriber.addMessageListener(new MessageListener<Image>()
		{
			public void onNewMessage(Image message)
			{
				handleImageRight(message);
			}
		});
	}

	private void setupPublishers(ConnectedNode connectedNode)
	{
		steeringWheelCommandPublisher = connectedNode.newPublisher("/drc_vehicle/hand_wheel/cmd", Float64._TYPE);
		handBrakeCommandPublisher = connectedNode.newPublisher("/drc_vehicle/hand_brake/cmd", Float64._TYPE);
		gasPedalCommandPublisher = connectedNode.newPublisher("/drc_vehicle/gas_pedal/cmd", Float64._TYPE);
		brakePedalCommandPublisher = connectedNode.newPublisher("/drc_vehicle/brake_pedal/cmd", Float64._TYPE);

		teleportInToCarPublisher = connectedNode.newPublisher("/drc_world/robot_enter_car", geometry_msgs.Pose._TYPE);
		teleportOutOfCarPublisher = connectedNode.newPublisher("/drc_world/robot_exit_car", geometry_msgs.Pose._TYPE);
	}

	private void setupSubscribers(ConnectedNode connectedNode)
	{
		leftEyeImageSubscriber = connectedNode.newSubscriber(STEREO_NAMESPACE + "left/" + IMAGE, Image._TYPE);
		rightEyeImageSubscriber = connectedNode.newSubscriber(STEREO_NAMESPACE + "right/" + IMAGE, Image._TYPE);

		steeringWheelStateSubscriber = connectedNode.newSubscriber("/drc_vehicle/hand_wheel/state", Float64._TYPE);
		handBrakeStateSubscriber = connectedNode.newSubscriber("/drc_vehicle/hand_brake/state", Float64._TYPE);
		gasPedalStateSubscriber = connectedNode.newSubscriber("/drc_vehicle/gas_pedal/state", Float64._TYPE);
		brakePedalStateSubscriber = connectedNode.newSubscriber("/drc_vehicle/brake_pedal/state", Float64._TYPE);
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


	protected BufferedImage bufferedImageFromRosMessage(Image imageMessage)
	{
		int width = imageMessage.getWidth();
		int height = imageMessage.getHeight();

		byte[] payload = imageMessage.getData().array();
		DataBuffer dataBuffer = new DataBufferByte(payload, payload.length, imageMessage.getData().arrayOffset());
		SampleModel sampleModel = colorModel.createCompatibleSampleModel(width, height);
		WritableRaster raster = Raster.createWritableRaster(sampleModel, dataBuffer, null);

		int bufferedImageColorType = (COLOR_IMAGE ? BufferedImage.TYPE_INT_BGR : BufferedImage.TYPE_BYTE_GRAY);

		BufferedImage ret = new BufferedImage(width, height, bufferedImageColorType);
		ret.setData(raster);

		return ret;
	}

	public static void main(String[] args) throws URISyntaxException {
		try
		{
			RosRun.main(new String[]{"us.ihmc.darpaRoboticsChallenge.DRCGazeboDrivingInterface"});
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}
	}
}
