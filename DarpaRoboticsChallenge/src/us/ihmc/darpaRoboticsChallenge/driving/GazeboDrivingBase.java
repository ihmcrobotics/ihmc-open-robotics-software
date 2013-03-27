package us.ihmc.darpaRoboticsChallenge.driving;

import com.xuggle.mediatool.IMediaWriter;
import com.xuggle.mediatool.ToolFactory;
import com.xuggle.xuggler.IRational;
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
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;

/**
 * Modified from {@link us.ihmc.darpaRoboticsChallenge.DRCGazeboDrivingInterface}.
 *
 * @author Peter Abeles
 */
public class GazeboDrivingBase extends AbstractNodeMain
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

	private BackgroundVideoExporter leftEyeVideoExporter;
	private BackgroundVideoExporter rightEyeVideoExporter;

	private ConnectedNode connectedNode;
	private long recordingStartTime;

	private double desiredSteering, desiredSpeed, desiredHandBrake, desiredFootBrake;

	public GazeboDrivingBase()
	{
		new RepeatingReleasedEventsFixer().install();

		colorSpace = (COLOR_IMAGE ? ColorSpace.getInstance(ColorSpace.CS_sRGB) : ColorSpace.getInstance(ColorSpace.CS_GRAY));
		colorModel = new ComponentColorModel(colorSpace, false, false, Transparency.OPAQUE, DataBuffer.TYPE_BYTE);

		desiredSpeed = desiredSteering = 0;
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

	private void setUpVehicleStateSubscriberListeners()
	{
		steeringWheelStateSubscriber.addMessageListener(new MessageListener<Float64>()
		{
			int first = 1;

			public void onNewMessage(Float64 message)
			{
				System.out.println("Steering Wheel State (-pi to pi): " + message.getData());
				if (first > 0)
				{
					desiredSteering = message.getData();
					first--;
				}
			}
		});

		handBrakeStateSubscriber.addMessageListener(new MessageListener<Float64>()
		{

			public void onNewMessage(Float64 message)
			{
				System.out.println("Hand Brake state (0.0 to 1.0): " + message.getData());
			}
		});

		gasPedalStateSubscriber.addMessageListener(new MessageListener<Float64>()
		{
			public void onNewMessage(Float64 message)
			{
				System.out.println("Gas Pedal state (0.0 to 1.0): " + message.getData());
			}
		});

		brakePedalStateSubscriber.addMessageListener(new MessageListener<Float64>()
		{
			public void onNewMessage(Float64 message)
			{
				System.out.println("Brake Pedal state (0.0 to 1.0): " + message.getData());
			}
		});
	}


	private void setUpCameraSubscriberListeners()
	{
		leftEyeImageSubscriber.addMessageListener(new MessageListener<Image>()
		{
			public void onNewMessage(Image message)
			{
				System.out.println("left eye  "+message.getHeader().getStamp().totalNsecs());
				leftEyeImage = bufferedImageFromRosMessage(message);
				if (RECORD)
					leftEyeVideoExporter.pushImage(bufferedImageFromRosMessage(message), connectedNode.getCurrentTime().totalNsecs());
			}
		});

		rightEyeImageSubscriber.addMessageListener(new MessageListener<Image>()
		{
			public void onNewMessage(Image message)
			{
				System.out.println("right eye "+message.getHeader().getStamp().totalNsecs());
				rightEyeImage = bufferedImageFromRosMessage(message);
				if (RECORD)
					rightEyeVideoExporter.pushImage(bufferedImageFromRosMessage(message), connectedNode.getCurrentTime().totalNsecs());
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

	protected void stopTurning()
	{
		if (desiredSteering < 0)
		{
			while (desiredSteering != 0)
			{
				desiredSteering += 0.00002;
				if (desiredSteering > 0)
					desiredSteering = 0;

				steeringWheelCommand.setData(desiredSteering);
				steeringWheelCommandPublisher.publish(steeringWheelCommand);
			}
		}

		if (desiredSteering > 0)
		{
			while (desiredSteering != 0)
			{
				desiredSteering -= 0.00002;
				if (desiredSteering < 0)
					desiredSteering = 0;

				steeringWheelCommand.setData(desiredSteering);
				steeringWheelCommandPublisher.publish(steeringWheelCommand);
			}
		}
	}

	protected void processHandBreak( boolean engaged ) {

		double val = engaged ? 1 : 0;
		handBrakeCommand.setData(val);
		handBrakeCommandPublisher.publish(handBrakeCommand);
	}


	protected void processFootBreak( boolean engaged ) {
		desiredFootBrake = (engaged ? 1 : 0);
		brakePedalCommand.setData(desiredFootBrake);
		brakePedalCommandPublisher.publish(brakePedalCommand);
	}

	protected void processLeftTurn()
	{
		desiredSteering += 0.09;
		if (desiredSteering > Math.PI)
			desiredSteering = Math.PI;

		steeringWheelCommand.setData(desiredSteering);
		steeringWheelCommandPublisher.publish(steeringWheelCommand);
	}

	protected void processRightTurn()
	{
		desiredSteering -= 0.09;
		if (desiredSteering < -Math.PI)
			desiredSteering = -Math.PI;

		steeringWheelCommand.setData(desiredSteering);
		steeringWheelCommandPublisher.publish(steeringWheelCommand);
	}

	protected void processAccelerate()
	{
		desiredSpeed = 1;

		gasPedalCommand.setData(desiredSpeed);
		gasPedalCommandPublisher.publish(gasPedalCommand);
	}

	protected void processDecelerate()
	{
		desiredSpeed = 0;

		gasPedalCommand.setData(desiredSpeed);
		gasPedalCommandPublisher.publish(gasPedalCommand);
	}


	private BufferedImage bufferedImageFromRosMessage(Image imageMessage)
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

	public static void main(String[] args)
	{
		try
		{
			RosRun.main(new String[]{"us.ihmc.darpaRoboticsChallenge.DRCGazeboDrivingInterface"});
		}
		catch (Exception e)
		{
			e.printStackTrace();
		}
	}

	public class BackgroundVideoExporter extends Thread
	{
		private ConcurrentLinkedQueue<TimeStampedImage> imageQueue = new ConcurrentLinkedQueue<TimeStampedImage>();
		private String fileName;
		private boolean shouldFinish = false;

		private Semaphore sem = new Semaphore(1, true);

		public BackgroundVideoExporter(String fileName)
		{
			this.fileName = fileName;
		}

		public void run()
		{
			this.processImageQueueToVideo();
		}

		private void processImageQueueToVideo()
		{
			sem.tryAcquire();
			double frameRate = 30.0;
			double playbackRate = 1.0;

			IRational FRAME_RATE = IRational.make((int) frameRate);
			final IMediaWriter writer = ToolFactory.makeWriter(fileName + ".mp4");

			while (imageQueue.isEmpty() && !shouldFinish)
				;

			BufferedImage firstImage = convertBufferedImageToSuitableFormat(imageQueue.poll().getImage());
			writer.addVideoStream(0, 0, FRAME_RATE, firstImage.getWidth(), firstImage.getHeight());

			while (!shouldFinish)
			{
				while (imageQueue.isEmpty() && !shouldFinish)
					;
				if (!shouldFinish)
				{
					TimeStampedImage nextImage = imageQueue.poll();
					double timeSinceStart = nextImage.getTimestamp() - recordingStartTime;
					long timeStamp = new Double((timeSinceStart) * (1.0 / playbackRate)).longValue();
					writer.encodeVideo(0, convertBufferedImageToSuitableFormat(nextImage.getImage()), timeStamp, TimeUnit.NANOSECONDS);
				}
			}
			imageQueue.clear();
			writer.flush();
			writer.close();
			sem.release();
		}

		public void pushImage(BufferedImage image, long timeStamp)
		{
			imageQueue.add(new TimeStampedImage(image, timeStamp));
		}

		private BufferedImage convertBufferedImageToSuitableFormat(BufferedImage image)
		{
			int newWidth = (image.getWidth() / 2) * 2;
			int newHeight = (image.getHeight() / 2) * 2;

			BufferedImage out = new BufferedImage(newWidth, newHeight, BufferedImage.TYPE_3BYTE_BGR);
			out.getGraphics().drawImage(image, 0, 0, newWidth, newHeight, 0, 0, newWidth, newHeight, null);
			return out;
		}

		public void finish()
		{
			shouldFinish = true;
		}

		public boolean isFinished()
		{
			return sem.availablePermits() == 1;
		}

		class TimeStampedImage
		{
			private BufferedImage image;
			private long timeStamp;

			public TimeStampedImage(BufferedImage image, long timeStamp)
			{
				this.image = image;
				this.timeStamp = timeStamp;
			}

			public BufferedImage getImage()
			{
				return image;
			}

			public long getTimestamp()
			{
				return timeStamp;
			}
		}
	}
}
