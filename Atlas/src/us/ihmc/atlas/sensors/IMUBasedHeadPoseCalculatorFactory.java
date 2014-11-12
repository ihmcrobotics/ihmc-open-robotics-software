package us.ihmc.atlas.sensors;

import javax.vecmath.Vector3d;

import multisense_ros.RawImuData;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.communication.NetworkProcessorControllerStateHandler;
import us.ihmc.communication.packets.sensing.HeadPosePacket;
import us.ihmc.communication.packets.sensing.RawIMUPacket;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class IMUBasedHeadPoseCalculatorFactory {
	
	private IMUBasedHeadPoseCalculatorFactory() 
	{
	}
	
	static IMUBasedHeadPoseCalculator create(NetworkProcessorControllerStateHandler uiCommunicator, ObjectCommunicator scsCommunicator)
	{
		IMUBasedHeadPoseCalculator calculator = new IMUBasedHeadPoseCalculator(uiCommunicator);
		scsCommunicator.attachListener(RawIMUPacket.class, calculator);
		return calculator;
	}

	static IMUBasedHeadPoseCalculator create(NetworkProcessorControllerStateHandler uiCommunicator, RosMainNode rosMainNode)
	{
		
		IMUBasedHeadPoseCalculator calculator = new IMUBasedHeadPoseCalculator(uiCommunicator);
		rosMainNode.attachSubscriber(AtlasSensorInformation.head_imu_acceleration_topic, calculator);
		return calculator;
	}


} 

class IMUBasedHeadPoseCalculator extends AbstractRosTopicSubscriber<multisense_ros.RawImuData> implements ObjectConsumer<RawIMUPacket>
{
	
	NetworkProcessorControllerStateHandler uiCommunicator;
	YoVariableRegistry registry;
	Vector3d lastMeasurement;
	HeadPosePacket headPosePacket=new HeadPosePacket();
	
	public IMUBasedHeadPoseCalculator(NetworkProcessorControllerStateHandler uiCommunicator) {
		super(multisense_ros.RawImuData._TYPE);
		this.uiCommunicator = uiCommunicator;
		registry  = new YoVariableRegistry(getClass().getSimpleName());
		lastMeasurement = new Vector3d();
	}

	private void process(long timestampInNanoSeconds, Vector3d acceleration)
	{
		//TODO: hardcoded from Multisense IMU frame to Head frame, should take fullRobotModel and use the transform there.
		headPosePacket.pitch = Math.acos(acceleration.x / acceleration.length())-Math.PI/2;
		uiCommunicator.sendSerializableObject(headPosePacket);
		lastMeasurement.set(acceleration);
	}
	
	@Override
	public void consumeObject(RawIMUPacket packet) {
		process(packet.timestampInNanoSecond, packet.linearAcceleration);
	}
	

	@Override
	public void onNewMessage(RawImuData message) {
		Vector3d linearAcceleration = new Vector3d();
		linearAcceleration.set(message.getX(), message.getY(), message.getZ());
		process(message.getTimeStamp().totalNsecs(), linearAcceleration);
	}
	
}