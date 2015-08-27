package us.ihmc.atlas.sensors;

import java.util.LinkedList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import multisense_ros.RawImuData;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.sensing.HeadPosePacket;
import us.ihmc.communication.packets.sensing.HeadPosePacket.MeasurementStatus;
import us.ihmc.communication.packets.sensing.RawIMUPacket;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class IMUBasedHeadPoseCalculatorFactory
{
	private IMUBasedHeadPoseCalculatorFactory()
	{
	}
	
	static IMUBasedHeadPoseCalculator create(PacketCommunicator packetCommunicator, DRCRobotSensorInformation sensorInformation)
	{
		IMUBasedHeadPoseCalculator calculator = new IMUBasedHeadPoseCalculator(packetCommunicator, sensorInformation);
		packetCommunicator.attachListener(RawIMUPacket.class, calculator); 
		return calculator;
	}

	static IMUBasedHeadPoseCalculator create(PacketCommunicator sensorSuitePacketCommunicator, DRCRobotSensorInformation sensorInformation, RosMainNode rosMainNode)
	{
		
		IMUBasedHeadPoseCalculator calculator = new IMUBasedHeadPoseCalculator(sensorSuitePacketCommunicator, sensorInformation);
		rosMainNode.attachSubscriber(AtlasSensorInformation.head_imu_acceleration_topic, calculator);
		boolean USE_REAL_ROBOT_TRANSFORM=true;
		if(USE_REAL_ROBOT_TRANSFORM)
		{
			calculator.setHeadIMUFrameWhenLevel(AtlasSensorInformation.getHeadIMUFramesWhenLevel().get(DRCRobotModel.RobotTarget.REAL_ROBOT));
		}
		return calculator;
	}
}

class RunningStatistics
{
	LinkedList<Vector3d> data;
	Vector3d sum;
	Vector3d squared_sum;
	int windowSize=0;
	public RunningStatistics(int windowSize)
	{
		data = new LinkedList<>();
		sum=new Vector3d();
		squared_sum=new Vector3d();
		this.windowSize =windowSize;
	}
	
	public void update(Vector3d newData)
	{
		sum.add(newData);
		squared_sum.add(new Vector3d(newData.x*newData.x, newData.y*newData.y, newData.z*newData.z));
		data.addLast(newData);

		if(data.size()>windowSize)
		{
                  Vector3d removeData=data.removeFirst();
                  sum.sub(removeData);
                  squared_sum.sub(new Vector3d(removeData.x*removeData.x, removeData.y*removeData.y, removeData.z*removeData.z));
		}
	}
	
	public Vector3d getMean()
	{
		Vector3d mean=new Vector3d();
		mean.x = sum.x/windowSize;
		mean.y = sum.y/windowSize;
		mean.z = sum.z/windowSize;
		return mean;
	}
	
	public Vector3d getStdEv()
	{
		Vector3d stdev = new Vector3d();
		Vector3d mean = getMean();
		stdev.x = Math.sqrt(squared_sum.x/windowSize - mean.x*mean.x);
		stdev.y = Math.sqrt(squared_sum.y/windowSize - mean.y*mean.y);
		stdev.z = Math.sqrt(squared_sum.z/windowSize - mean.z*mean.z);
		return stdev;
	}
}

class IMUBasedHeadPoseCalculator extends AbstractRosTopicSubscriber<multisense_ros.RawImuData> implements PacketConsumer<RawIMUPacket>
{
	PacketCommunicator packetCommunicator;
	HeadPosePacket headPosePacket = new HeadPosePacket();
	ReferenceFrame headIMUFrameWhenLevel;
	RunningStatistics stat = new RunningStatistics(100);

	public IMUBasedHeadPoseCalculator(PacketCommunicator sensorSuitePacketCommunicator, DRCRobotSensorInformation sensorInformation) {
		super(multisense_ros.RawImuData._TYPE);
		this.packetCommunicator = sensorSuitePacketCommunicator;

		headIMUFrameWhenLevel = sensorInformation.getHeadIMUFrameWhenLevel();
	}
	
	public void setHeadIMUFrameWhenLevel(ReferenceFrame frame)
	{
		headIMUFrameWhenLevel = frame;
	}

	private boolean checkMeasurementStability(Vector3d newMeasurement)
	{
		stat.update(newMeasurement);
		Vector3d stdev= stat.getStdEv();
		return stdev.length() < 0.07;
	}

	private void process(long timestampInNanoSeconds, Vector3d rawAcceleration)
	{
			//stability detection
			
			if (checkMeasurementStability(rawAcceleration)) {
				Vector3d gravityInducedAcceleration = new Vector3d(0.0, 0.0, +9.82);
				FrameVector accel = new FrameVector(headIMUFrameWhenLevel,rawAcceleration);
				accel.changeFrame(ReferenceFrame.getWorldFrame());
				double[] eulerAngles = EulerAnglesFromVectors(accel.getVector(), gravityInducedAcceleration);
				headPosePacket.setEulerAngles(eulerAngles);
				headPosePacket.status = MeasurementStatus.STABLE;
				headPosePacket.measuredGravityInWorld.set(rawAcceleration);
			} else {
                headPosePacket.reset();
                headPosePacket.status = MeasurementStatus.UNSTABLE_WAIT;
//                System.out.println("stdev"+stat.getStdEv().length());
			}

		packetCommunicator.send(headPosePacket);
	}
	private static double[] EulerAnglesFromVectors(Vector3d vectorPreTransform, Vector3d vectorPostTransform)
	{
		Vector3d rotationAxis = new Vector3d();
		rotationAxis.cross(vectorPreTransform,vectorPostTransform);
		double angle = Math.acos(vectorPreTransform.dot(vectorPostTransform)/vectorPreTransform.length()/vectorPostTransform.length());
		AxisAngle4d aRotation = new AxisAngle4d(rotationAxis, angle);
		Quat4d qRotation = new Quat4d();
		qRotation.set(aRotation);
		double[] eulerAngles =new double[3];
		RotationFunctions.setYawPitchRollBasedOnQuaternion(eulerAngles, qRotation);
		return eulerAngles;
	}
	
	@Override
	public void receivedPacket(RawIMUPacket packet) {
		process(packet.timestampInNanoSecond, packet.linearAcceleration);
	}
	

	@Override
	public void onNewMessage(RawImuData message) {
		Vector3d linearAcceleration = new Vector3d();
		linearAcceleration.set(message.getX(), message.getY(), message.getZ());
		process(message.getTimeStamp().totalNsecs(), linearAcceleration);
	}
	
}