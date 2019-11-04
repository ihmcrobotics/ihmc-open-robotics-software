package us.ihmc.utilities.ros.subscriber;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;

public abstract class RosNavMsgsOdometrySubscriber extends AbstractRosTopicSubscriber<nav_msgs.Odometry> {
	private long timeStamp;
	private String frameID;
	private Vector3D pos;
	private Vector3D linearVelocity;
	private Vector3D angularVelocity;
	private us.ihmc.euclid.tuple4D.Quaternion rot;

	public RosNavMsgsOdometrySubscriber() {
		super(nav_msgs.Odometry._TYPE);
	}

	public synchronized void onNewMessage(nav_msgs.Odometry msg) {
		// get timestamp
		timeStamp = msg.getHeader().getStamp().totalNsecs();

		// get Point
		Pose pose = msg.getPose().getPose();
		Point position = pose.getPosition();
		Double posx = position.getX();
		Double posy = position.getY();
		Double posz = position.getZ();

		pos = new Vector3D(posx, posy, posz);

		// get Rotation
		Quaternion orientation = pose.getOrientation();
		Double rotx = orientation.getX();
		Double roty = orientation.getY();
		Double rotz = orientation.getZ();
		Double rotw = orientation.getW();

		rot = new us.ihmc.euclid.tuple4D.Quaternion(rotx, roty, rotz, rotw);

		// get frameID
		frameID = msg.getHeader().getFrameId();

		TimeStampedTransform3D transform = new TimeStampedTransform3D();
		transform.getTransform3D().setTranslation(pos);
		transform.getTransform3D().setRotation(rot);
		transform.setTimeStamp(timeStamp);

		newPose(frameID, transform);

		// get linear velocity
		linearVelocity = new Vector3D(msg.getTwist().getTwist().getLinear().getX(),
				msg.getTwist().getTwist().getLinear().getY(), msg.getTwist().getTwist().getLinear().getZ());

		// get angular velocity
		angularVelocity = new Vector3D(msg.getTwist().getTwist().getAngular().getX(),
				msg.getTwist().getTwist().getAngular().getY(), msg.getTwist().getTwist().getAngular().getZ());

	}

	public synchronized Vector3D getPoint() {
		return pos;
	}

	public synchronized us.ihmc.euclid.tuple4D.Quaternion getRotation() {
		return rot;
	}

	public synchronized long getTimestamp() {
		return timeStamp;
	}

	public synchronized String getFrameID() {
		return frameID;
	}

	public Vector3D getLinearVelocity() {
		return linearVelocity;
	}

	public Vector3D getAngularVelocity() {
		return angularVelocity;
	}

	protected abstract void newPose(String frameID, TimeStampedTransform3D transform);

}
