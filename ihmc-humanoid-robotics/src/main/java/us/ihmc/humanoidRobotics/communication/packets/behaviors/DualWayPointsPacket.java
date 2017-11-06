package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class DualWayPointsPacket extends Packet<DualWayPointsPacket> {
	public int numberOfWayPointsLeft;

	public Point3D[] positionOfWayPointsLeft;
	public Quaternion[] orientationOfWayPointsLeft;

	public int numberOfWayPointsRight;

	public Point3D[] positionOfWayPointsRight;
	public Quaternion[] orientationOfWayPointsRight;

	public double trajectoryTime;

	public DualWayPointsPacket() {

	}

	public DualWayPointsPacket(DualWayPointsPacket other) {
		numberOfWayPointsLeft = other.numberOfWayPointsLeft;
		numberOfWayPointsRight = other.numberOfWayPointsRight;
		trajectoryTime = other.trajectoryTime;

		this.positionOfWayPointsLeft = new Point3D[this.numberOfWayPointsLeft];
		this.orientationOfWayPointsLeft = new Quaternion[this.numberOfWayPointsLeft];

		for (int i = 0; i < this.numberOfWayPointsLeft; i++) {
			this.positionOfWayPointsLeft[i] = new Point3D(other.positionOfWayPointsLeft[i]);
			this.orientationOfWayPointsLeft[i] = new Quaternion(other.orientationOfWayPointsLeft[i]);
		}

		for (int i = 0; i < this.numberOfWayPointsRight; i++) {
			this.positionOfWayPointsRight[i] = new Point3D(other.positionOfWayPointsRight[i]);
			this.orientationOfWayPointsRight[i] = new Quaternion(other.orientationOfWayPointsRight[i]);
		}
	}

	public void setTrajectoryTime(double trajectoryTime) {
		this.trajectoryTime = trajectoryTime;
	}

	public void setWayPoints(RobotSide robotSide, Pose3D[] poseOfWayPoints) {
		if (robotSide == RobotSide.LEFT) {
			this.numberOfWayPointsLeft = poseOfWayPoints.length;

			this.positionOfWayPointsLeft = new Point3D[this.numberOfWayPointsLeft];
			this.orientationOfWayPointsLeft = new Quaternion[this.numberOfWayPointsLeft];

			for (int i = 0; i < this.numberOfWayPointsLeft; i++) {
				this.positionOfWayPointsLeft[i] = new Point3D(poseOfWayPoints[i].getPosition());
				this.orientationOfWayPointsLeft[i] = new Quaternion(poseOfWayPoints[i].getOrientation());
			}
		} else {
			this.numberOfWayPointsRight = poseOfWayPoints.length;

			this.positionOfWayPointsRight = new Point3D[this.numberOfWayPointsRight];
			this.orientationOfWayPointsRight = new Quaternion[this.numberOfWayPointsRight];

			for (int i = 0; i < this.numberOfWayPointsRight; i++) {
				this.positionOfWayPointsRight[i] = new Point3D(poseOfWayPoints[i].getPosition());
				this.orientationOfWayPointsRight[i] = new Quaternion(poseOfWayPoints[i].getOrientation());
			}
		}

		PrintTools.info("check " + numberOfWayPointsLeft + " " + numberOfWayPointsRight);
	}
	
	public void setWayPoints(RobotSide robotSide, ArrayList<Pose3D> poseOfWayPoints) {
		if (robotSide == RobotSide.LEFT) {
			this.numberOfWayPointsLeft = poseOfWayPoints.size();

			this.positionOfWayPointsLeft = new Point3D[this.numberOfWayPointsLeft];
			this.orientationOfWayPointsLeft = new Quaternion[this.numberOfWayPointsLeft];

			for (int i = 0; i < this.numberOfWayPointsLeft; i++) {
				this.positionOfWayPointsLeft[i] = new Point3D(poseOfWayPoints.get(i).getPosition());
				this.orientationOfWayPointsLeft[i] = new Quaternion(poseOfWayPoints.get(i).getOrientation());
			}
		} else {
			this.numberOfWayPointsRight = poseOfWayPoints.size();

			this.positionOfWayPointsRight = new Point3D[this.numberOfWayPointsRight];
			this.orientationOfWayPointsRight = new Quaternion[this.numberOfWayPointsRight];

			for (int i = 0; i < this.numberOfWayPointsRight; i++) {
				this.positionOfWayPointsRight[i] = new Point3D(poseOfWayPoints.get(i).getPosition());
				this.orientationOfWayPointsRight[i] = new Quaternion(poseOfWayPoints.get(i).getOrientation());
			}
		}

		PrintTools.info("check " + numberOfWayPointsLeft + " " + numberOfWayPointsRight);
	}

	public Pose3D getPoseOfWayPoint(RobotSide robotSide, int index) {
		if (robotSide == RobotSide.LEFT)
			return new Pose3D(positionOfWayPointsLeft[index], orientationOfWayPointsLeft[index]);
		else
			return new Pose3D(positionOfWayPointsRight[index], orientationOfWayPointsRight[index]);
	}

	@Override
	public boolean epsilonEquals(DualWayPointsPacket other, double epsilon) {
		if (this.positionOfWayPointsLeft != other.positionOfWayPointsLeft)
			return false;

		if (this.numberOfWayPointsRight != other.numberOfWayPointsRight)
			return false;

		if (this.trajectoryTime != other.trajectoryTime)
			return false;

		if (this.positionOfWayPointsLeft.length != other.positionOfWayPointsLeft.length)
			return false;
		if (this.orientationOfWayPointsLeft.length != other.orientationOfWayPointsLeft.length)
			return false;

		if (this.positionOfWayPointsRight.length != other.positionOfWayPointsRight.length)
			return false;
		if (this.orientationOfWayPointsRight.length != other.orientationOfWayPointsRight.length)
			return false;

		for (int i = 0; i < this.positionOfWayPointsLeft.length; i++) {
			if (!this.positionOfWayPointsLeft[i].epsilonEquals(other.positionOfWayPointsLeft[i], epsilon))
				return false;
			if (!this.orientationOfWayPointsLeft[i].epsilonEquals(other.orientationOfWayPointsLeft[i], epsilon))
				return false;
		}

		for (int i = 0; i < this.positionOfWayPointsRight.length; i++) {
			if (!this.positionOfWayPointsRight[i].epsilonEquals(other.positionOfWayPointsRight[i], epsilon))
				return false;
			if (!this.orientationOfWayPointsRight[i].epsilonEquals(other.orientationOfWayPointsRight[i], epsilon))
				return false;
		}

		if (!MathTools.epsilonEquals(trajectoryTime, other.trajectoryTime, epsilon))
			return false;

		return true;
	}

}
