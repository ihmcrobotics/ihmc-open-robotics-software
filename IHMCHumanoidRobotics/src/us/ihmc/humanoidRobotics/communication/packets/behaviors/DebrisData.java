package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;

public class DebrisData extends Packet<DebrisData>
{
	public RigidBodyTransform debrisTransform;
	public Vector3d graspVector;
	public Point3d graspVectorPosition;
	
	public DebrisData(Random random)
	{
	   debrisTransform = RigidBodyTransform.generateRandomTransform(random);
	   graspVector = RandomTools.generateRandomVector(random);
	   double max = Double.MAX_VALUE / 2;
      graspVectorPosition = RandomTools.generateRandomPoint(random, max, max, max);
	}
	
	public DebrisData()
	{
	}
	
	public DebrisData(RigidBodyTransform debrisTransform, Vector3d graspVector, Point3d graspVectorPosition)
	{
		this.debrisTransform = debrisTransform;
		this.graspVector = graspVector;
		this.graspVectorPosition = graspVectorPosition;
	}
	
	public DebrisData(DebrisData debrisData)
	{
		this.debrisTransform = debrisData.getDebrisTransform();
		this.graspVectorPosition = debrisData.getGraspVectorPosition();
		this.graspVector = debrisData.getGraspVector();
	}
	
	public RigidBodyTransform getDebrisTransform()
	{
		return debrisTransform;
	}
	
	public Vector3d getGraspVector()
	{
		return graspVector;
	}
	
	public Point3d getGraspVectorPosition()
	{
		return graspVectorPosition;
	}
	
	public boolean epsilonEquals(DebrisData debrisData, double epsilon)
	{
		boolean transformEquals = debrisTransform.epsilonEquals(debrisData.getDebrisTransform(), epsilon);
		boolean vectorEquals = graspVector.epsilonEquals(debrisData.getGraspVector(), epsilon);
		boolean positionEquals = graspVectorPosition.epsilonEquals(debrisData.getGraspVectorPosition(), epsilon);
		
		return transformEquals && vectorEquals && positionEquals;
	}
}
