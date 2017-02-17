package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomTools;

public class DebrisData extends Packet<DebrisData>
{
	public RigidBodyTransform debrisTransform;
	public Vector3D graspVector;
	public Point3D graspVectorPosition;
	
	public DebrisData(Random random)
	{
	   debrisTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
	   graspVector = RandomTools.generateRandomVector(random);
	   double max = Double.MAX_VALUE / 2;
      graspVectorPosition = RandomTools.generateRandomPoint(random, max, max, max);
	}
	
	public DebrisData()
	{
	}
	
	public DebrisData(RigidBodyTransform debrisTransform, Vector3D graspVector, Point3D graspVectorPosition)
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
	
	public Vector3D getGraspVector()
	{
		return graspVector;
	}
	
	public Point3D getGraspVectorPosition()
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
