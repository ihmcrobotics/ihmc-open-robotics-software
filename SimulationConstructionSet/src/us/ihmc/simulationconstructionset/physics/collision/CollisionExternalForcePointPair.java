package us.ihmc.simulationconstructionset.physics.collision;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;

public class CollisionExternalForcePointPair
{
   private final double kpCollision = 10.0; //1000;
   private final double kdCollision = 0.0; //100;
   
   private final ExternalForcePoint ef_collision;
   private final Joint parentJoint;
   private CollisionExternalForcePointPair matchingPair;

   private Vector3d surfaceNormalInLinkFrame = new Vector3d();

   public CollisionExternalForcePointPair(String name, int index, Joint parentJoint, YoVariableRegistry registry)
   {
      this.ef_collision = new ExternalForcePoint("ef_" + name + "_collision" + index, registry);
      this.parentJoint = parentJoint;
      if (parentJoint != null) parentJoint.addExternalForcePoint(ef_collision);
   }

   public ExternalForcePoint getExternalForcePoint()
   {
      return ef_collision;
   }

   public CollisionExternalForcePointPair getMatchingPair()
   {
      return matchingPair;
   }

   public void setMatchingPair(CollisionExternalForcePointPair matchingPair)
   {
      this.matchingPair = matchingPair;
   }
   
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   
   public void setSurfaceNormalInWorld(Vector3d surfaceNormalInWorld)
   {
      surfaceNormalInLinkFrame.set(surfaceNormalInWorld);

      parentJoint.getTransformToWorld(transformToWorld);
      transformToWorld.invert();
      transformToWorld.transform(surfaceNormalInLinkFrame);
   }
   

   public void performSpringDamper()
   {
      ExternalForcePoint matchingExternalForcePoint = matchingPair.getExternalForcePoint();

      Point3d position = new Point3d();
      Vector3d velocity = new Vector3d();
      Point3d matchingPosition = new Point3d();
      Vector3d matchingVelocity = new Vector3d();
 
      ef_collision.getPosition(position);
      ef_collision.getVelocity(velocity);
      
      matchingExternalForcePoint.getPosition(matchingPosition);
      matchingExternalForcePoint.getVelocity(matchingVelocity);
      
      Vector3d positionDifference = new Vector3d();
      Vector3d velocityDifference = new Vector3d();
      
      positionDifference.set(matchingPosition);
      positionDifference.sub(position);
      
      velocityDifference.set(matchingVelocity);
      velocityDifference.sub(velocity);
      
      Vector3d springForce = new Vector3d();
      Vector3d damperForce = new Vector3d();

      springForce.set(positionDifference);
      springForce.scale(kpCollision);
      
      damperForce.set(velocityDifference);
      damperForce.scale(kdCollision);
      
      Vector3d totalForce = new Vector3d();
      totalForce.set(springForce);
      totalForce.add(damperForce);
      
      ef_collision.setForce(totalForce);
      totalForce.negate();
      matchingExternalForcePoint.setForce(totalForce);

   }

   public void setPositionInWorld(Point3d position)
   {
      ef_collision.setOffsetWorld(position.getX(), position.getY(), position.getZ()); // Put the external force points in the right places.      
   }

}
