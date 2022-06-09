package us.ihmc.commonWalkingControlModules.contact.kinematics;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.robotics.physics.Collidable;

import java.util.List;

public interface ContactableRigidBodyDescription
{
   /** Collision shape, note there can be multiple per rigid body */
   Collidable getCollidable();

   /** Updates all of the data below and returns height of lowest contact point */
   double updateHeightInWorld();

   /** Height of lowest contact point */
   double getHeightInWorld();

   /** Pack all contact points below the given height threshold */
   void packContactPoints(List<FramePoint3DReadOnly> contactPoints, double heightThreshold);

   /** Packs all contact points within the given distance threshold. Returns distance of closest point (negative if intersecting) */
   double packContactPoints(List<FramePoint3DReadOnly> contactPoints, double distanceThreshold, ConvexPolytope3DReadOnly contactablePolytope);

   /** Gets bounding box of the shape in world frame */
   BoundingBox3DReadOnly getShapeBoundingBox();
}
