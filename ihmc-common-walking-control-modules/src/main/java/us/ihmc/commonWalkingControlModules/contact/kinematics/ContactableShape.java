package us.ihmc.commonWalkingControlModules.contact.kinematics;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class ContactableShape
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final FrameVector3DReadOnly zDownWorld = new FrameVector3D(worldFrame, 0.0, 0.0, -1.0);

   private final Collidable collidable;
   private final FramePoint3D contactPoint = new FramePoint3D();
   private final FrameVector3D supportDirection = new FrameVector3D();
   private double heightInWorld = Double.POSITIVE_INFINITY;

   private final ExpandingPolytopeAlgorithm collisionDetector = new ExpandingPolytopeAlgorithm();
   private final EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();
   private final RigidBodyTransform shapeToWorldFrameTransform = new RigidBodyTransform();
   private final RigidBodyTransform worldToShapeFrameTransform = new RigidBodyTransform();

   private final ConvexPolytope3D polytopeInShapeFrame = new ConvexPolytope3D();
   private final BoundingBox3D shapeBoundingBoxInWorld = new BoundingBox3D();

   /* Only used if shape is a box */
   private boolean isABox = false;
   private final List<FramePoint3D> boxCorners = new ArrayList<>();
   private final FramePose3D boxPose;
   private final PoseReferenceFrame boxFrame;

   public ContactableShape(String namePrefix, Collidable collidable)
   {
      this.collidable = collidable;

      if (collidable.getShape() instanceof FrameBox3DReadOnly)
      {
         isABox = true;
         for (int i = 0; i < 8; i++)
         {
            boxCorners.add(new FramePoint3D());
         }

         this.boxPose = new FramePose3D();
         this.boxFrame = new PoseReferenceFrame(namePrefix + "Frame", ReferenceFrame.getWorldFrame());
      }
      else
      {
         boxPose = null;
         boxFrame = null;
      }
   }

   public Collidable getCollidable()
   {
      return collidable;
   }

   public double updateHeightInWorld()
   {
      supportDirection.setIncludingFrame(zDownWorld);
      supportDirection.changeFrame(collidable.getShape().getReferenceFrame());
      collidable.getShape().getSupportingVertex(supportDirection, contactPoint);
      contactPoint.changeFrame(worldFrame);
      heightInWorld = contactPoint.getZ();
      return heightInWorld;
   }

   public double getHeightInWorld()
   {
      return heightInWorld;
   }

   public boolean detectFlatGroundContact(List<FramePoint3DReadOnly> contactPoints, double heightThreshold)
   {
      if (isABox)
      {
         FrameBox3DReadOnly boxShape = (FrameBox3DReadOnly) collidable.getShape();
         boxPose.setIncludingFrame(boxShape.getReferenceFrame(), boxShape.getPosition(), boxShape.getOrientation());
         boxPose.changeFrame(ReferenceFrame.getWorldFrame());
         boxFrame.setPoseAndUpdate(boxPose);

         double halfX = 0.5 * boxShape.getSize().getX();
         double halfY = 0.5 * boxShape.getSize().getY();
         double halfZ = 0.5 * boxShape.getSize().getZ();

         boxCorners.get(0).setIncludingFrame(boxFrame, halfX, halfY, halfZ);
         boxCorners.get(1).setIncludingFrame(boxFrame, halfX, halfY, -halfZ);
         boxCorners.get(2).setIncludingFrame(boxFrame, halfX, -halfY, halfZ);
         boxCorners.get(3).setIncludingFrame(boxFrame, halfX, -halfY, -halfZ);
         boxCorners.get(4).setIncludingFrame(boxFrame, -halfX, halfY, halfZ);
         boxCorners.get(5).setIncludingFrame(boxFrame, -halfX, halfY, -halfZ);
         boxCorners.get(6).setIncludingFrame(boxFrame, -halfX, -halfY, halfZ);
         boxCorners.get(7).setIncludingFrame(boxFrame, -halfX, -halfY, -halfZ);

         boxCorners.forEach(corner -> corner.changeFrame(ReferenceFrame.getWorldFrame()));
         boxCorners.sort(Comparator.comparingDouble(FramePoint3D::getZ));

         int addedContacts = 0;
         for (int i = 0; i < boxCorners.size(); i++)
         {
            if (boxCorners.get(i).getZ() < heightThreshold)
            {
               addedContacts++;
               contactPoints.add(boxCorners.get(i));
            }
            if (addedContacts == 4)
               break;
         }

         return addedContacts > 0;
      }
      else
      {
         boolean isInContact = heightInWorld < heightThreshold;
         if (isInContact)
         {
            contactPoints.add(contactPoint);
         }
         return isInContact;
      }
   }

   public boolean detectPolytopeContact(List<FramePoint3DReadOnly> contactPoints, double distanceThreshold, ConvexPolytope3DReadOnly contactablePolytope)
   {
      // Collision check in shape frame
      FrameShape3DReadOnly collisionShape = collidable.getShape();
      ReferenceFrame collisionShapeFrame = collisionShape.getReferenceFrame();
      collisionShapeFrame.getTransformToDesiredFrame(shapeToWorldFrameTransform, ReferenceFrame.getWorldFrame());
      worldToShapeFrameTransform.setAndInvert(shapeToWorldFrameTransform);

      polytopeInShapeFrame.set(contactablePolytope);
      polytopeInShapeFrame.applyTransform(worldToShapeFrameTransform);

      collisionDetector.evaluateCollision(polytopeInShapeFrame, collisionShape, collisionResult);

      if (collisionResult.areShapesColliding())
      {
         FramePoint3D contactPoint = new FramePoint3D(collisionShapeFrame, collisionResult.getPointOnB());
         contactPoint.changeFrame(ReferenceFrame.getWorldFrame());
         contactPoints.add(contactPoint);
      }

      return collisionResult.areShapesColliding();
   }

   public BoundingBox3DReadOnly getShapeBoundingBox()
   {
      collidable.getShape().getReferenceFrame().update();
      collidable.getShape().getBoundingBox(ReferenceFrame.getWorldFrame(), shapeBoundingBoxInWorld);
      return shapeBoundingBoxInWorld;
   }

   public static int getMaximumNumberOfContactPoints(Collidable collidable)
   {
      if (collidable.getShape() instanceof FrameBox3DReadOnly)
         return 4;
      else
         return 1;
   }
}
