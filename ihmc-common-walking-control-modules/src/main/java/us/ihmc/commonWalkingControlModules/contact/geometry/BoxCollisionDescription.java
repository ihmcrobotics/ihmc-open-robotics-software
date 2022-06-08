package us.ihmc.commonWalkingControlModules.contact.geometry;

import us.ihmc.commonWalkingControlModules.contact.geometry.RigidBodyCollisionDescription;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.ArrayList;
import java.util.List;

public class BoxCollisionDescription implements RigidBodyCollisionDescription
{
   private final Collidable collidable;
   private final FrameBox3DReadOnly collidableShape;
   private final List<FramePoint3D> boxCorners = new ArrayList<>();
   private double heightInWorld = Double.POSITIVE_INFINITY;

   private final FramePose3D boxPose = new FramePose3D();
   private final PoseReferenceFrame boxFrame;

   public BoxCollisionDescription(String namePrefix, Collidable collidable)
   {
      checkShape(collidable);

      this.collidable = collidable;
      this.collidableShape = (FrameBox3DReadOnly) collidable.getShape();
      this.boxFrame = new PoseReferenceFrame(namePrefix + "Frame", ReferenceFrame.getWorldFrame());

      for (int i = 0; i < 8; i++)
      {
         boxCorners.add(new FramePoint3D());
      }
   }

   @Override
   public Collidable getCollidable()
   {
      return collidable;
   }

   @Override
   public double updateHeightInWorld()
   {
      updateBoxCorners();
      heightInWorld = Double.POSITIVE_INFINITY;
      for (int i = 0; i < boxCorners.size(); i++)
      {
         boxCorners.get(i).changeFrame(ReferenceFrame.getWorldFrame());
         double height = boxCorners.get(i).getZ();
         if (height < heightInWorld)
         {
            heightInWorld = height;
         }
      }

      return heightInWorld;
   }

   private void updateBoxCorners()
   {
      boxPose.setIncludingFrame(collidableShape.getReferenceFrame(), collidableShape.getPosition(), collidableShape.getOrientation());
      boxPose.changeFrame(ReferenceFrame.getWorldFrame());
      boxFrame.setPoseAndUpdate(boxPose);

      double halfX = 0.5 * collidableShape.getSize().getX();
      double halfY = 0.5 * collidableShape.getSize().getY();
      double halfZ = 0.5 * collidableShape.getSize().getZ();

      boxCorners.get(0).setIncludingFrame(boxFrame, halfX, halfY, halfZ);
      boxCorners.get(1).setIncludingFrame(boxFrame, halfX, halfY, -halfZ);
      boxCorners.get(2).setIncludingFrame(boxFrame, halfX, -halfY, halfZ);
      boxCorners.get(3).setIncludingFrame(boxFrame, halfX, -halfY, -halfZ);
      boxCorners.get(4).setIncludingFrame(boxFrame, -halfX, halfY, halfZ);
      boxCorners.get(5).setIncludingFrame(boxFrame, -halfX, halfY, -halfZ);
      boxCorners.get(6).setIncludingFrame(boxFrame, -halfX, -halfY, halfZ);
      boxCorners.get(7).setIncludingFrame(boxFrame, -halfX, -halfY, -halfZ);
   }

   @Override
   public double getHeightInWorld()
   {
      return heightInWorld;
   }

   @Override
   public void packContactPoints(List<FramePoint3DReadOnly> contactPoints, double heightThreshold)
   {
      for (int i = 0; i < boxCorners.size(); i++)
      {
         if (boxCorners.get(i).getZ() < heightThreshold)
         {
            contactPoints.add(boxCorners.get(i));
         }
      }
   }

   @Override
   public double packContactPoints(List<FramePoint3DReadOnly> contactPoints, double distanceThreshold, PlanarRegion planarRegion)
   {
      updateBoxCorners();
      double closestDistance = Double.POSITIVE_INFINITY;

      for (int i = 0; i < boxCorners.size(); i++)
      {
         FramePoint3D boxCorner = boxCorners.get(i);
         boxCorner.changeFrame(ReferenceFrame.getWorldFrame());
         double distance = PlanarRegionTools.distanceToPlanarRegion(boxCorner, planarRegion);
         if (distance < distanceThreshold)
         {
            contactPoints.add(boxCorner);
         }

         if (distance < closestDistance)
            closestDistance = distance;
      }

      return closestDistance;
   }

   private static void checkShape(Collidable collidable)
   {
      if (!(collidable.getShape() instanceof FrameBox3DReadOnly))
      {
         throw new RuntimeException("BoxCollisionDescription should only be passed in a box-shaped collidable");
      }
   }
}
