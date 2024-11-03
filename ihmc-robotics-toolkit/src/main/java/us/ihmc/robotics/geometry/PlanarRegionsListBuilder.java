package us.ihmc.robotics.geometry;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameRamp3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;

import java.util.Stack;

import static us.ihmc.robotics.geometry.PlanerRegionBuilderTools.*;

public class PlanarRegionsListBuilder
{
   private PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private PoseReferenceFrame placementFrame = new PoseReferenceFrame("placementFrame", ReferenceFrame.getWorldFrame());

   private Stack<Pose3D> placementFrameStack = new Stack<>();

   public PoseReferenceFrame getPlacementFrame()
   {
      return placementFrame;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public void setRegionIds()
   {
      setRegionIds(0);
   }

   public void setRegionIds(int startId)
   {
      PlanerRegionBuilderTools.setRegionsIds(startId, planarRegionsList);
   }

   public void addXYPlaneSquareReferencedAtCenter(double sizeX, double sizeY)
   {
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(sizeX / 2.0, sizeY / 2.0);
      convexPolygon.addVertex(-sizeX / 2.0, sizeY / 2.0);
      convexPolygon.addVertex(-sizeX / 2.0, -sizeY / 2.0);
      convexPolygon.addVertex(sizeX / 2.0, -sizeY / 2.0);
      convexPolygon.update();
      PlanarRegion planarRegion = new PlanarRegion(placementFrame.getTransformToWorldFrame(), convexPolygon);
      planarRegionsList.addPlanarRegion(planarRegion);
   }

   public void addBoxReferencedAtCenter(double sizeX, double sizeY, double sizeZ)
   {
      addBoxReferencedAtCenter(new Box3D(sizeX, sizeY, sizeZ));
   }

   public void addBoxReferencedAtCenter(Box3D box)
   {
      planarRegionsList.addPlanarRegionsList(createRegionsFromBox(new FrameBox3D(placementFrame, box)));
   }

   public void addRampReferencedAtCenter(Ramp3D ramp)
   {
      planarRegionsList.addPlanarRegionsList(createRegionsFromRamp(new FrameRamp3D(placementFrame, ramp)));
   }

   public void addBoxReferencedAtNegativeXYZCorner(double sizeX, double sizeY, double sizeZ)
   {
      addBoxReferencedAtNegativeXYZCorner(new Box3D(sizeX, sizeY, sizeZ));
   }

   public void addBoxReferencedAtNegativeXYZCorner(Box3D box)
   {
      placeWithOffset(new Point3D(box.getSizeX() / 2.0, box.getSizeY() / 2.0, box.getSizeZ() / 2.0), () ->
      {
         addBoxReferencedAtCenter(box);
      });
   }

   public void addBoxReferencedAtNegativeXYZCorner(Ramp3D ramp)
   {
      placeWithOffset(new Point3D(ramp.getSizeX() / 2.0, ramp.getSizeY() / 2.0, ramp.getSizeZ() / 2.0), () ->
      {
         addRampReferencedAtCenter(ramp);
      });
   }

   public void pushOffset(double yaw)
   {
      pushOffset(new AxisAngle(Axis3D.Z, yaw));
   }

   public void pushOffset(Orientation3DReadOnly orientationOffset)
   {
      pushOffset(null, orientationOffset);
   }

   public void pushOffset(Tuple2DReadOnly positionOffset)
   {
      pushOffset(new Point3D(positionOffset), null);
   }

   public void pushOffset(double x, double y)
   {
      pushOffset(new Point3D(x, y, 0.0), null);
   }

   public void pushOffset(double x, double y, double z, Axis3D axis, double angle)
   {
      pushOffset(new Point3D(x, y, z), new AxisAngle(axis, angle));
   }

   public void pushOffset(double x, double y, double z)
   {
      pushOffset(new Point3D(x, y, z), null);
   }

   public void pushOffset(Tuple3DReadOnly positionOffset)
   {
      pushOffset(positionOffset, null);
   }

   public void placeWithOffset(double yaw, Runnable runnable)
   {
      placeWithOffset(new AxisAngle(Axis3D.Z, yaw), runnable);
   }

   public void placeWithOffset(Orientation3DReadOnly orientationOffset, Runnable runnable)
   {
      placeWithOffset(null, orientationOffset, runnable);
   }

   public void placeWithOffset(Tuple2DReadOnly positionOffset, Runnable runnable)
   {
      placeWithOffset(new Point3D(positionOffset), runnable);
   }

   public void placeWithOffset(double x, double y, Runnable runnable)
   {
      placeWithOffset(new Point3D(x, y, 0.0), runnable);
   }

   public void placeWithOffset(double x, double y, double z, Axis3D axis, double angle, Runnable runnable)
   {
      placeWithOffset(new Point3D(x, y, z), new AxisAngle(axis, angle), runnable);
   }

   public void placeWithOffset(double x, double y, double z, Runnable runnable)
   {
      placeWithOffset(new Point3D(x, y, z), runnable);
   }

   public void placeWithOffset(Tuple3DReadOnly positionOffset, Runnable runnable)
   {
      placeWithOffset(positionOffset, null, runnable);
   }

   public void placeWithOffset(Tuple3DReadOnly positionOffset, Orientation3DReadOnly orientationOffset, Runnable userPlacement)
   {
      pushOffset(positionOffset, orientationOffset);
      userPlacement.run();
      popOffset();
   }

   public void pushOffset(Tuple3DReadOnly positionOffset, Orientation3DReadOnly orientationOffset)
   {
      Pose3D originalPlacementFrame = new Pose3D(placementFrame.getTranslation(), placementFrame.getOrientation());
      placementFrameStack.push(originalPlacementFrame);
      Pose3D offsetPose = new Pose3D(originalPlacementFrame);
      if (positionOffset != null)
         offsetPose.appendTranslation(positionOffset);
      if (orientationOffset != null)
         offsetPose.appendRotation(orientationOffset);
      placementFrame.setPoseAndUpdate(offsetPose);
   }

   public void popOffset()
   {
      placementFrame.setPoseAndUpdate(placementFrameStack.pop());
   }

   public void popOffset(int numberOfOffsetsToPop)
   {
      for (int i = 0; i < numberOfOffsetsToPop; i++)
      {
         placementFrame.setPoseAndUpdate(placementFrameStack.pop());
      }
   }

   public void popAllRemainingOffsets()
   {
      while (!placementFrameStack.isEmpty())
      {
         popOffset();

      }
   }
}