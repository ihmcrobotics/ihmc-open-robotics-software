package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;


public class RotatableTableTerrainObject extends RotatableBoxTerrainObject
{
   double tableTopThickness = 1.0;

   public RotatableTableTerrainObject(RigidBodyTransform configuration, double lengthX, double widthY, double heightZ, double tableTopThickness)
   {
      super(configuration, lengthX, widthY, heightZ, YoAppearance.Blue());
      this.tableTopThickness = tableTopThickness;
      addGraphics();
   }

   protected void addGraphics()
   {
      Matrix3d rotation = box.getRotationCopy();
      Point3d center = box.getCenterCopy();
      
      linkGraphics = new Graphics3DObject();
      FrameOrientation orientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), rotation);
      linkGraphics.identity();
      linkGraphics.translate(center.getX(), center.getY(), center.getZ() + box.getDimension(Direction.Z) / 2.0 - tableTopThickness / 2.0);
      linkGraphics.rotate(orientation.getMatrix3dCopy());
      linkGraphics.scale(new Vector3d(box.getDimension(Direction.X), box.getDimension(Direction.Y), tableTopThickness));

      linkGraphics.addModelFile("models/plasticTableTop.obj");
      linkGraphics.identity();
      linkGraphics.translate(center.getX(), center.getY(), center.getZ() + box.getDimension(Direction.Z) / 2.0 - tableTopThickness / 2.0);
      linkGraphics.rotate(orientation.getMatrix3dCopy());
      linkGraphics.scale(new Vector3d(box.getDimension(Direction.X), box.getDimension(Direction.Y), box.getDimension(Direction.Z) - tableTopThickness / 2));
      linkGraphics.addModelFile("models/FoldingTableLegs.obj");

   }


}
