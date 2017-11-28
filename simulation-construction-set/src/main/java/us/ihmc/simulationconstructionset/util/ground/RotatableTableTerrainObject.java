package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;


public class RotatableTableTerrainObject extends RotatableBoxTerrainObject
{
   double tableTopThickness = 1.0;

   public RotatableTableTerrainObject(RigidBodyTransform configuration, double lengthX, double widthY, double heightZ, double tableTopThickness)
   {
      super(configuration, lengthX, widthY, heightZ, YoAppearance.Blue());
      this.tableTopThickness = tableTopThickness;
      addGraphics();
   }

   @Override
   protected void addGraphics()
   {
      RotationMatrixReadOnly rotation = box.getOrientation();
      Tuple3DReadOnly center = box.getPosition();
      
      linkGraphics = new Graphics3DObject();
      FrameQuaternion orientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), rotation);
      linkGraphics.identity();
      linkGraphics.translate(center.getX(), center.getY(), center.getZ() + box.getSizeZ() / 2.0 - tableTopThickness / 2.0);
      linkGraphics.rotate(new RotationMatrix(orientation));
      linkGraphics.scale(new Vector3D(box.getSizeX(), box.getSizeY(), tableTopThickness));

      linkGraphics.addModelFile("models/plasticTableTop.obj");
      linkGraphics.identity();
      linkGraphics.translate(center.getX(), center.getY(), center.getZ() + box.getSizeZ() / 2.0 - tableTopThickness / 2.0);
      linkGraphics.rotate(new RotationMatrix(orientation));
      linkGraphics.scale(new Vector3D(box.getSizeX(), box.getSizeY(), box.getSizeZ() - tableTopThickness / 2));
      linkGraphics.addModelFile("models/FoldingTableLegs.obj");

   }


}
