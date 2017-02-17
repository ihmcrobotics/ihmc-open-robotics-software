package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.geometry.shapes.Box3d;


public class RotatableCinderBlockTerrainObject extends RotatableBoxTerrainObject
{
   public RotatableCinderBlockTerrainObject(Box3d box, AppearanceDefinition appearance)
   {
      super(box, appearance);
   }

   @Override
   protected void addGraphics()
   {      
      RigidBodyTransform transformCenterConventionToBottomConvention = box.getTransformCopy();
      transformCenterConventionToBottomConvention = TransformTools.transformLocalZ(transformCenterConventionToBottomConvention, -box.getDimension(Direction.Z) / 2.0);

      Vector3D vector = new Vector3D(box.getDimension(Direction.X), box.getDimension(Direction.Y), box.getDimension(Direction.Z));
      
      linkGraphics = new Graphics3DObject();
      linkGraphics.transform(transformCenterConventionToBottomConvention);
      linkGraphics.scale(vector);
      linkGraphics.addModelFile("models/cinderblock1Meter.obj");
   }

}
