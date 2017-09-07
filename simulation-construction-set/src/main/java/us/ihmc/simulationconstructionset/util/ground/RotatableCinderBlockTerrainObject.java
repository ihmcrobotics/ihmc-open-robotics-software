package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;


public class RotatableCinderBlockTerrainObject extends RotatableBoxTerrainObject
{
   public RotatableCinderBlockTerrainObject(Box3D box, AppearanceDefinition appearance)
   {
      super(box, appearance);
   }

   @Override
   protected void addGraphics()
   {      
      RigidBodyTransform transformCenterConventionToBottomConvention = new RigidBodyTransform();
      box.getPose(transformCenterConventionToBottomConvention);
      transformCenterConventionToBottomConvention.appendTranslation(0.0, 0.0, -box.getSizeZ() / 2.0);

      Vector3D vector = new Vector3D(box.getSizeX(), box.getSizeY(), box.getSizeZ());
      
      linkGraphics = new Graphics3DObject();
      linkGraphics.transform(transformCenterConventionToBottomConvention);
      linkGraphics.scale(vector);
      linkGraphics.addModelFile("models/cinderblock1Meter.obj");
   }

}
