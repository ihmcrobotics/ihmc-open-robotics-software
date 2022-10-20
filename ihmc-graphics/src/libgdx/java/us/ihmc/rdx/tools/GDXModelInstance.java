package us.ihmc.rdx.tools;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.math.collision.BoundingBox;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.definition.visual.ColorDefinition;

public class GDXModelInstance extends ModelInstance
{
   public final Vector3 center = new Vector3();
   public final Vector3 dimensions = new Vector3();
   public float radius;

   private final static BoundingBox bounds = new BoundingBox();

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public GDXModelInstance(Model model)
   {
      super(model);
   }

   public GDXModelInstance(ModelInstance modelInstance)
   {
      super(modelInstance);
   }

   public void calculateBoundingBox()
   {
      calculateBoundingBox(bounds);
      bounds.getCenter(center);
      bounds.getDimensions(dimensions);
      radius = dimensions.len() / 2f;
   }

   public void setTransformToReferenceFrame(ReferenceFrame referenceFrame)
   {
      referenceFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
      GDXTools.toGDX(tempTransform, transform);
   }

   public void setTransformToWorldFrame(RigidBodyTransform transformToWorldFrame)
   {
      GDXTools.toGDX(transformToWorldFrame, transform);
   }

   public void setPoseInWorldFrame(Pose3DReadOnly pose)
   {
      GDXTools.toGDX(pose, tempTransform, transform);
   }

   public void setTransparency(float transparency)
   {
      GDXTools.setTransparency(this, transparency);
   }

   public void setDiffuseColor(Color color)
   {
      GDXTools.setDiffuseColor(this, color);
   }

   public void setColor(ColorDefinition color)
   {
      Color colorGDX = GDXTools.toGDX(color);
      setDiffuseColor(colorGDX);
      if (colorGDX.a < 1.0f)
      {
         setTransparency(colorGDX.a);
      }
   }
}
