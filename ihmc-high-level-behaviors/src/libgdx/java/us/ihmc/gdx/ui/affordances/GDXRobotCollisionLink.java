package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class GDXRobotCollisionLink implements RenderableProvider
{
   private final ReferenceFrame shapeFrame;
   private final ModelInstance modelInstance;
   private final RigidBodyTransform transformToJoint;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ReferenceFrame collisionMeshFrame;

   public GDXRobotCollisionLink(Collidable collidable, Color color)
   {
      Shape3DReadOnly shape = collidable.getShape();
      shapeFrame = collidable.getShape().getReferenceFrame();
      MovingReferenceFrame frameAfterJoint = collidable.getRigidBody().getParentJoint().getFrameAfterJoint();
      // TODO update every frame
      transformToJoint = new RigidBodyTransform(shapeFrame.getTransformToDesiredFrame(frameAfterJoint));
      collisionMeshFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent("collisionMeshFrame" + collidable.getRigidBody().getName(),
                                                                                                  frameAfterJoint,
                                                                                                  transformToJoint);

      modelInstance = GDXModelPrimitives.buildModelInstance(meshBuilder ->
      {
         if (shape instanceof Sphere3DReadOnly)
         {
            Sphere3DReadOnly sphere = (Sphere3DReadOnly) shape;
            meshBuilder.addSphere((float) sphere.getRadius(), sphere.getPosition(), color);
         }
         else if (shape instanceof Capsule3DReadOnly)
         {
            Capsule3DReadOnly capsule = (Capsule3DReadOnly) shape;
            Quaternion orientation = new Quaternion();
            EuclidGeometryTools.orientation3DFromZUpToVector3D(capsule.getAxis(), orientation);
            transformToJoint.appendTranslation(capsule.getPosition());
            transformToJoint.appendOrientation(orientation);
            meshBuilder.addCapsule(capsule.getLength(),
                                   capsule.getRadius(),
                                   capsule.getRadius(),
                                   capsule.getRadius(),
                                   50,
                                   50,
                                   color);
         }
         else if (shape instanceof Box3DReadOnly)
         {
            Box3DReadOnly box = (Box3DReadOnly) shape;
            transformToJoint.appendTranslation(box.getPosition());
            transformToJoint.appendOrientation(box.getOrientation());
            meshBuilder.addBox(box.getSizeX(),
                               box.getSizeY(),
                               box.getSizeZ(),
                               color);
         }
         else if (shape instanceof PointShape3DReadOnly)
         {
            PointShape3DReadOnly pointShape = (PointShape3DReadOnly) shape;
            meshBuilder.addSphere((float) 0.01, pointShape, color);
         }
         else
         {
            LogTools.warn("Shape not handled: {}", shape);
         }
      }, collidable.getRigidBody().getName());
      GDXTools.setTransparency(modelInstance, color.a);
   }

   public void update()
   {
//      tempTransform.set(shapeFrame.getTransformToWorldFrame());
//      transformToJoint.transform(tempTransform);
////      shapeFrame.getTransformToWorldFrame().transform(tempTransform);
//      GDXTools.toGDX(tempTransform, modelInstance.transform);
      collisionMeshFrame.update();
      GDXTools.toGDX(collisionMeshFrame.getTransformToWorldFrame(), modelInstance.transform);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }
}
