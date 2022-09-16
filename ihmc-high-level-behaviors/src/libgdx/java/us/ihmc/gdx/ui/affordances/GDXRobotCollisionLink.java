package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.shape.primitives.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.input.ImGui3DViewPickResult;
import us.ihmc.gdx.tools.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.*;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.gdx.vr.GDXVRPickResult;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXRobotCollisionLink implements RenderableProvider
{
   private final GDXModelInstance collisionModelInstance;
   private final RigidBodyTransform shapeTransformToParentFrameAfterJoint;
   private final ReferenceFrame collisionShapeFrame;
   private final FramePose3D boxPose = new FramePose3D();
   private final RigidBodyTransform boxCenterToWorldTransform = new RigidBodyTransform();
   private final FrameShape3DBasics shape;
   private final MovingReferenceFrame frameAfterJoint;
   private String rigidBodyName;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final GDXVRPickResult vrPickResult = new GDXVRPickResult();
   private SphereRayIntersection sphereRayIntersection;
   private CapsuleRayIntersection capsuleIntersection;
   private CylinderRayIntersection cylinderRayIntersection;
   private EllipsoidRayIntersection ellipsoidRayIntersection;
   private BoxRayIntersection boxRayIntersection;
   private GDXModelInstance collisionShapeCoordinateFrameGraphic;
   private boolean isDetached = false;
   private final RigidBodyTransform detachedTransformToWorld = new RigidBodyTransform();
   private final ReferenceFrame detachedFrameAfterJoint;
   private final ReferenceFrame detachedShapeFrame;
   private boolean desktopPickSelected = false;
   private boolean vrPickSelected = false;
   private boolean pickSelected = false;

   public GDXRobotCollisionLink(us.ihmc.scs2.simulation.collision.Collidable collidable, Color color)
   {
      this(collidable.getShape(),
           collidable.getShape().getReferenceFrame(),
           collidable.getRigidBody().getParentJoint().getFrameAfterJoint(),
           collidable.getRigidBody().getName(),
           color);
   }

   public GDXRobotCollisionLink(Collidable collidable, Color color)
   {
      this(collidable.getShape(),
           collidable.getShape().getReferenceFrame(),
           collidable.getRigidBody().getParentJoint().getFrameAfterJoint(),
           collidable.getRigidBody().getName(),
           color);
   }

   public GDXRobotCollisionLink(FrameShape3DReadOnly shape,
                                ReferenceFrame shapeFrame,
                                MovingReferenceFrame frameAfterJoint,
                                String rigidBodyName,
                                Color color)
   {
      this.shape = (FrameShape3DBasics) shape;
      this.frameAfterJoint = frameAfterJoint;
      this.rigidBodyName = rigidBodyName;
      shapeTransformToParentFrameAfterJoint = new RigidBodyTransform(shapeFrame.getTransformToDesiredFrame(frameAfterJoint));
      collisionShapeFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("collisionShapeFrame" + rigidBodyName,
                                                                                            frameAfterJoint,
                                                                                            shapeTransformToParentFrameAfterJoint);
      detachedFrameAfterJoint = ReferenceFrameTools.constructFrameWithChangingTransformToParent("detachedFrameAfterJoint" + rigidBodyName,
                                                                                                ReferenceFrame.getWorldFrame(),
                                                                                                detachedTransformToWorld);
      detachedShapeFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("detachedShapeFrame" + rigidBodyName,
                                                                                           detachedFrameAfterJoint,
                                                                                           shapeTransformToParentFrameAfterJoint);

      collisionModelInstance = new GDXModelInstance(GDXModelBuilder.buildModel(meshBuilder ->
      {
         if (shape instanceof FrameSphere3DReadOnly sphere)
         {
            meshBuilder.addSphere((float) sphere.getRadius(), sphere.getPosition(), color);
            sphereRayIntersection = new SphereRayIntersection();
         }
         else if (shape instanceof FrameCapsule3DReadOnly capsule)
         {
            Quaternion orientation = new Quaternion();
            EuclidGeometryTools.orientation3DFromZUpToVector3D(capsule.getAxis(), orientation);
            shapeTransformToParentFrameAfterJoint.appendTranslation(capsule.getPosition());
            shapeTransformToParentFrameAfterJoint.appendOrientation(orientation);
            meshBuilder.addCapsule(capsule.getLength(),
                                   capsule.getRadius(),
                                   capsule.getRadius(),
                                   capsule.getRadius(),
                                   50,
                                   50,
                                   color);
            capsuleIntersection = new CapsuleRayIntersection();
         }
         else if (shape instanceof FrameBox3DReadOnly box)
         {
            shapeTransformToParentFrameAfterJoint.appendTranslation(box.getPosition());
            shapeTransformToParentFrameAfterJoint.appendOrientation(box.getOrientation());
            meshBuilder.addBox(box.getSizeX(),
                               box.getSizeY(),
                               box.getSizeZ(),
                               color);
            boxRayIntersection = new BoxRayIntersection();
         }
         else if (shape instanceof FramePointShape3DReadOnly pointShape)
         {
            meshBuilder.addSphere((float) 0.01, pointShape, color);
         }
         else if (shape instanceof FrameCylinder3DReadOnly cylinder)
         {
            Quaternion orientation = new Quaternion();
            shapeTransformToParentFrameAfterJoint.appendTranslation(cylinder.getPosition());
            EuclidGeometryTools.orientation3DFromZUpToVector3D(cylinder.getAxis(), orientation);
            shapeTransformToParentFrameAfterJoint.appendOrientation(orientation);
            meshBuilder.addCylinder(cylinder.getLength(), cylinder.getRadius(), new Point3D(0.0, 0.0, -cylinder.getHalfLength()), color);
            cylinderRayIntersection = new CylinderRayIntersection();
         }
         else if (shape instanceof FrameEllipsoid3DReadOnly ellipsoid)
         {
            shapeTransformToParentFrameAfterJoint.appendTranslation(ellipsoid.getPosition());
            shapeTransformToParentFrameAfterJoint.appendOrientation(ellipsoid.getOrientation());
            meshBuilder.addEllipsoid(ellipsoid.getRadiusX(),
                                     ellipsoid.getRadiusY(),
                                     ellipsoid.getRadiusZ(),
                                     new Point3D(),
                                     color);
            ellipsoidRayIntersection = new EllipsoidRayIntersection();
         }
         else
         {
            LogTools.warn("Shape not handled: {}", shape);
         }
      }, rigidBodyName));
      GDXTools.setTransparency(collisionModelInstance, color.a);

      collisionShapeCoordinateFrameGraphic = new GDXModelInstance(GDXModelBuilder.createCoordinateFrame(0.15));
   }

   public void update()
   {
      if (isDetached)
      {
         detachedFrameAfterJoint.update();
         detachedShapeFrame.update();
         collisionModelInstance.setTransformToReferenceFrame(detachedShapeFrame);
         collisionShapeCoordinateFrameGraphic.setTransformToReferenceFrame(detachedShapeFrame);
      }
      else
      {
         collisionShapeFrame.update();
         collisionModelInstance.setTransformToReferenceFrame(collisionShapeFrame);
         collisionShapeCoordinateFrameGraphic.setTransformToReferenceFrame(collisionShapeFrame);
      }
   }

   public void calculateVRPick(GDXVRContext vrContext)
   {
      vrPickResult.reset();
      ReferenceFrame frameAfterJointToUse = isDetached ? detachedShapeFrame : frameAfterJoint;
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            FramePose3D pickPose = controller.getPickPose();
            pickPose.changeFrame(frameAfterJointToUse);
            try
            {
               shape.setReferenceFrame(frameAfterJointToUse);
               if (shape.isPointInside(pickPose.getPosition()))
               {
                  vrPickResult.addPickCollision(shape.getCentroid().distance(pickPose.getPosition()));
               }
            }
            catch (UnsupportedOperationException unsupportedOperationException)
            {
               unsupportedOperationException.printStackTrace();
            }
            pickPose.changeFrame(ReferenceFrame.getWorldFrame());
         });
      }
      if (vrPickResult.getPickCollisionWasAddedSinceReset())
      {
         vrContext.addPickResult(vrPickResult);
      }
   }

   public void processVRInput(GDXVRContext vrContext)
   {
      vrPickSelected = vrContext.getSelectedPick() == vrPickResult;
   }

   public void calculatePick(ImGui3DViewInput input)
   {
      Line3DReadOnly pickRayInWorld = input.getPickRayInWorld();
      ReferenceFrame frameAfterJointToUse = isDetached ? detachedShapeFrame : frameAfterJoint;
      pickResult.reset();
      if (shape instanceof Sphere3DReadOnly sphere)
      {
         sphereRayIntersection.setup(sphere.getRadius(), sphere.getPosition());
         if (sphereRayIntersection.intersect(input.getPickRayInWorld()))
         {
            pickResult.addPickCollision(input.getPickRayInWorld().getPoint().distance(sphereRayIntersection.getFirstIntersectionToPack()));
         }
      }
      else if (shape instanceof Capsule3DReadOnly capsule)
      {
         UnitVector3DReadOnly axis = capsule.getAxis();
         Point3DReadOnly position = capsule.getPosition();
         double length = capsule.getLength();
         double radius = capsule.getRadius();
         capsuleIntersection.setup(radius, length, position, axis, frameAfterJointToUse);
         if (capsuleIntersection.intersect(pickRayInWorld))
         {
            pickResult.addPickCollision(capsuleIntersection.getDistanceToCollision(input.getPickRayInWorld()));
         }
      }
      else if (shape instanceof Box3DReadOnly box)
      {
         boxPose.setToZero(frameAfterJointToUse);
         if (!isDetached)
            boxPose.set(box.getPose());
         boxPose.changeFrame(ReferenceFrame.getWorldFrame());
         boxPose.get(boxCenterToWorldTransform);
         if (boxRayIntersection.intersect(box.getSizeX(), box.getSizeY(), box.getSizeZ(), boxCenterToWorldTransform, pickRayInWorld))
         {
            pickResult.addPickCollision(boxRayIntersection.getFirstIntersectionToPack().distance(input.getPickRayInWorld().getPoint()));
         }
      }
      else if (shape instanceof PointShape3DReadOnly pointShape)
      {
         // We're not colliding with points as they have no volume
      }
      else if (shape instanceof Cylinder3DReadOnly cylinder)
      {
         cylinderRayIntersection.setup(cylinder.getLength(), cylinder.getRadius(), cylinder.getPosition(), cylinder.getAxis(), frameAfterJointToUse);
         double intersection = cylinderRayIntersection.intersect(input.getPickRayInWorld());
         if (!Double.isNaN(intersection))
         {
            pickResult.addPickCollision(intersection);
         }
      }
      else if (shape instanceof Ellipsoid3DReadOnly ellipsoid)
      {
         ellipsoidRayIntersection.setup(ellipsoid.getRadiusX(),
                                        ellipsoid.getRadiusY(),
                                        ellipsoid.getRadiusZ(),
                                        ellipsoid.getPosition(),
                                        ellipsoid.getOrientation(),
                                        frameAfterJointToUse);
         if (ellipsoidRayIntersection.intersect(input.getPickRayInWorld()))
         {
            pickResult.addPickCollision(ellipsoidRayIntersection.getFirstIntersectionToPack().distance(input.getPickRayInWorld().getPoint()));
         }
      }
      else
      {
         LogTools.warn("Shape not handled: {}", shape);
      }

      if (pickResult.getPickCollisionWasAddedSinceReset())
      {
         input.addPickResult(pickResult);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      desktopPickSelected = input.getClosestPick() == pickResult;

      // TODO: This really needs to be moved to update(), but update really should happen after the inputs
      boolean newPickSelected = vrPickSelected || desktopPickSelected;
      if (newPickSelected != pickSelected)
         GDXTools.setTransparency(collisionModelInstance, pickSelected ? 1.0f : 0.4f);
      pickSelected = newPickSelected;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      collisionModelInstance.getRenderables(renderables, pool);
      collisionShapeCoordinateFrameGraphic.getRenderables(renderables, pool);
   }

   public RigidBodyTransform setDetachedTransform(boolean isDetached)
   {
      this.isDetached = isDetached;
      return detachedTransformToWorld;
   }

   public boolean getPickSelected()
   {
      return pickSelected;
   }

   public String getRigidBodyName()
   {
      return rigidBodyName;
   }

   public FrameShape3DReadOnly getShape()
   {
      return shape;
   }
}
