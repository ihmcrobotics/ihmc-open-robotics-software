package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRPickResult;
import us.ihmc.robotics.interaction.MouseCollidable;
import us.ihmc.robotics.interaction.PointCollidable;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

/**
 * A robot collidable is a part of the robot that the user can click on.
 * It supports all the different collision shape types collisions up to
 * date with the correct pose. It can be attached/synced to the robot
 * or "detached" from the robot.
 */
public class RDXRobotCollidable implements RenderableProvider
{
   /** This link frame can be changed by the user to "detach" the collidable from the synced robot link. */
   private ReferenceFrame linkFrame;
   private final MovingReferenceFrame syncedLinkFrame;
   private final MutableReferenceFrame collisionShapeFrame;
   private final MouseCollidable mouseCollidable;
   private final MouseCollidable vrPickRayCollidable;
   private final PointCollidable pointCollidable;
   private final RDXModelInstance collisionModelInstance;
   private final RDXModelInstance collisionShapeCoordinateFrameGraphic;
   private final FrameShape3DBasics shape;
   private final String rigidBodyName;
   private final int collidableIndex;
   private final String name;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private final SideDependentList<RDXVRPickResult> vrPickResult = new SideDependentList<>(RDXVRPickResult::new);
   private boolean isHoveredByAnything = false;
   private boolean isMouseHovering = false;
   private final SideDependentList<Boolean> isVRHovering = new SideDependentList<>(false, false);
   private final SideDependentList<Boolean> isVRPointing = new SideDependentList<>(false, false);
   private final RDXModelInstance pickRayCollisionPointGraphic;
   private boolean ranVRCalculateInput = false;
   private boolean ranVRProcessInput = false;
   private boolean ranMouseCalculateInput = false;
   private boolean ranMouseProcessInput = false;

   public RDXRobotCollidable(us.ihmc.scs2.simulation.collision.Collidable collidable, int collidableIndex, Color color)
   {
      this(collidable.getShape(),
           collidable.getRigidBody().getParentJoint().getFrameAfterJoint(),
           collidable.getRigidBody().getName(),
           collidableIndex,
           color);
   }

   public RDXRobotCollidable(Collidable collidable, int collidableIndex, Color color)
   {
      this(collidable.getShape(),
           collidable.getRigidBody().getParentJoint().getFrameAfterJoint(),
           collidable.getRigidBody().getName(),
           collidableIndex,
           color);
   }

   public RDXRobotCollidable(FrameShape3DReadOnly shape,
                             MovingReferenceFrame syncedLinkFrame,
                             String rigidBodyName,
                             int collidableIndex,
                             Color color)
   {
      // Copy because we will be changing the frame
      // Cast because FixedFrame gets returned and won't let us change the frame.
      this.shape = (FrameShape3DBasics) shape.copy();
      this.syncedLinkFrame = syncedLinkFrame;
      this.rigidBodyName = rigidBodyName;
      this.collidableIndex = collidableIndex;
      name = "%s %d - %s".formatted(rigidBodyName, collidableIndex, shape.getClass().getSimpleName());

      linkFrame = syncedLinkFrame;

      RigidBodyTransform collisionToLinkFrameTransform = new RigidBodyTransform();
      collisionShapeFrame = new MutableReferenceFrame("collisionShapeFrame" + rigidBodyName, linkFrame);

      mouseCollidable = new MouseCollidable(this.shape);
      vrPickRayCollidable = new MouseCollidable(this.shape);
      pointCollidable = new PointCollidable(this.shape);
      for (RobotSide side : RobotSide.values)
         vrPickResult.get(side).setPickedObjectID(this, name);

      collisionModelInstance = new RDXModelInstance(RDXModelBuilder.buildModel(meshBuilder ->
      {
         if (shape instanceof FrameSphere3DReadOnly sphere)
         {
            meshBuilder.addSphere((float) sphere.getRadius(), sphere.getPosition(), color);
         }
         else if (shape instanceof FrameCapsule3DReadOnly capsule)
         {
            Quaternion orientation = new Quaternion();
            EuclidGeometryTools.orientation3DFromZUpToVector3D(capsule.getAxis(), orientation);
            collisionToLinkFrameTransform.appendTranslation(capsule.getPosition());
            collisionToLinkFrameTransform.appendOrientation(orientation);
            meshBuilder.addCapsule(capsule.getLength(),
                                   capsule.getRadius(),
                                   capsule.getRadius(),
                                   capsule.getRadius(),
                                   50,
                                   50,
                                   color);
         }
         else if (shape instanceof FrameBox3DReadOnly box)
         {
            collisionToLinkFrameTransform.appendTranslation(box.getPosition());
            collisionToLinkFrameTransform.appendOrientation(box.getOrientation());
            meshBuilder.addBox(box.getSizeX(),
                               box.getSizeY(),
                               box.getSizeZ(),
                               color);
         }
         else if (shape instanceof FrameRamp3DReadOnly ramp)
         {
            collisionToLinkFrameTransform.getTranslation().addZ(ramp.getSizeZ());
            collisionToLinkFrameTransform.appendTranslation(ramp.getPosition());
            collisionToLinkFrameTransform.appendOrientation(ramp.getOrientation());
            meshBuilder.addWedge(ramp.getSizeX(),
                                 ramp.getSizeY(),
                                 ramp.getSizeZ(),
                                 color);
         }
         else if (shape instanceof FramePointShape3DReadOnly pointShape)
         {
            meshBuilder.addSphere((float) 0.01, pointShape, color);
         }
         else if (shape instanceof FrameCylinder3DReadOnly cylinder)
         {
            Quaternion orientation = new Quaternion();
            collisionToLinkFrameTransform.appendTranslation(cylinder.getPosition());
            EuclidGeometryTools.orientation3DFromZUpToVector3D(cylinder.getAxis(), orientation);
            collisionToLinkFrameTransform.appendOrientation(orientation);
            meshBuilder.addCylinder(cylinder.getLength(), cylinder.getRadius(), new Point3D(0.0, 0.0, -cylinder.getHalfLength()), color);
         }
         else if (shape instanceof FrameEllipsoid3DReadOnly ellipsoid)
         {
            collisionToLinkFrameTransform.appendTranslation(ellipsoid.getPosition());
            collisionToLinkFrameTransform.appendOrientation(ellipsoid.getOrientation());
            meshBuilder.addEllipsoid(ellipsoid.getRadiusX(),
                                     ellipsoid.getRadiusY(),
                                     ellipsoid.getRadiusZ(),
                                     new Point3D(),
                                     color);
         }
         else
         {
            LogTools.warn("Shape not handled: {}", shape);
         }
      }, rigidBodyName));
      LibGDXTools.setOpacity(collisionModelInstance, color.a);

      collisionShapeFrame.update(transformToParent -> transformToParent.set(collisionToLinkFrameTransform));

      collisionShapeCoordinateFrameGraphic = new RDXModelInstance(RDXModelBuilder.createCoordinateFrame(0.15));
      pickRayCollisionPointGraphic = new RDXModelInstance(RDXModelBuilder.createSphere(0.0015f, new Color(Color.WHITE)));
   }

   public void update()
   {
      if (collisionShapeFrame.getReferenceFrame().getParent() != linkFrame)
      {
         collisionShapeFrame.setParentFrame(linkFrame);
      }

      collisionModelInstance.setOpacity(isHoveredByAnything ? 1.0f : 0.4f);
      collisionModelInstance.setTransformToReferenceFrame(collisionShapeFrame.getReferenceFrame());
      collisionShapeCoordinateFrameGraphic.setTransformToReferenceFrame(collisionShapeFrame.getReferenceFrame());

      isHoveredByAnything = false;
      for (RobotSide side : RobotSide.values)
      {
         isVRPointing.set(side, false);
         isVRHovering.set(side, false);
      }
      // The robot collidables are shared by things, so we only want to do
      // these at most once per frame each.
      ranVRCalculateInput = false;
      ranVRProcessInput = false;
      ranMouseCalculateInput = false;
      ranMouseProcessInput = false;
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {
      if (!ranVRCalculateInput)
      {
         ranVRCalculateInput = true;

         for (RobotSide side : RobotSide.values)
         {
            vrContext.getController(side).runIfConnected(controller ->
            {
               if (!controller.anythingElseBeingDragged(this))
               {
                  // The shape has the offsets from link frame built in.
                  // Do the collisions in link frame.
                  shape.setReferenceFrame(linkFrame);

                  boolean isInside = pointCollidable.collide(controller.getPickPointPose().getPosition());
                  LibGDXTools.toLibGDX(pointCollidable.getClosestPointOnSurface(), pickRayCollisionPointGraphic.transform);

                  if (isInside)
                  {
                     vrPickResult.get(side).setHoveringCollsion(controller.getPickPointPose().getPosition(), pointCollidable.getClosestPointOnSurface());
                     pickRayCollisionPointGraphic.setColor(ColorDefinitions.Green());
                     controller.addPickResult(vrPickResult.get(side));
                  }
                  else
                  {
                     pickRayCollisionPointGraphic.setColor(ColorDefinitions.White());

                     // Do the pick ray 2nd so we can only do one
                     Line3DReadOnly pickRay = controller.getPickRay();
                     double collision = vrPickRayCollidable.collide(pickRay, collisionShapeFrame.getReferenceFrame());
                     if (!Double.isNaN(collision))
                     {
                        vrPickResult.get(side).setPointingAtCollision(collision);
                        controller.addPickResult(vrPickResult.get(side));
                     }
                  }
               }
            });
         }
      }
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      if (!ranVRProcessInput)
      {
         ranVRProcessInput = true;

         for (RobotSide side : RobotSide.values)
         {
            boolean pickSelected = vrContext.getController(side).getSelectedPick() == vrPickResult.get(side);
            boolean isHovering = pickSelected && vrPickResult.get(side).isHoveringNotPointingAt();
            boolean isPointing = pickSelected && !isHovering;
            isVRHovering.set(side, isHovering);
            isVRPointing.set(side, isPointing);
            isHoveredByAnything |= isHovering;
            isHoveredByAnything |= isPointing;
         }
      }
   }

   public void calculatePick(ImGui3DViewInput input)
   {
      if (!ranMouseCalculateInput)
      {
         ranMouseCalculateInput = true;

         Line3DReadOnly pickRayInWorld = input.getPickRayInWorld();
         pickResult.reset();
         double collision = mouseCollidable.collide(pickRayInWorld, collisionShapeFrame.getReferenceFrame());
         if (!Double.isNaN(collision))
            pickResult.addPickCollision(collision);

         if (pickResult.getPickCollisionWasAddedSinceReset())
         {
            input.addPickResult(pickResult);
         }
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (!ranMouseProcessInput)
      {
         ranMouseProcessInput = true;

         isMouseHovering = input.getClosestPick() == pickResult;
         isHoveredByAnything |= isMouseHovering;
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      collisionModelInstance.getRenderables(renderables, pool);
      collisionShapeCoordinateFrameGraphic.getRenderables(renderables, pool);
      pickRayCollisionPointGraphic.getRenderables(renderables, pool);
   }

   public void setDetached(ReferenceFrame detachedLinkFrame)
   {
      linkFrame = detachedLinkFrame;
   }

   public void setAttachedToSyncedLink()
   {
      linkFrame = syncedLinkFrame;
   }

   /**
    * isHoveredByAnything is used to find if either VR controller
    * or the mouse is hovering over the object
    * getIsHoveredByAnything is used to highlight the object which is hovered
    * <p>
    * getMouseHovering is used to allow for modifying different parts of robot
    * with the mouse
    * While getVRHovering does the same except with VR Controllers
    */
   public boolean getIsHoveredByAnything()
   {
      return isHoveredByAnything;
   }

   public boolean getMouseHovering()
   {
      return isMouseHovering;
   }

   public boolean getVRPointing(RobotSide side)
   {
      return isVRPointing.get(side);
   }
   public boolean getVRHovering(RobotSide side)
   {
      return isVRHovering.get(side);
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
