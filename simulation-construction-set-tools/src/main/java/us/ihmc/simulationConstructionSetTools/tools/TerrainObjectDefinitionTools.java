package us.ihmc.simulationConstructionSetTools.tools;

import java.util.List;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.conversion.VisualsConversionTools;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPBox3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPCapsule3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPConvexPolytope3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPCylinder3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPRamp3DReadOnly;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.ConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Point3DDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.geometry.STPBox3DDefinition;
import us.ihmc.scs2.definition.geometry.STPCapsule3DDefinition;
import us.ihmc.scs2.definition.geometry.STPConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.STPCylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.STPRamp3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class TerrainObjectDefinitionTools
{
   public static TerrainObjectDefinition toTerrainObjectDefinition(CommonAvatarEnvironmentInterface environment)
   {
      CollidableHelper collidableHelper = new CollidableHelper();
      String robotCollisionName = "robot";
      String terrainCollisionName = "terrain";
      return toTerrainObjectDefinition(environment, collidableHelper, robotCollisionName, terrainCollisionName);
   }

   public static TerrainObjectDefinition toTerrainObjectDefinition(CommonAvatarEnvironmentInterface environment,
                                                                   CollidableHelper collidableHelper,
                                                                   String terrainCollisionMask,
                                                                   String... interactableCollisionGroups)
   {
      return toTerrainObjectDefinition(environment.getTerrainObject3D(), collidableHelper, terrainCollisionMask, interactableCollisionGroups);
   }

   public static TerrainObjectDefinition toTerrainObjectDefinition(TerrainObject3D terrainObject3D,
                                                                   CollidableHelper collidableHelper,
                                                                   String terrainCollisionMask,
                                                                   String... interactableCollisionGroups)
   {
      return toTerrainObjectDefinition(terrainObject3D,
                                       collidableHelper.getCollisionMask(terrainCollisionMask),
                                       collidableHelper.createCollisionGroup(interactableCollisionGroups));
   }

   public static TerrainObjectDefinition toTerrainObjectDefinition(CommonAvatarEnvironmentInterface environment, long terrainCollisionMask, long collisionGroup)
   {
      return toTerrainObjectDefinition(environment.getTerrainObject3D(), terrainCollisionMask, collisionGroup);
   }

   public static TerrainObjectDefinition toTerrainObjectDefinition(TerrainObject3D terrainObject3D, long terrainCollisionMask, long collisionGroup)
   {
      TerrainObjectDefinition output = new TerrainObjectDefinition();
      List<Collidable> collidables = CollidableTools.toCollidables(terrainCollisionMask, collisionGroup, terrainObject3D);

      for (Collidable collidable : collidables)
      {
         output.addCollisionShapeDefinition(toCollisionShapeDefinition(collidable));
      }

      List<VisualDefinition> visualDefinitions = VisualsConversionTools.toVisualDefinitions(terrainObject3D.getLinkGraphics());
      visualDefinitions.forEach(output::addVisualDefinition);

      return output;
   }

   public static CollisionShapeDefinition toCollisionShapeDefinition(Collidable source)
   {
      RigidBodyTransform pose = new RigidBodyTransform();
      GeometryDefinition geometry = null;
      CollisionShapeDefinition output = new CollisionShapeDefinition();
      if (source.getRigidBody() != null)
         output.setName(source.getRigidBody().getName());
      output.setCollisionMask(source.getCollisionMask());
      output.setCollisionGroup(source.getCollisionGroup());
   
      FrameShape3DReadOnly shape = source.getShape();
   
      if (shape instanceof FrameSTPBox3DReadOnly)
      {
         FrameSTPBox3DReadOnly stpBox3D = (FrameSTPBox3DReadOnly) shape;
         STPBox3DDefinition stpGeometry = new STPBox3DDefinition(stpBox3D.getSize());
         stpGeometry.setMargins(stpBox3D.getMinimumMargin(), stpBox3D.getMaximumMargin());
         pose.set(shape.getPose());
         geometry = stpGeometry;
      }
      else if (shape instanceof FrameBox3DReadOnly)
      {
         FrameBox3DReadOnly box3D = (FrameBox3DReadOnly) shape;
         geometry = new Box3DDefinition(box3D.getSize());
         pose.set(shape.getPose());
      }
      else if (shape instanceof FrameSTPCapsule3DReadOnly)
      {
         FrameSTPCapsule3DReadOnly stpCapsule3D = (FrameSTPCapsule3DReadOnly) shape;
         STPCapsule3DDefinition stpGeometry = new STPCapsule3DDefinition(stpCapsule3D.getLength(), stpCapsule3D.getRadius());
         stpGeometry.setMargins(stpCapsule3D.getMinimumMargin(), stpCapsule3D.getMaximumMargin());
         geometry = stpGeometry;
      }
      else if (shape instanceof FrameCapsule3DReadOnly)
      {
         FrameCapsule3DReadOnly capsule3D = (FrameCapsule3DReadOnly) shape;
         geometry = new Capsule3DDefinition(capsule3D.getLength(), capsule3D.getRadius());
         pose.getTranslation().set(capsule3D.getPosition());
         EuclidGeometryTools.orientation3DFromZUpToVector3D(capsule3D.getAxis(), pose.getRotation());
      }
      else if (shape instanceof FrameSTPConvexPolytope3DReadOnly)
      {
         FrameSTPConvexPolytope3DReadOnly stpConvexPolytope3D = (FrameSTPConvexPolytope3DReadOnly) shape;
         STPConvexPolytope3DDefinition stpGeometry = new STPConvexPolytope3DDefinition(stpConvexPolytope3D);
         stpGeometry.setMargins(stpConvexPolytope3D.getMinimumMargin(), stpConvexPolytope3D.getMaximumMargin());
         geometry = stpGeometry;
      }
      else if (shape instanceof FrameConvexPolytope3DReadOnly)
      {
         FrameConvexPolytope3DReadOnly convexPolytope3D = (FrameConvexPolytope3DReadOnly) shape;
         geometry = new ConvexPolytope3DDefinition(convexPolytope3D);
      }
      else if (shape instanceof FrameSTPCylinder3DReadOnly)
      {
         FrameSTPCylinder3DReadOnly stpCylinder3D = (FrameSTPCylinder3DReadOnly) shape;
         STPCylinder3DDefinition stpGeometry = new STPCylinder3DDefinition(stpCylinder3D.getLength(), stpCylinder3D.getRadius());
         stpGeometry.setMargins(stpCylinder3D.getMinimumMargin(), stpCylinder3D.getMaximumMargin());
         geometry = stpGeometry;
      }
      else if (shape instanceof FrameCylinder3DReadOnly)
      {
         FrameCylinder3DReadOnly cylinder3D = (FrameCylinder3DReadOnly) shape;
         geometry = new Cylinder3DDefinition(cylinder3D.getLength(), cylinder3D.getRadius());
         pose.getTranslation().set(cylinder3D.getPosition());
         EuclidGeometryTools.orientation3DFromZUpToVector3D(cylinder3D.getAxis(), pose.getRotation());
      }
      else if (shape instanceof FrameEllipsoid3DReadOnly)
      {
         FrameEllipsoid3DReadOnly ellipsoid3D = (FrameEllipsoid3DReadOnly) shape;
         geometry = new Ellipsoid3DDefinition(ellipsoid3D.getRadii());
         pose.set(ellipsoid3D.getPose());
      }
      else if (shape instanceof FramePointShape3DReadOnly)
      {
         FramePointShape3DReadOnly pointShape3D = (FramePointShape3DReadOnly) shape;
         geometry = new Point3DDefinition(pointShape3D);
      }
      else if (shape instanceof FrameSTPRamp3DReadOnly)
      {
         FrameSTPRamp3DReadOnly stpRamp3D = (FrameSTPRamp3DReadOnly) shape;
         STPRamp3DDefinition stpGeometry = new STPRamp3DDefinition(stpRamp3D.getSize());
         stpGeometry.setMargins(stpRamp3D.getMinimumMargin(), stpRamp3D.getMaximumMargin());
         geometry = stpGeometry;
         pose.set(stpRamp3D.getPose());
         // So the origin is at the center of the bottom face
         pose.appendTranslation(0.5 * stpRamp3D.getSizeX(), 0.0, 0.0);
      }
      else if (shape instanceof FrameRamp3DReadOnly)
      {
         FrameRamp3DReadOnly ramp3D = (FrameRamp3DReadOnly) shape;
         geometry = new Ramp3DDefinition(ramp3D.getSize());
         pose.set(ramp3D.getPose());
         // So the origin is at the center of the bottom face
         pose.appendTranslation(0.5 * ramp3D.getSizeX(), 0.0, 0.0);
      }
      else if (shape instanceof FrameSphere3DReadOnly)
      {
         FrameSphere3DReadOnly sphere3D = (FrameSphere3DReadOnly) shape;
         geometry = new Sphere3DDefinition(sphere3D.getRadius());
         pose.getTranslation().set(sphere3D.getPosition());
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported shape: " + shape);
      }
   
      if (source.getRigidBody() != null)
      {
         RigidBodyTransform additionalTransform = shape.getReferenceFrame()
                                                       .getTransformToDesiredFrame(source.getRigidBody().getParentJoint().getFrameAfterJoint());
         pose.preMultiply(additionalTransform);
      }
   
      output.setOriginPose(pose);
      output.setGeometryDefinition(geometry);
   
      return output;
   }
}
