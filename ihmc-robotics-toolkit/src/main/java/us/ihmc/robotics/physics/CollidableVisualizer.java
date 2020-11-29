package us.ihmc.robotics.physics;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class CollidableVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Collidable collidable;
   private final Shape3DGraphicUpdater shape3DGraphicUpdater;

   public CollidableVisualizer(String name, String groupName, Collidable collidable, AppearanceDefinition appearanceDefinition, YoRegistry registry,
                               YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.collidable = collidable;

      if (collidable.getShape() instanceof FrameSphere3DReadOnly)
      {
         shape3DGraphicUpdater = new Sphere3DGraphicUpdater(name, (FrameSphere3DReadOnly) collidable.getShape(), registry, appearanceDefinition);
      }
      else if (collidable.getShape() instanceof FrameBox3DReadOnly)
      {
         shape3DGraphicUpdater = new Box3DGraphicUpdater(name, (FrameBox3DReadOnly) collidable.getShape(), registry, appearanceDefinition);
      }
      else if (collidable.getShape() instanceof FrameCapsule3DReadOnly)
      {
         shape3DGraphicUpdater = new Capsule3DGraphicUpdater(name, (FrameCapsule3DReadOnly) collidable.getShape(), registry, appearanceDefinition);
      }
      else if (collidable.getShape() instanceof FrameCylinder3DReadOnly)
      {
         shape3DGraphicUpdater = new Cylinder3DGraphicUpdater(name, (FrameCylinder3DReadOnly) collidable.getShape(), registry, appearanceDefinition);
      }
      else if (collidable.getShape() instanceof FramePointShape3DReadOnly)
      {
         shape3DGraphicUpdater = new Point3DGraphicUpdater(name, (FramePointShape3DReadOnly) collidable.getShape(), registry, appearanceDefinition);
      }
      else if (collidable.getShape() instanceof FrameConvexPolytope3DReadOnly)
      {
         shape3DGraphicUpdater = new ConvexPolytope3DGraphicUpdater(name,
                                                                    (FrameConvexPolytope3DReadOnly) collidable.getShape(),
                                                                    registry,
                                                                    appearanceDefinition);
      }
      else if (collidable.getShape() instanceof FrameEllipsoid3DReadOnly)
      {
         shape3DGraphicUpdater = new EllipsoidGraphicUpdater(name, (FrameEllipsoid3DReadOnly) collidable.getShape(), registry, appearanceDefinition);
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported type of shape: " + collidable.getShape().getClass().getSimpleName());
      }

      hide();
      yoGraphicsListRegistry.registerYoGraphic(groupName, shape3DGraphicUpdater.getYoGraphic());
   }

   public void hide()
   {
      shape3DGraphicUpdater.hide();
   }

   public void update()
   {
      shape3DGraphicUpdater.update();
   }

   public Collidable getCollidable()
   {
      return collidable;
   }

   private static interface Shape3DGraphicUpdater
   {
      void hide();

      void update();

      YoGraphic getYoGraphic();
   }

   private static class Sphere3DGraphicUpdater implements Shape3DGraphicUpdater
   {
      private final YoFramePoint3D position;
      private final YoGraphicPosition graphicSphere;
      private final FrameSphere3DReadOnly shape;

      public Sphere3DGraphicUpdater(String name, FrameSphere3DReadOnly shape, YoRegistry registry, AppearanceDefinition appearanceDefinition)
      {
         this.shape = shape;
         position = new YoFramePoint3D(name, worldFrame, registry);
         double radius = shape.getRadius();
         graphicSphere = new YoGraphicPosition(name, position, radius, appearanceDefinition);
      }

      @Override
      public void hide()
      {
         position.setToNaN();
      }

      FramePoint3D tempPoint = new FramePoint3D();

      @Override
      public void update()
      {
         tempPoint.setIncludingFrame(shape.getPosition());
         position.setMatchingFrame(tempPoint);
      }

      @Override
      public YoGraphic getYoGraphic()
      {
         return graphicSphere;
      }
   }

   private static class Box3DGraphicUpdater implements Shape3DGraphicUpdater
   {
      private final YoFramePoseUsingYawPitchRoll pose;
      private final YoGraphicShape graphicBox;
      private final FrameBox3DReadOnly shape;

      public Box3DGraphicUpdater(String name, FrameBox3DReadOnly shape, YoRegistry registry, AppearanceDefinition appearanceDefinition)
      {
         this.shape = shape;
         pose = new YoFramePoseUsingYawPitchRoll(name, worldFrame, registry);
         Graphics3DObject boxGraphicDefinition = new Graphics3DObject();
         boxGraphicDefinition.addCube(shape.getSizeX(), shape.getSizeY(), shape.getSizeZ(), true, appearanceDefinition);
         graphicBox = new YoGraphicShape(name, boxGraphicDefinition, pose, 1.0);
      }

      @Override
      public void hide()
      {
         pose.setToNaN();
      }

      private final FramePose3D tempPose = new FramePose3D();

      @Override
      public void update()
      {
         tempPose.setIncludingFrame(shape.getReferenceFrame(), shape.getPose());
         pose.setMatchingFrame(tempPose);
      }

      @Override
      public YoGraphic getYoGraphic()
      {
         return graphicBox;
      }
   }

   private static class Capsule3DGraphicUpdater implements Shape3DGraphicUpdater
   {
      private final YoFramePoseUsingYawPitchRoll pose;
      private final YoGraphicShape graphicCapsule;
      private final FrameCapsule3DReadOnly shape;

      public Capsule3DGraphicUpdater(String name, FrameCapsule3DReadOnly shape, YoRegistry registry, AppearanceDefinition appearanceDefinition)
      {
         this.shape = shape;
         pose = new YoFramePoseUsingYawPitchRoll(name, worldFrame, registry);
         Graphics3DObject capsuleGraphicDefinition = new Graphics3DObject();
         double radius = shape.getRadius();
         double length = shape.getLength();
         capsuleGraphicDefinition.addCapsule(radius, length + 2.0 * radius, appearanceDefinition);
         graphicCapsule = new YoGraphicShape(name, capsuleGraphicDefinition, pose, 1.0);
      }

      @Override
      public void hide()
      {
         pose.setToNaN();
      }

      private final FramePose3D tempPose = new FramePose3D();

      @Override
      public void update()
      {
         tempPose.setReferenceFrame(shape.getReferenceFrame());
         tempPose.getPosition().set(shape.getPosition());
         EuclidGeometryTools.orientation3DFromZUpToVector3D(shape.getAxis(), tempPose.getOrientation());
         pose.setMatchingFrame(tempPose);
      }

      @Override
      public YoGraphic getYoGraphic()
      {
         return graphicCapsule;
      }
   }

   private static class Cylinder3DGraphicUpdater implements Shape3DGraphicUpdater
   {
      private final YoFramePoseUsingYawPitchRoll pose;
      private final YoGraphicShape graphicCylinder;
      private final FrameCylinder3DReadOnly shape;

      public Cylinder3DGraphicUpdater(String name, FrameCylinder3DReadOnly shape, YoRegistry registry, AppearanceDefinition appearanceDefinition)
      {
         this.shape = shape;
         pose = new YoFramePoseUsingYawPitchRoll(name, worldFrame, registry);
         Graphics3DObject cylinderGraphicDefinition = new Graphics3DObject();
         double radius = shape.getRadius();
         double length = shape.getLength();
         cylinderGraphicDefinition.translate(0.0, 0.0, -0.5 * length);
         cylinderGraphicDefinition.addCylinder(length, radius, appearanceDefinition);
         graphicCylinder = new YoGraphicShape(name, cylinderGraphicDefinition, pose, 1.0);
      }

      @Override
      public void hide()
      {
         pose.setToNaN();
      }

      private final FramePose3D tempPose = new FramePose3D();

      @Override
      public void update()
      {
         tempPose.setReferenceFrame(shape.getReferenceFrame());
         tempPose.getPosition().set(shape.getPosition());
         EuclidGeometryTools.orientation3DFromZUpToVector3D(shape.getAxis(), tempPose.getOrientation());
         pose.setMatchingFrame(tempPose);
      }

      @Override
      public YoGraphic getYoGraphic()
      {
         return graphicCylinder;
      }
   }

   private static class Point3DGraphicUpdater implements Shape3DGraphicUpdater
   {
      private final YoFramePoint3D position;
      private final YoGraphicPosition graphicSphere;
      private final FramePointShape3DReadOnly shape;

      public Point3DGraphicUpdater(String name, FramePointShape3DReadOnly shape, YoRegistry registry, AppearanceDefinition appearanceDefinition)
      {
         this.shape = shape;
         position = new YoFramePoint3D(name, worldFrame, registry);
         graphicSphere = new YoGraphicPosition(name, position, 0.01, appearanceDefinition);
      }

      @Override
      public void hide()
      {
         position.setToNaN();
      }

      @Override
      public void update()
      {
         position.setMatchingFrame(shape);
      }

      @Override
      public YoGraphic getYoGraphic()
      {
         return graphicSphere;
      }
   }

   private static class ConvexPolytope3DGraphicUpdater implements Shape3DGraphicUpdater
   {
      private final YoFramePoseUsingYawPitchRoll pose;
      private final YoGraphicShape graphicConvexPolytope;
      private final FrameConvexPolytope3DReadOnly shape;

      public ConvexPolytope3DGraphicUpdater(String name, FrameConvexPolytope3DReadOnly shape, YoRegistry registry,
                                            AppearanceDefinition appearanceDefinition)
      {
         this.shape = shape;
         pose = new YoFramePoseUsingYawPitchRoll(name, worldFrame, registry);
         Graphics3DObject convexPolytopeGraphicDefinition = new Graphics3DObject();
         convexPolytopeGraphicDefinition.addMeshData(newConvexPolytope3DMesh(shape), appearanceDefinition);
         graphicConvexPolytope = new YoGraphicShape(name, convexPolytopeGraphicDefinition, pose, 1.0);
      }

      @Override
      public void hide()
      {
         pose.setToNaN();
      }

      @Override
      public void update()
      {
         pose.setFromReferenceFrame(shape.getReferenceFrame());
      }

      @Override
      public YoGraphic getYoGraphic()
      {
         return graphicConvexPolytope;
      }
   }

   public static MeshDataHolder newConvexPolytope3DMesh(ConvexPolytope3DReadOnly convexPolytope3D)
   {
      MeshDataBuilder meshBuilder = new MeshDataBuilder();

      for (Face3DReadOnly face : convexPolytope3D.getFaces())
      {
         List<Point3D> ccwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         Collections.reverse(ccwFaceVertices);
         meshBuilder.addMesh(MeshDataGenerator.Polygon(ccwFaceVertices));
      }

      return meshBuilder.generateMeshDataHolder();
   }

   private static class EllipsoidGraphicUpdater implements Shape3DGraphicUpdater
   {
      private final YoFramePoseUsingYawPitchRoll pose;
      private final YoGraphicShape graphicEllipsoid;
      private final FrameEllipsoid3DReadOnly shape;

      private final FramePose3D tempPose = new FramePose3D();

      public EllipsoidGraphicUpdater(String name, FrameEllipsoid3DReadOnly shape, YoRegistry registry, AppearanceDefinition appearanceDefinition)
      {
         this.shape = shape;
         pose = new YoFramePoseUsingYawPitchRoll(name, worldFrame, registry);
         Graphics3DObject ellipsoidGraphicDefinition = new Graphics3DObject();
         ellipsoidGraphicDefinition.addEllipsoid(shape.getRadiusX(), shape.getRadiusY(), shape.getRadiusZ(), appearanceDefinition);
         graphicEllipsoid = new YoGraphicShape(name, ellipsoidGraphicDefinition, pose, 1.0);
      }

      @Override
      public void hide()
      {
         pose.setToNaN();
      }

      @Override
      public void update()
      {
         tempPose.setReferenceFrame(shape.getReferenceFrame());
         tempPose.set(shape.getPose());
         pose.setMatchingFrame(tempPose);
      }

      @Override
      public YoGraphic getYoGraphic()
      {
         return graphicEllipsoid;
      }
   }
}
