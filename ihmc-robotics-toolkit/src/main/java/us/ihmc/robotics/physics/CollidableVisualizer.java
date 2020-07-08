package us.ihmc.robotics.physics;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

public class CollidableVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Collidable collidable;
   private final Shape3DGraphicUpdater shape3DGraphicUpdater;

   public CollidableVisualizer(String name, String groupName, Collidable collidable, AppearanceDefinition appearanceDefinition, YoVariableRegistry registry,
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

      public Sphere3DGraphicUpdater(String name, FrameSphere3DReadOnly shape, YoVariableRegistry registry, AppearanceDefinition appearanceDefinition)
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

      public Box3DGraphicUpdater(String name, FrameBox3DReadOnly shape, YoVariableRegistry registry, AppearanceDefinition appearanceDefinition)
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
      private final YoGraphicShape graphicBox;
      private final FrameCapsule3DReadOnly shape;

      public Capsule3DGraphicUpdater(String name, FrameCapsule3DReadOnly shape, YoVariableRegistry registry, AppearanceDefinition appearanceDefinition)
      {
         this.shape = shape;
         pose = new YoFramePoseUsingYawPitchRoll(name, worldFrame, registry);
         Graphics3DObject capsuleGraphicDefinition = new Graphics3DObject();
         double radius = shape.getRadius();
         double length = shape.getLength();
         capsuleGraphicDefinition.addCapsule(radius, length + 2.0 * radius, appearanceDefinition);
         graphicBox = new YoGraphicShape(name, capsuleGraphicDefinition, pose, 1.0);
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
         return graphicBox;
      }
   }

   private static class Cylinder3DGraphicUpdater implements Shape3DGraphicUpdater
   {
      private final YoFramePoseUsingYawPitchRoll pose;
      private final YoGraphicShape graphicBox;
      private final FrameCylinder3DReadOnly shape;

      public Cylinder3DGraphicUpdater(String name, FrameCylinder3DReadOnly shape, YoVariableRegistry registry, AppearanceDefinition appearanceDefinition)
      {
         this.shape = shape;
         pose = new YoFramePoseUsingYawPitchRoll(name, worldFrame, registry);
         Graphics3DObject cylinderGraphicDefinition = new Graphics3DObject();
         double radius = shape.getRadius();
         double length = shape.getLength();
         cylinderGraphicDefinition.translate(0.0, 0.0, -0.5 * length);
         cylinderGraphicDefinition.addCylinder(length, radius, appearanceDefinition);
         graphicBox = new YoGraphicShape(name, cylinderGraphicDefinition, pose, 1.0);
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
         return graphicBox;
      }
   }

   private static class Point3DGraphicUpdater implements Shape3DGraphicUpdater
   {
      private final YoFramePoint3D position;
      private final YoGraphicPosition graphicSphere;
      private final FramePointShape3DReadOnly shape;

      public Point3DGraphicUpdater(String name, FramePointShape3DReadOnly shape, YoVariableRegistry registry, AppearanceDefinition appearanceDefinition)
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
}
