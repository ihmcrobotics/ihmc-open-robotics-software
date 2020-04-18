package us.ihmc.robotics.physics;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
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

      if (collidable.getShape() instanceof Sphere3DReadOnly)
      {
         shape3DGraphicUpdater = new Sphere3DGraphicUpdater(name,
                                                            (Sphere3DReadOnly) collidable.getShape(),
                                                            collidable.getShapeFrame(),
                                                            registry,
                                                            appearanceDefinition);
      }
      else if (collidable.getShape() instanceof Box3DReadOnly)
      {
         shape3DGraphicUpdater = new Box3DGraphicUpdater(name,
                                                         (Box3DReadOnly) collidable.getShape(),
                                                         collidable.getShapeFrame(),
                                                         registry,
                                                         appearanceDefinition);
      }
      else if (collidable.getShape() instanceof Capsule3DReadOnly)
      {
         shape3DGraphicUpdater = new Capsule3DGraphicUpdater(name,
                                                             (Capsule3DReadOnly) collidable.getShape(),
                                                             collidable.getShapeFrame(),
                                                             registry,
                                                             appearanceDefinition);
      }
      else if (collidable.getShape() instanceof PointShape3DReadOnly)
      {
         shape3DGraphicUpdater = new Point3DGraphicUpdater(name,
                                                           (PointShape3DReadOnly) collidable.getShape(),
                                                           collidable.getShapeFrame(),
                                                           registry,
                                                           appearanceDefinition);
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported type of shape: " + collidable.getShape().getClass().getSimpleName());
      }

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
      private Sphere3DReadOnly shape;
      private ReferenceFrame shapeFrame;

      public Sphere3DGraphicUpdater(String name, Sphere3DReadOnly shape, ReferenceFrame shapeFrame, YoVariableRegistry registry,
                                    AppearanceDefinition appearanceDefinition)
      {
         this.shape = shape;
         this.shapeFrame = shapeFrame;
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
         tempPoint.setIncludingFrame(shapeFrame, shape.getPosition());
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
      private final Box3DReadOnly shape;
      private final ReferenceFrame shapeFrame;

      public Box3DGraphicUpdater(String name, Box3DReadOnly shape, ReferenceFrame shapeFrame, YoVariableRegistry registry,
                                 AppearanceDefinition appearanceDefinition)
      {
         this.shape = shape;
         this.shapeFrame = shapeFrame;
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
         tempPose.setIncludingFrame(shapeFrame, shape.getPose());
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
      private final Capsule3DReadOnly shape;
      private final ReferenceFrame shapeFrame;

      public Capsule3DGraphicUpdater(String name, Capsule3DReadOnly shape, ReferenceFrame shapeFrame, YoVariableRegistry registry,
                                     AppearanceDefinition appearanceDefinition)
      {
         this.shape = shape;
         this.shapeFrame = shapeFrame;
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
         tempPose.setReferenceFrame(shapeFrame);
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
      private PointShape3DReadOnly shape;
      private ReferenceFrame shapeFrame;

      public Point3DGraphicUpdater(String name, PointShape3DReadOnly shape, ReferenceFrame shapeFrame, YoVariableRegistry registry,
                                   AppearanceDefinition appearanceDefinition)
      {
         this.shape = shape;
         this.shapeFrame = shapeFrame;
         position = new YoFramePoint3D(name, worldFrame, registry);
         graphicSphere = new YoGraphicPosition(name, position, 0.01, appearanceDefinition);
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
         tempPoint.setIncludingFrame(shapeFrame, shape);
         position.setMatchingFrame(tempPoint);
      }

      @Override
      public YoGraphic getYoGraphic()
      {
         return graphicSphere;
      }
   }
}
