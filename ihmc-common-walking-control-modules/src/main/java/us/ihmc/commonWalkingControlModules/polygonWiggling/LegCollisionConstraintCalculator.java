package us.ihmc.commonWalkingControlModules.polygonWiggling;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCylinder;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class LegCollisionConstraintCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final static double defaultLegRadiusGraphic = 0.21; // YoGraphicCylinder doesn't support changing radius on the fly...

   private Cylinder3D legCollisionShape = null;
   private final RigidBodyTransform legShapeTransformToSoleFrame = new RigidBodyTransform();
   private final RigidBodyTransform legShapeToWorld = new RigidBodyTransform();

   private final Vector3D collisionDirection = new Vector3D();
   private final BoundingBox3D legBoundingBox = new BoundingBox3D();
   private final ExpandingPolytopeAlgorithm collisionDetector = new ExpandingPolytopeAlgorithm();
   private final EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();

   private final YoFramePoint3D legShapeBase = new YoFramePoint3D("legShapeBase", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D legShapeDirection = new YoFrameVector3D("legShapeDirection", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicCylinder legCollisionShapeGraphic;
   private final YoFramePoint3D legIntersectionPosition = new YoFramePoint3D("legIntersectionPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D regionIntersectionPosition = new YoFramePoint3D("regionIntersectionPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D gradientDirection = new YoFrameVector3D("intersectionNormal", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition legIntersectionPositionGraphic;
   private final YoGraphicPosition regionIntersectionPositionGraphic;
   private final YoGraphicVector gradientDirectionGraphic;

   public LegCollisionConstraintCalculator()
   {
      legCollisionShapeGraphic = null;
      legIntersectionPositionGraphic = null;
      regionIntersectionPositionGraphic = null;
      gradientDirectionGraphic = null;
   }

   public LegCollisionConstraintCalculator(YoGraphicsListRegistry graphicsListRegistry, YoRegistry parentRegistry)
   {
      AppearanceDefinition appearance = YoAppearance.LightGoldenRodYellow();
      appearance.setTransparency(0.85);

      legCollisionShapeGraphic = new YoGraphicCylinder("legCollisionGraphic", legShapeBase, legShapeDirection, appearance, defaultLegRadiusGraphic);
      legIntersectionPositionGraphic = new YoGraphicPosition("intersectionPositionGraphic", legIntersectionPosition, 0.01, YoAppearance.Orange());
      regionIntersectionPositionGraphic = new YoGraphicPosition("regionIntersectionPositionGraphic", regionIntersectionPosition, 0.01, YoAppearance.Black());
      gradientDirectionGraphic = new YoGraphicVector("intersectionDirectionGraphic", legIntersectionPosition, gradientDirection, 1.0, YoAppearance.Orange(), true, 0.01);

      legShapeBase.setToZero();
      legShapeDirection.set(0.0, 0.0, 1.0);

      String graphicListName = getClass().getSimpleName();
      graphicsListRegistry.registerYoGraphic(graphicListName, legCollisionShapeGraphic);
      graphicsListRegistry.registerYoGraphic(graphicListName, legIntersectionPositionGraphic);
//      graphicsListRegistry.registerYoGraphic(graphicListName, regionIntersectionPositionGraphic);
      graphicsListRegistry.registerYoGraphic(graphicListName, gradientDirectionGraphic);

      legIntersectionPosition.setToNaN();
      regionIntersectionPosition.setToNaN();

      parentRegistry.addChild(registry);
   }

   public void calculateLegCollisionGradient(RigidBodyTransformReadOnly soleFrameToRegionFrame,
                                             RigidBodyTransformReadOnly regionToWorld,
                                             PlanarRegionsList planarRegionsList,
                                             Tuple3DBasics gradientToPack)
   {
      gradientToPack.setToZero();

      if (legCollisionShape == null)
      {
         return;
      }

      legShapeToWorld.set(legShapeTransformToSoleFrame);
      legShapeToWorld.preMultiply(soleFrameToRegionFrame);
      legShapeToWorld.preMultiply(regionToWorld);
      legCollisionShape.getPosition().set(legShapeToWorld.getTranslation());
      legCollisionShape.getAxis().set(legShapeToWorld.getM02(), legShapeToWorld.getM12(), legShapeToWorld.getM22());

      legCollisionShape.getBoundingBox(legBoundingBox);
      updateGraphics();

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion region = planarRegionsList.getPlanarRegion(i);

         if (!region.getBoundingBox3dInWorld().intersectsInclusive(legBoundingBox))
         {
            continue;
         }
         else if (!collisionDetector.evaluateCollision(legCollisionShape, region, collisionResult))
         {
            continue;
         }

         collisionDirection.sub(collisionResult.getPointOnB(), collisionResult.getPointOnA());
         collisionDirection.applyInverseTransform(regionToWorld);
         collisionDirection.setZ(0.0);

         gradientToPack.set(collisionDirection.getX(), collisionDirection.getY(), 0.0);
         updateGraphics(collisionResult.getPointOnA(), collisionResult.getPointOnB());

         break;
      }
   }

   private void updateGraphics()
   {
      if (legCollisionShapeGraphic == null)
      {
         return;
      }

      legShapeDirection.set(legCollisionShape.getAxis());
      legShapeDirection.scale(legCollisionShape.getLength());

      legShapeBase.set(legShapeDirection);
      legShapeBase.scale(-0.5);
      legShapeBase.add(legCollisionShape.getPosition());

      legCollisionShapeGraphic.update();
   }

   private void updateGraphics(Point3D legIntersectionPoint, Point3D regionIntersectionPoint)
   {
      if (legCollisionShapeGraphic == null)
      {
         return;
      }

      this.legIntersectionPosition.set(legIntersectionPoint);
      this.regionIntersectionPosition.set(regionIntersectionPoint);
      this.gradientDirection.set(collisionDirection);

      legIntersectionPositionGraphic.update();
      regionIntersectionPositionGraphic.update();
      gradientDirectionGraphic.update();
   }

   public void setLegCollisionShape(Cylinder3D legCollisionShape, RigidBodyTransform legShapeTransformToSoleFrame)
   {
      this.legCollisionShape = legCollisionShape;
      this.legShapeTransformToSoleFrame.set(legShapeTransformToSoleFrame);
   }
}
