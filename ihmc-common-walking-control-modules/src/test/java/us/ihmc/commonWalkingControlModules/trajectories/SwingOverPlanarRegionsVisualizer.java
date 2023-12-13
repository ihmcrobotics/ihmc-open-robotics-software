package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander.SwingOverPlanarRegionsCollisionType;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SwingOverPlanarRegionsVisualizer
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();

   private final SimulationConstructionSet scs;
   private final SimulationConstructionSet2 scs2;

   private final YoFrameConvexPolygon2D yoFootPolygon;
   private final YoFrameConvexPolygon2D yoCollisionPolygon;
   private final YoFrameVector3D planeNormal;
   private final YoFramePoint3D planeOrigin;
   private final YoFramePoseUsingYawPitchRoll solePose;
   private final YoFramePoint3D swingFootPoint;
   private final YoFramePoint3D collisionBoxPoint;
   private final YoFramePoint3D firstWaypoint;
   private final YoFramePoint3D secondWaypoint;
   private final YoGraphicEllipsoid collisionSphere;
   private final YoGraphicPolygon swingFoot;
   private final YoGraphicPolygon collisionBox;
   private final YoGraphicPolygon stanceFootGraphic;
   private final YoGraphicPolygon swingStartGraphic;
   private final YoGraphicPolygon swingEndGraphic;
   private final YoGraphicPosition firstWaypointGraphic;
   private final YoGraphicPosition secondWaypointGraphic;
   private final YoGraphicVector planeVector;
   private final Map<SwingOverPlanarRegionsCollisionType, YoGraphicPosition> intersectionMap;

   private final ConvexPolygon2DReadOnly footPolygon;
   private final SwingOverPlanarRegionsTrajectoryExpander trajectoryExpander;

   public SwingOverPlanarRegionsVisualizer(SimulationConstructionSet scs, YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry,
                                           ConvexPolygon2DReadOnly footPolygon,
                                           SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander)
   {
      this(scs, null, registry, yoGraphicsListRegistry, footPolygon, swingOverPlanarRegionsTrajectoryExpander);
   }

   public SwingOverPlanarRegionsVisualizer(SimulationConstructionSet2 scs, YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry,
                                           ConvexPolygon2DReadOnly footPolygon,
                                           SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander)
   {
      this(null, scs, registry, yoGraphicsListRegistry, footPolygon, swingOverPlanarRegionsTrajectoryExpander);
   }

   private SwingOverPlanarRegionsVisualizer(SimulationConstructionSet scs, SimulationConstructionSet2 scs2, YoRegistry registry,
                                           YoGraphicsListRegistry yoGraphicsListRegistry,
                                           ConvexPolygon2DReadOnly footPolygon,
                                           SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander)
   {
      this.scs = scs;
      this.scs2 = scs2;
      this.trajectoryExpander = swingOverPlanarRegionsTrajectoryExpander;

      this.footPolygon = footPolygon;

      yoFootPolygon = new YoFrameConvexPolygon2D("footPolygon", ReferenceFrame.getWorldFrame(), 4, registry);
      yoCollisionPolygon = new YoFrameConvexPolygon2D("collisionPolygon", ReferenceFrame.getWorldFrame(), 4, registry);
      yoFootPolygon.set(footPolygon);
      double clearance = swingOverPlanarRegionsTrajectoryExpander.getMinimumClearance();
      yoCollisionPolygon.addVertex(trajectoryExpander.getToeLength() + clearance, 0.5 * trajectoryExpander.getFootWidth() + clearance);
      yoCollisionPolygon.addVertex(trajectoryExpander.getToeLength() + clearance, -0.5 * trajectoryExpander.getFootWidth() - clearance);
      yoCollisionPolygon.addVertex(-trajectoryExpander.getHeelLength() - clearance, 0.5 * trajectoryExpander.getFootWidth() + clearance);
      yoCollisionPolygon.addVertex(-trajectoryExpander.getHeelLength() - clearance, -0.5 * trajectoryExpander.getFootWidth() - clearance);

      swingOverPlanarRegionsTrajectoryExpander.attachVisualizer(this::update);

      double footHeight = trajectoryExpander.getFootHeight();
      double collisionBoxHeight = footHeight + 2.0 * trajectoryExpander.getMinimumClearance();

      solePose = new YoFramePoseUsingYawPitchRoll("SolePose", WORLD, registry);
      swingFootPoint = new YoFramePoint3D("SwingFootPoint", WORLD, registry);
      collisionBoxPoint = new YoFramePoint3D("collisionBoxPoint", WORLD, registry);
      firstWaypoint = new YoFramePoint3D("FirstWaypointViz", WORLD, registry);
      secondWaypoint = new YoFramePoint3D("SecondWaypointViz", WORLD, registry);
      AppearanceDefinition bubble = YoAppearance.LightBlue();
      bubble.setTransparency(0.5);
      collisionSphere = new YoGraphicEllipsoid("CollisionSphere", solePose.getPosition(), solePose.getYawPitchRoll(), bubble, new Vector3D());
      YoGraphicPosition trajectoryPosition = new YoGraphicPosition("TrajectoryPosition", swingFootPoint, 0.03, YoAppearance.Red());
      collisionBox = new YoGraphicPolygon("CollisionBox", yoCollisionPolygon, collisionBoxPoint, solePose.getYawPitchRoll(),
                                          1.0, collisionBoxHeight, bubble);
      swingFoot = new YoGraphicPolygon("SwingFoot", yoFootPolygon, solePose.getPosition(), solePose.getYawPitchRoll(),
                                         1.0, footHeight, YoAppearance.Green());
      stanceFootGraphic = new YoGraphicPolygon("StanceFootGraphic", footPolygon.getNumberOfVertices(), registry, true, 1.0, YoAppearance.Blue());
      swingStartGraphic = new YoGraphicPolygon("SwingStartGraphic", footPolygon.getNumberOfVertices(), registry, true, 1.0, YoAppearance.Green());
      swingEndGraphic = new YoGraphicPolygon("SwingEndGraphic", footPolygon.getNumberOfVertices(), registry, true, 1.0, YoAppearance.Yellow());
      firstWaypointGraphic = new YoGraphicPosition("FirstWaypointGraphic", firstWaypoint, 0.02, YoAppearance.White());
      secondWaypointGraphic = new YoGraphicPosition("SecondWaypointGraphic", secondWaypoint, 0.02, YoAppearance.White());
      intersectionMap = new HashMap<>();

      planeNormal = new YoFrameVector3D("planeNormal", ReferenceFrame.getWorldFrame(), registry);
      planeOrigin = new YoFramePoint3D("planeOrigin", ReferenceFrame.getWorldFrame(), registry);
      planeVector = new YoGraphicVector("planeNormal", planeOrigin, planeNormal, 0.2, YoAppearance.Red());

      for (SwingOverPlanarRegionsCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsCollisionType.values())
      {
         AppearanceDefinition appearance;
         double size;
         switch (swingOverPlanarRegionsTrajectoryCollisionType)
         {
         case COLLISION_BETWEEN_FEET:
         case COLLISION_INSIDE_TRAJECTORY:
            appearance = YoAppearance.Red();
            size = 0.014;
            break;
         case OUTSIDE_TRAJECTORY:
            appearance = YoAppearance.Orange();
            size = 0.013;
            break;
         case TOO_CLOSE_TO_IGNORE_PLANE:
            appearance = YoAppearance.Yellow();
            size = 0.012;
            break;
         case NO_INTERSECTION:
            appearance = YoAppearance.Blue();
            size = 0.011;
            break;
         default:
            appearance = YoAppearance.Black();
            size = 0.01;
            break;
         }
         intersectionMap.put(swingOverPlanarRegionsTrajectoryCollisionType,
                             new YoGraphicPosition("IntersectionGraphic" + swingOverPlanarRegionsTrajectoryCollisionType.name(),
                                                   new YoFramePoint3D("IntersectionPoint" + swingOverPlanarRegionsTrajectoryCollisionType.name(), WORLD,
                                                                      registry), size, appearance));

         yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", intersectionMap.get(swingOverPlanarRegionsTrajectoryCollisionType));
      }

      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", trajectoryPosition);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", planeVector);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", collisionSphere);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", collisionBox);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", swingFoot);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", stanceFootGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", swingStartGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", swingEndGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", firstWaypointGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", secondWaypointGraphic);
   }

   public void update()
   {
      solePose.setFromReferenceFrame(trajectoryExpander.getSolePoseReferenceFrame());

      for (SwingOverPlanarRegionsCollisionType collisionType : SwingOverPlanarRegionsCollisionType.values())
      {
         intersectionMap.get(collisionType).setPosition(trajectoryExpander.getClosestPolygonPoint(collisionType));
      }

      double sphereRadius = trajectoryExpander.getCollisionSphereRadius() + trajectoryExpander.getMinimumClearance();
      collisionSphere.setRadii(new Vector3D(sphereRadius, sphereRadius, sphereRadius));
      collisionSphere.update();

      double footHeight = trajectoryExpander.getFootHeight();
      double collisionBoxHeight = footHeight + 2.0 * trajectoryExpander.getMinimumClearance();

      swingFootPoint.set(solePose.getPosition());
      swingFootPoint.addZ(0.5 * footHeight);
      collisionBoxPoint.set(solePose.getPosition());
      collisionBoxPoint.subZ(0.5 * collisionBoxHeight);

      swingFoot.update();
      collisionBox.update();

      firstWaypoint.set(trajectoryExpander.getExpandedWaypoints().get(0));
      secondWaypoint.set(trajectoryExpander.getExpandedWaypoints().get(1));

      planeOrigin.set(trajectoryExpander.getSwingFloorPlane().getPoint());
      planeNormal.set(trajectoryExpander.getSwingFloorPlane().getNormal());
      swingStartGraphic.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), trajectoryExpander.getStartPose()));
      swingEndGraphic.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), trajectoryExpander.getEndPose()));

      swingStartGraphic.updateConvexPolygon2d(footPolygon);
      swingEndGraphic.updateConvexPolygon2d(footPolygon);

//      swingFloorPlane.set(swingStartPosition, tempPlaneNormal);

      if (scs != null)
         scs.tickAndUpdate(scs.getTime() + 0.1);
      if (scs2 != null)
         scs2.tick();
   }

   /*
   public void updateSwingFoot(FramePose3D swingFootPose)
   {
      Point3D frontLeft = new Point3D(trajectoryExpander.getToeLength(), 0.5 * trajectoryExpander.getFootWidth(), 0.0);
      Point3D frontRight = new Point3D(trajectoryExpander.getToeLength(), -0.5 * trajectoryExpander.getFootWidth(), 0.0);
      Point3D hindLeft = new Point3D(-trajectoryExpander.getHeelLength(), 0.5 * trajectoryExpander.getFootWidth(), 0.0);
      Point3D hindRight = new Point3D(-trajectoryExpander.getHeelLength(), -0.5 * trajectoryExpander.getFootWidth(), 0.0);

      RigidBodyTransform footTransform = new RigidBodyTransform();
      swingFootPose.get(footTransform);
      footTransform.transform(frontLeft);
      footTransform.transform(frontRight);
      footTransform.transform(hindRight);
      footTransform.transform(hindLeft);

      List<Point3D> footVertices = new ArrayList<>();
      footVertices.add(frontLeft);
      footVertices.add(frontRight);
      footVertices.add(hindLeft);
      footVertices.add(hindRight);

      swingFoot.setPose(swingFootPose);


      double clearance = trajectoryExpander.getMinimumClearance();
      frontLeft = new Point3D(trajectoryExpander.getToeLength() + clearance, 0.5 * trajectoryExpander.getFootWidth() + clearance, 0.0);
      frontRight = new Point3D(trajectoryExpander.getToeLength() + clearance, -0.5 * trajectoryExpander.getFootWidth() - clearance, 0.0);
      hindLeft = new Point3D(-trajectoryExpander.getHeelLength() - clearance, 0.5 * trajectoryExpander.getFootWidth()  + clearance, 0.0);
      hindRight = new Point3D(-trajectoryExpander.getHeelLength() - clearance, -0.5 * trajectoryExpander.getFootWidth() - clearance, 0.0);
      footTransform.transform(frontLeft);
      footTransform.transform(frontRight);
      footTransform.transform(hindRight);
      footTransform.transform(hindLeft);

      footVertices = new ArrayList<>();
      footVertices.add(frontLeft);
      footVertices.add(frontRight);
      footVertices.add(hindLeft);
      footVertices.add(hindRight);

      collisionBox.set
      collisionBox.set(footVertices);
   }
   */

   public void updateFoot(FramePose3D stanceFootPose, FramePose3D swingStartPose, FramePose3D swingEndPose)
   {
      stanceFootGraphic.setPose(stanceFootPose);
      stanceFootGraphic.updateConvexPolygon2d(footPolygon);
      swingStartGraphic.setPose(swingStartPose);
      swingStartGraphic.updateConvexPolygon2d(footPolygon);
      swingEndGraphic.setPose(swingEndPose);
      swingEndGraphic.updateConvexPolygon2d(footPolygon);
   }
}
