package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander.SwingOverPlanarRegionsCollisionType;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SwingOverPlanarRegionsVisualizer
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();

   private final SimulationConstructionSet scs;

   private final YoFrameVector3D planeNormal;
   private final YoFramePoint3D planeOrigin;
   private final YoFramePoseUsingYawPitchRoll solePose;
   private final YoFramePoint3D firstWaypoint;
   private final YoFramePoint3D secondWaypoint;
   private final YoGraphicEllipsoid collisionSphere;
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
      this.scs = scs;
      this.trajectoryExpander = swingOverPlanarRegionsTrajectoryExpander;

      this.footPolygon = footPolygon;

      swingOverPlanarRegionsTrajectoryExpander.attachVisualizer(this::update);

      solePose = new YoFramePoseUsingYawPitchRoll("SolePose", WORLD, registry);
      firstWaypoint = new YoFramePoint3D("FirstWaypointViz", WORLD, registry);
      secondWaypoint = new YoFramePoint3D("SecondWaypointViz", WORLD, registry);
      AppearanceDefinition bubble = YoAppearance.LightBlue();
      bubble.setTransparency(0.5);
      collisionSphere = new YoGraphicEllipsoid("CollisionSphere", solePose.getPosition(), solePose.getYawPitchRoll(), bubble, new Vector3D());
      YoGraphicPosition trajectoryPosition = new YoGraphicPosition("TrajectoryPosition", solePose.getPosition(), 0.03, YoAppearance.Red());
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

      firstWaypoint.set(trajectoryExpander.getExpandedWaypoints().get(0));
      secondWaypoint.set(trajectoryExpander.getExpandedWaypoints().get(1));

      planeOrigin.set(trajectoryExpander.getSwingFloorPlane().getPoint());
      planeNormal.set(trajectoryExpander.getSwingFloorPlane().getNormal());
      swingStartGraphic.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), trajectoryExpander.getStartPose()));
      swingEndGraphic.setPose(new FramePose3D(ReferenceFrame.getWorldFrame(), trajectoryExpander.getEndPose()));

      swingStartGraphic.updateConvexPolygon2d(footPolygon);
      swingEndGraphic.updateConvexPolygon2d(footPolygon);

//      swingFloorPlane.set(swingStartPosition, tempPlaneNormal);


      scs.tickAndUpdate(scs.getTime() + 0.1);
   }

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
