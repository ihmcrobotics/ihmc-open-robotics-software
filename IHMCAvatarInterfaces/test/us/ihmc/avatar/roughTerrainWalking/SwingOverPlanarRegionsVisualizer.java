package us.ihmc.avatar.roughTerrainWalking;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.graphics3DDescription.Graphics3DObject;
import us.ihmc.graphics3DDescription.appearance.AppearanceDefinition;
import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicEllipsoid;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class SwingOverPlanarRegionsVisualizer
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();
   private static final AppearanceDefinition[] appearances = {YoAppearance.Gray(), YoAppearance.Gray()};

   private final SimulationConstructionSet scs;
   private final YoVariableRegistry registry;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final YoFramePose solePose;
   private final YoFramePoint closestPolygonPoint;
   private final YoFramePoint intersectionPoint;
   private final YoFramePoint blockingIntersectionPoint;
   private final YoGraphicEllipsoid collisionSphere;
   private final YoGraphicPolygon stanceFootGraphic;
   private final YoGraphicPolygon swingStartGraphic;
   private final YoGraphicPolygon swingEndGraphic;
   private final YoGraphicPosition closestPolygonPointGraphic;
   private final YoGraphicPosition intersectionGraphic;
   private final YoGraphicPosition blockingIntersectionGraphic;

   private final ConvexPolygon2d footPolygon;
   private final FramePose stanceFootPose;
   private final FramePose swingStartPose;
   private final FramePose swingEndPose;
   private final SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;

   public SwingOverPlanarRegionsVisualizer(SimulationConstructionSet scs, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry,
                                           WalkingControllerParameters walkingControllerParameters, RobotContactPointParameters contactPointParameters)
   {
      this.scs = scs;
      this.registry = registry;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      footPolygon = new ConvexPolygon2d(contactPointParameters.getFootContactPoints().get(RobotSide.LEFT));

      swingOverPlanarRegionsTrajectoryExpander = new SwingOverPlanarRegionsTrajectoryExpander(walkingControllerParameters, registry, yoGraphicsListRegistry);
      swingOverPlanarRegionsTrajectoryExpander.attachVisualizer(this::update);

      solePose = new YoFramePose("SolePose", WORLD, registry);
      intersectionPoint = new YoFramePoint("IntersectionPoint", WORLD, registry);
      blockingIntersectionPoint = new YoFramePoint("BlockingIntersectionPoint", WORLD, registry);
      closestPolygonPoint = new YoFramePoint("ClosestPolygonPoint", WORLD, registry);
      AppearanceDefinition bubble = YoAppearance.LightBlue();
      bubble.setTransparency(0.5);
      collisionSphere = new YoGraphicEllipsoid("CollisionSphere", solePose.getPosition(), solePose.getOrientation(), bubble, new Vector3d());
      stanceFootGraphic = new YoGraphicPolygon("StanceFootGraphic", footPolygon.getNumberOfVertices(), registry, 1.0, YoAppearance.Blue());
      swingStartGraphic = new YoGraphicPolygon("SwingStartGraphic", footPolygon.getNumberOfVertices(), registry, 1.0, YoAppearance.Green());
      swingEndGraphic = new YoGraphicPolygon("SwingEndGraphic", footPolygon.getNumberOfVertices(), registry, 1.0, YoAppearance.Yellow());
      closestPolygonPointGraphic = new YoGraphicPosition("ClosestPolygonPointGraphic", closestPolygonPoint, 0.01, YoAppearance.Blue());
      intersectionGraphic = new YoGraphicPosition("IntersectionGraphic", intersectionPoint, 0.011, YoAppearance.Yellow());
      blockingIntersectionGraphic = new YoGraphicPosition("BlockingIntersectionGraphic", blockingIntersectionPoint, 0.012, YoAppearance.Red());

      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", collisionSphere);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", stanceFootGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", swingStartGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", swingEndGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", closestPolygonPointGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", intersectionGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", blockingIntersectionGraphic);

      stanceFootPose = new FramePose(WORLD);
      swingStartPose = new FramePose(WORLD);
      swingEndPose = new FramePose(WORLD);
   }

   public SwingOverPlanarRegionsVisualizer(WalkingControllerParameters walkingControllerParameters, RobotContactPointParameters contactPointParameters)
   {
      this(new SimulationConstructionSet(new Robot("Robot")), new YoVariableRegistry(SwingOverPlanarRegionsVisualizer.class.getSimpleName()),
           new YoGraphicsListRegistry(), walkingControllerParameters, contactPointParameters);

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setDT(1.0, 1);
      scs.setCameraFix(0.4, 0.0, 0.0);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.4, 0.12, 0.0001);
      generator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 0.001);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.1);
      PlanarRegionsList terrain = generator.getPlanarRegionsList();
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.addCoordinateSystem(0.3);
      graphics3DObject.addPlanarRegionsList(terrain, appearances);
      scs.addStaticLinkGraphics(graphics3DObject);

      stanceFootPose.setPosition(0.4, 0.3, 0.0);
      swingStartPose.setPosition(0.0, 0.0, 0.0);
      swingEndPose.setPosition(0.8, 0.0, 0.0);
      updateFootGraphics();
      swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(footPolygon, stanceFootPose, swingStartPose, swingEndPose, terrain);

      scs.startOnAThread();
      scs.cropBuffer();
      ThreadTools.sleepForever();
   }

   public void expandTrajectoryOverPlanarRegions(ConvexPolygon2d footPolygonSoleFrame, FramePose stanceFootPose, FramePose swingStartPose,
                                                 FramePose swingEndPose, PlanarRegionsList planarRegionsList)
   {
      this.stanceFootPose.set(stanceFootPose);
      this.swingStartPose.set(swingStartPose);
      this.swingEndPose.set(swingEndPose);
      updateFootGraphics();
      swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(footPolygon, stanceFootPose, swingStartPose, swingEndPose, planarRegionsList);
   }

   public SwingOverPlanarRegionsTrajectoryExpander getSwingOverPlanarRegionsTrajectoryExpander()
   {
      return swingOverPlanarRegionsTrajectoryExpander;
   }

   private void update(double dt)
   {
      solePose.setFromReferenceFrame(swingOverPlanarRegionsTrajectoryExpander.getSolePoseReferenceFrame());
      closestPolygonPoint.set(swingOverPlanarRegionsTrajectoryExpander.getClosestPolygonPoint());
      if (swingOverPlanarRegionsTrajectoryExpander.isIntersecting())
      {
         intersectionPoint.set(swingOverPlanarRegionsTrajectoryExpander.getClosestPolygonPoint());
      }
      else
      {
         intersectionPoint.setToNaN();
      }
      if (swingOverPlanarRegionsTrajectoryExpander.isIntersectingAndAbovePlane())
      {
         blockingIntersectionPoint.set(swingOverPlanarRegionsTrajectoryExpander.getClosestPolygonPoint());
      }
      else
      {
         blockingIntersectionPoint.setToNaN();
      }
      double sphereRadius = swingOverPlanarRegionsTrajectoryExpander.getSphereRadius();
      collisionSphere.setRadii(new Vector3d(sphereRadius, sphereRadius, sphereRadius));
      collisionSphere.update();

      scs.tickAndUpdate(scs.getTime() + dt);
   }

   private void updateFootGraphics()
   {
      stanceFootGraphic.setPose(stanceFootPose);
      stanceFootGraphic.updateConvexPolygon2d(footPolygon);
      swingStartGraphic.setPose(swingStartPose);
      swingStartGraphic.updateConvexPolygon2d(footPolygon);
      swingEndGraphic.setPose(swingEndPose);
      swingEndGraphic.updateConvexPolygon2d(footPolygon);
   }
}
