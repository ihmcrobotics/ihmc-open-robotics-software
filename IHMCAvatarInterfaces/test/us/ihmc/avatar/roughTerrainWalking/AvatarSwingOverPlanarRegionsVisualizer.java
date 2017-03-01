package us.ihmc.avatar.roughTerrainWalking;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander.SwingOverPlanarRegionsTrajectoryCollisionType;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicEllipsoid;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
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

public class AvatarSwingOverPlanarRegionsVisualizer
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();
   private static final AppearanceDefinition[] appearances = {YoAppearance.Gray(), YoAppearance.Gray()};

   private final SimulationConstructionSet scs;
   private final YoVariableRegistry registry;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final YoFramePose solePose;
   private final YoGraphicEllipsoid collisionSphere;
   private final YoGraphicPolygon stanceFootGraphic;
   private final YoGraphicPolygon swingStartGraphic;
   private final YoGraphicPolygon swingEndGraphic;
   private final Map<SwingOverPlanarRegionsTrajectoryCollisionType, YoGraphicPosition> intersectionMap;

   private final ConvexPolygon2d footPolygon;
   private final FramePose stanceFootPose;
   private final FramePose swingStartPose;
   private final FramePose swingEndPose;
   private final SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;

   public AvatarSwingOverPlanarRegionsVisualizer(SimulationConstructionSet scs, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry,
                                                 WalkingControllerParameters walkingControllerParameters, RobotContactPointParameters contactPointParameters)
   {
      this.scs = scs;
      this.registry = registry;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      footPolygon = new ConvexPolygon2d(contactPointParameters.getFootContactPoints().get(RobotSide.LEFT));

      swingOverPlanarRegionsTrajectoryExpander = new SwingOverPlanarRegionsTrajectoryExpander(walkingControllerParameters, registry, yoGraphicsListRegistry);
      swingOverPlanarRegionsTrajectoryExpander.attachVisualizer(this::update);

      solePose = new YoFramePose("SolePose", WORLD, registry);
      AppearanceDefinition bubble = YoAppearance.LightBlue();
      bubble.setTransparency(0.5);
      collisionSphere = new YoGraphicEllipsoid("CollisionSphere", solePose.getPosition(), solePose.getOrientation(), bubble, new Vector3D());
      stanceFootGraphic = new YoGraphicPolygon("StanceFootGraphic", footPolygon.getNumberOfVertices(), registry, 1.0, YoAppearance.Blue());
      swingStartGraphic = new YoGraphicPolygon("SwingStartGraphic", footPolygon.getNumberOfVertices(), registry, 1.0, YoAppearance.Green());
      swingEndGraphic = new YoGraphicPolygon("SwingEndGraphic", footPolygon.getNumberOfVertices(), registry, 1.0, YoAppearance.Yellow());
      intersectionMap = new HashMap<SwingOverPlanarRegionsTrajectoryCollisionType, YoGraphicPosition>();
      for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType.values())
      {
         AppearanceDefinition appearance;
         double size;
         switch (swingOverPlanarRegionsTrajectoryCollisionType)
         {
         case CRITICAL_INTERSECTION:
            appearance = YoAppearance.Red();
            size = 0.014;
            break;
         case INTERSECTION_BUT_OUTSIDE_TRAJECTORY:
            appearance = YoAppearance.Orange();
            size = 0.013;
            break;
         case INTERSECTION_BUT_BELOW_IGNORE_PLANE:
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
                                                   new YoFramePoint("IntersectionPoint" + swingOverPlanarRegionsTrajectoryCollisionType.name(), WORLD,
                                                                    registry),
                                                   size, appearance));

         yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", intersectionMap.get(swingOverPlanarRegionsTrajectoryCollisionType));
      }

      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", collisionSphere);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", stanceFootGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", swingStartGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", swingEndGraphic);

      stanceFootPose = new FramePose(WORLD);
      swingStartPose = new FramePose(WORLD);
      swingEndPose = new FramePose(WORLD);
   }

   public AvatarSwingOverPlanarRegionsVisualizer(WalkingControllerParameters walkingControllerParameters, RobotContactPointParameters contactPointParameters)
   {
      this(new SimulationConstructionSet(new Robot("Robot")), new YoVariableRegistry(AvatarSwingOverPlanarRegionsVisualizer.class.getSimpleName()),
           new YoGraphicsListRegistry(), walkingControllerParameters, contactPointParameters);

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setDT(1.0, 1);
      scs.setCameraFix(0.4, 0.0, 0.0);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.4, 0.0, 0.0001);
      generator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 0.001);
      generator.translate(-0.15, 0.0, 0.0001);
      generator.translate(0.0, 0.0, 0.25);
      generator.addRectangle(0.1, 0.1);
//      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.1);
      generator.translate(0.52, -0.12, 0.1);
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
      swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose, swingStartPose, swingEndPose, terrain);

      scs.startOnAThread();
      scs.cropBuffer();
      ThreadTools.sleepForever();
   }

   public double expandTrajectoryOverPlanarRegions(FramePose stanceFootPose, FramePose swingStartPose, FramePose swingEndPose,
                                                 PlanarRegionsList planarRegionsList)
   {
      this.stanceFootPose.set(stanceFootPose);
      this.swingStartPose.set(swingStartPose);
      this.swingEndPose.set(swingEndPose);
      updateFootGraphics();
      return swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose, swingStartPose, swingEndPose, planarRegionsList);
   }

   public SwingOverPlanarRegionsTrajectoryExpander getSwingOverPlanarRegionsTrajectoryExpander()
   {
      return swingOverPlanarRegionsTrajectoryExpander;
   }

   private void update(double dt)
   {
      solePose.setFromReferenceFrame(swingOverPlanarRegionsTrajectoryExpander.getSolePoseReferenceFrame());

      for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType.values())
      {
         intersectionMap.get(swingOverPlanarRegionsTrajectoryCollisionType)
                        .setPosition(swingOverPlanarRegionsTrajectoryExpander.getClosestPolygonPoint(swingOverPlanarRegionsTrajectoryCollisionType));
      }

      double sphereRadius = swingOverPlanarRegionsTrajectoryExpander.getSphereRadius();
      collisionSphere.setRadii(new Vector3D(sphereRadius, sphereRadius, sphereRadius));
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
