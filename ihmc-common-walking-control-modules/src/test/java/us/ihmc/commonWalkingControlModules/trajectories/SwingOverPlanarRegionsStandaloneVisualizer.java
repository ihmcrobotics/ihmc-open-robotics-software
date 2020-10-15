package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SwingOverPlanarRegionsStandaloneVisualizer
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();
   private static final AppearanceDefinition[] appearances = {YoAppearance.Gray(), YoAppearance.Gray()};

   private final SimulationConstructionSet scs;
   private final YoRegistry registry;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;
   private final SwingOverPlanarRegionsVisualizer visualizer;

   public SwingOverPlanarRegionsStandaloneVisualizer(SimulationConstructionSet scs, YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry,
                                                     WalkingControllerParameters walkingControllerParameters, ConvexPolygon2DReadOnly footPolygon)
   {
      this.scs = scs;
      this.registry = registry;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      swingOverPlanarRegionsTrajectoryExpander = new SwingOverPlanarRegionsTrajectoryExpander(walkingControllerParameters, registry, yoGraphicsListRegistry);
      visualizer = new SwingOverPlanarRegionsVisualizer(scs, registry, yoGraphicsListRegistry, footPolygon, swingOverPlanarRegionsTrajectoryExpander);
      swingOverPlanarRegionsTrajectoryExpander.attachVisualizer(visualizer::update);
   }

   public SwingOverPlanarRegionsStandaloneVisualizer(WalkingControllerParameters walkingControllerParameters, ConvexPolygon2DReadOnly footPolygon)
   {
      this(new SimulationConstructionSet(new Robot("Robot")), new YoRegistry(SwingOverPlanarRegionsStandaloneVisualizer.class.getSimpleName()),
           new YoGraphicsListRegistry(), walkingControllerParameters, footPolygon);

      scs.addYoRegistry(registry);
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
      Graphics3DObjectTools.addPlanarRegionsList(graphics3DObject, terrain, appearances);
      scs.addStaticLinkGraphics(graphics3DObject);

      FramePose3D stanceFootPose = new FramePose3D(WORLD);
      FramePose3D swingStartPose = new FramePose3D(WORLD);
      FramePose3D swingEndPose = new FramePose3D(WORLD);
      stanceFootPose.getPosition().set(0.4, 0.3, 0.0);
      swingStartPose.getPosition().set(0.0, 0.0, 0.0);
      swingEndPose.getPosition().set(0.8, 0.0, 0.0);

      swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose, swingStartPose, swingEndPose, terrain);

      scs.startOnAThread();
      scs.cropBuffer();
      ThreadTools.sleepForever();
   }

   public double expandTrajectoryOverPlanarRegions(FramePose3D stanceFootPose, FramePose3D swingStartPose, FramePose3D swingEndPose,
                                                   PlanarRegionsList planarRegionsList)
   {
      visualizer.updateFoot(stanceFootPose, swingStartPose, swingEndPose);
      return swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose, swingStartPose, swingEndPose, planarRegionsList);
   }

   public SwingOverPlanarRegionsTrajectoryExpander getSwingOverPlanarRegionsTrajectoryExpander()
   {
      return swingOverPlanarRegionsTrajectoryExpander;
   }
}
