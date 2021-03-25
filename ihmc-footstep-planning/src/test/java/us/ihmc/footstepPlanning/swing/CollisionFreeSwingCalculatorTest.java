package us.ihmc.footstepPlanning.swing;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class CollisionFreeSwingCalculatorTest
{
   private static final boolean runSCS = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   private final SwingPlannerParametersBasics swingParameters = new DefaultSwingPlannerParameters();
   private final WalkingControllerParameters walkingControllerParameters = new ProxyAtlasWalkingControllerParameters();
   private final SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(ProxyAtlasWalkingControllerParameters::getProxyAtlasFootPolygon);

   @Test
   public void testFlatGround()
   {
      PlanarRegionsList flatGround = flatGround();
      SideDependentList<Pose3D> initialStance = new SideDependentList<>(side -> new Pose3D(0.0,
                                                                                           0.5
                                                                                           * side.negateIfRightSide(walkingControllerParameters.getSteppingParameters()
                                                                                                                                               .getInPlaceWidth()),
                                                                                           0.0,
                                                                                           0.0,
                                                                                           0.0,
                                                                                           0.0));
      FootstepPlan footstepPlan = new FootstepPlan();

      RobotSide steppingSide = RobotSide.LEFT;
      FramePose3D footstep = new FramePose3D(initialStance.get(steppingSide));
      footstep.getPosition().addX(0.3);
      footstepPlan.addFootstep(steppingSide, footstep);

      runTest(flatGround, initialStance, footstepPlan);
   }

   @Test
   public void testOverBox1()
   {
      double boxLengthX = 0.2;
      double boxHeight = 0.2;

      PlanarRegionsList boxOnGround = boxOnGround(boxLengthX, boxHeight);
      double footForwardOffset = walkingControllerParameters.getSteppingParameters().getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset();
      double xClearance = 0.05;

      SideDependentList<Pose3D> initialStance = new SideDependentList<>(side -> new Pose3D(-0.5 * boxLengthX - xClearance - footForwardOffset,
                                                                                           0.5
                                                                                           * side.negateIfRightSide(walkingControllerParameters.getSteppingParameters()
                                                                                                                                               .getInPlaceWidth()),
                                                                                           0.0,
                                                                                           0.0,
                                                                                           0.0,
                                                                                           0.0));
      FootstepPlan footstepPlan = new FootstepPlan();

      RobotSide steppingSide = RobotSide.LEFT;
      FramePose3D footstep = new FramePose3D();
      footstep.setX(0.5 * boxLengthX + xClearance + footBackwardOffset);
      footstep.setY(initialStance.get(steppingSide).getY());
      footstepPlan.addFootstep(steppingSide, footstep);

      runTest(boxOnGround, initialStance, footstepPlan);

   }

   @Test
   public void testOverBox2()
   {
      double boxLengthX = 0.3;
      double boxHeight = 0.3;

      PlanarRegionsList boxOnGround = boxOnGround(boxLengthX, boxHeight);
      double footForwardOffset = walkingControllerParameters.getSteppingParameters().getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset();
      double xClearance = 0.05;

      SideDependentList<Pose3D> initialStance = new SideDependentList<>(side -> new Pose3D(-0.5 * boxLengthX - xClearance - footForwardOffset,
                                                                                           0.5
                                                                                           * side.negateIfRightSide(walkingControllerParameters.getSteppingParameters()
                                                                                                                                               .getInPlaceWidth()),
                                                                                           0.0,
                                                                                           0.0,
                                                                                           0.0,
                                                                                           0.0));
      FootstepPlan footstepPlan = new FootstepPlan();

      RobotSide steppingSide = RobotSide.LEFT;
      FramePose3D footstep = new FramePose3D();
      footstep.setX(0.5 * boxLengthX + xClearance + footBackwardOffset);
      footstep.setY(initialStance.get(steppingSide).getY());
      footstepPlan.addFootstep(steppingSide, footstep);

      runTest(boxOnGround, initialStance, footstepPlan);
   }

   @Test
   public void testUpStairs()
   {
      DataSet stairsDataSet = DataSetIOTools.loadDataSet(DataSetName._20200513_151318_StairsIHMC_Bottom);
      PlanarRegionsList stairRegions = stairsDataSet.getPlanarRegionsList();

      SideDependentList<Pose3D> stanceSteps = new SideDependentList<>();
      stanceSteps.put(RobotSide.LEFT,  new Pose3D(new Point3D(-0.624,  0.365, -0.042 ), new Quaternion( 0.008, -0.000,  1.000,  0.002 )));
      stanceSteps.put(RobotSide.RIGHT, new Pose3D(new Point3D(-0.623,  0.735, -0.042 ), new Quaternion( 0.008, -0.000,  1.000,  0.000 )));

      FootstepPlan footstepPlan = new FootstepPlan();
      footstepPlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-1.051,  0.720,  0.108 ), new Quaternion( 0.007, -0.001,  0.997, -0.072 )));
      footstepPlan.addFootstep(RobotSide.LEFT,  new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-1.024,  0.252,  0.106 ), new Quaternion( 0.007, -0.001,  0.996, -0.087 )));
      footstepPlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-1.345,  0.550,  0.279 ), new Quaternion( 0.005,  0.000,  0.985, -0.174 )));
      footstepPlan.addFootstep(RobotSide.LEFT,  new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-1.317,  0.152,  0.279 ), new Quaternion( 0.005,  0.001,  1.000, -0.000 )));
      footstepPlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-1.607,  0.503,  0.460 ), new Quaternion( 0.002,  0.001,  0.996, -0.087 )));
      footstepPlan.addFootstep(RobotSide.LEFT,  new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-1.588,  0.197,  0.460 ), new Quaternion( 0.002,  0.001,  0.996, -0.088 )));
      footstepPlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-1.879,  0.352,  0.621 ), new Quaternion(-0.008,  0.000,  0.985, -0.174 )));
      footstepPlan.addFootstep(RobotSide.LEFT,  new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-1.863,  0.148,  0.621 ), new Quaternion(-0.008, -0.001,  1.000, -0.000 )));
      footstepPlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-2.100,  0.514,  0.782 ), new Quaternion(-0.004, -0.002,  0.996, -0.087 )));
      footstepPlan.addFootstep(RobotSide.LEFT,  new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-2.076,  0.150,  0.781 ), new Quaternion(-0.004, -0.002,  0.996, -0.087 )));
      footstepPlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-2.550,  0.360,  0.965 ), new Quaternion( 0.000,  0.000,  1.000,  0.001 )));

      runTest(stairRegions, stanceSteps, footstepPlan);
   }

   private PlanarRegionsList flatGround()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(5.0, 5.0);
      return generator.getPlanarRegionsList();
   }

   private PlanarRegionsList boxOnGround(double lengthX, double height)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(5.0, 5.0);
      generator.addCubeReferencedAtBottomMiddle(lengthX, 1.5, height);
      return generator.getPlanarRegionsList();
   }

   private void runTest(PlanarRegionsList planarRegionsList, SideDependentList<? extends Pose3DReadOnly> initialStance, FootstepPlan footstepPlan)
   {
      if (runSCS)
      {
         SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
         YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

         Graphics3DObject planarRegionsGraphics = new Graphics3DObject();
         AppearanceDefinition[] appearances = new AppearanceDefinition[] {YoAppearance.RGBColorFromHex(0xC9B887),
                                                                          YoAppearance.RGBColorFromHex(0x4A8577),
                                                                          YoAppearance.RGBColorFromHex(0x87616A),
                                                                          YoAppearance.RGBColorFromHex(0x655C7D),
                                                                          YoAppearance.RGBColorFromHex(0x5C797D)};
         Graphics3DObjectTools.addPlanarRegionsList(planarRegionsGraphics, planarRegionsList, appearances);
         scs.addStaticLinkGraphics(planarRegionsGraphics);

         CollisionFreeSwingCalculator swingCalculator = new CollisionFreeSwingCalculator(swingParameters,
                                                                                         walkingControllerParameters,
                                                                                         footPolygons,
                                                                                         scs,
                                                                                         graphicsListRegistry,
                                                                                         scs.getRootRegistry());

         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.setGroundVisible(false);
         scs.startOnAThread();

         swingCalculator.setPlanarRegionsList(planarRegionsList);
         swingCalculator.computeSwingTrajectories(initialStance, footstepPlan);
         scs.cropBuffer();
         ThreadTools.sleepForever();
      }
      else
      {
         CollisionFreeSwingCalculator swingCalculator = new CollisionFreeSwingCalculator(swingParameters, walkingControllerParameters, footPolygons);
         swingCalculator.setPlanarRegionsList(planarRegionsList);
         swingCalculator.computeSwingTrajectories(initialStance, footstepPlan);
      }
   }
}
