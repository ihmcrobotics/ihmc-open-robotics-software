package us.ihmc.footstepPlanning.swing;

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
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
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

public class CollisionFreeSwingCalculatorGraphicalTest
{
   private static final boolean runSCS = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   private final DefaultFootstepPlannerParametersBasics footstepPlannerParameters = new DefaultFootstepPlannerParameters();
   private final SwingPlannerParametersBasics swingParameters = new DefaultSwingPlannerParameters();
   private final WalkingControllerParameters walkingControllerParameters = new ProxyAtlasWalkingControllerParameters();
   private final SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(ProxyAtlasWalkingControllerParameters::getProxyAtlasFootPolygon);

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

   public void testSmallBox()
   {
      double boxLengthX = 0.15;
      double boxHeight = 0.15;

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

   public void testBigBox()
   {
      double boxLengthX = 0.3;
      double boxHeight = 0.2;

      swingParameters.set(SwingPlannerParameterKeys.maxDisplacementHigh, 0.4);
      swingParameters.set(SwingPlannerParameterKeys.maxDisplacementLow, 0.1);

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

   public void testStepUpAndDown()
   {
      double boxLengthX = 0.3;
      double boxHeight = 0.2;

      swingParameters.set(SwingPlannerParameterKeys.maxDisplacementHigh, 0.4);
      swingParameters.set(SwingPlannerParameterKeys.maxDisplacementLow, 0.1);

      PlanarRegionsList boxOnGround = boxOnGround(boxLengthX, boxHeight);
      double footForwardOffset = walkingControllerParameters.getSteppingParameters().getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset();
      double xClearance = 0.03;

      SideDependentList<Pose3D> initialStance = new SideDependentList<>(side -> new Pose3D(-0.5 * boxLengthX - xClearance - footForwardOffset,
                                                                                           0.5
                                                                                           * side.negateIfRightSide(walkingControllerParameters.getSteppingParameters()
                                                                                                                                               .getInPlaceWidth()),
                                                                                           0.0,
                                                                                           0.0,
                                                                                           0.0,
                                                                                           0.0));
      FootstepPlan footstepPlan = new FootstepPlan();

      {
         RobotSide steppingSide0 = RobotSide.LEFT;
         FramePose3D footstep0 = new FramePose3D();
         footstep0.setZ(boxHeight);
         footstep0.setY(0.5 * walkingControllerParameters.getSteppingParameters().getInPlaceWidth());
         footstepPlan.addFootstep(steppingSide0, footstep0);
      }

      {
         RobotSide steppingSide1 = RobotSide.RIGHT;
         FramePose3D footstep1 = new FramePose3D();
         footstep1.setZ(boxHeight);
         footstep1.setY(-0.5 * walkingControllerParameters.getSteppingParameters().getInPlaceWidth());
         footstepPlan.addFootstep(steppingSide1, footstep1);
      }

      {
         RobotSide steppingSide2 = RobotSide.LEFT;
         FramePose3D footstep2 = new FramePose3D();
         footstep2.setX(0.5 * boxLengthX + xClearance + footBackwardOffset);
         footstep2.setY(initialStance.get(steppingSide2).getY());
         footstepPlan.addFootstep(steppingSide2, footstep2);
      }

      {
         RobotSide steppingSide3 = RobotSide.LEFT;
         FramePose3D footstep3 = new FramePose3D();
         footstep3.setX(0.5 * boxLengthX + xClearance + footBackwardOffset);
         footstep3.setY(-initialStance.get(steppingSide3).getY());
         footstepPlan.addFootstep(steppingSide3, footstep3);
      }

      runTest(boxOnGround, initialStance, footstepPlan);
   }

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

   public void testOverCinders1()
   {
      DataSet cindersDataSet = DataSetIOTools.loadDataSet(DataSetName._20190220_172417_EOD_Cinders);
      PlanarRegionsList cindersRegions = cindersDataSet.getPlanarRegionsList();

      SideDependentList<Pose3D> stanceSteps = new SideDependentList<>();
      stanceSteps.put(RobotSide.LEFT,  new Pose3D(new Point3D( 0.488,  0.110, -0.000 ), new Quaternion(-0.002, -0.008, -0.199,  0.980 )));
      stanceSteps.put(RobotSide.RIGHT, new Pose3D(new Point3D( 0.344, -0.240, -0.000 ), new Quaternion(-0.002, -0.008, -0.198,  0.980 )));

      FootstepPlan footstepPlan = new FootstepPlan();
      footstepPlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D( 0.700, -0.300,  0.109 ), new Quaternion(-0.001, -0.004, -0.174,  0.985 )));
      footstepPlan.addFootstep(RobotSide.LEFT , new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D( 0.950, -0.100,  0.110 ), new Quaternion(-0.001, -0.004, -0.174,  0.985 )));
      footstepPlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D( 1.258, -0.452,  0.203 ), new Quaternion(-0.001, -0.004, -0.173,  0.985 )));
      footstepPlan.addFootstep(RobotSide.LEFT , new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D( 1.500, -0.250,  0.204 ), new Quaternion(-0.001, -0.004, -0.174,  0.985 )));
      footstepPlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D( 1.834, -0.601,  0.120 ), new Quaternion( 0.000,  0.000, -0.164,  0.986 )));
      footstepPlan.addFootstep(RobotSide.LEFT , new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D( 1.906, -0.393,  0.120 ), new Quaternion( 0.000,  0.000, -0.164,  0.986 )));

      runTest(cindersRegions, stanceSteps, footstepPlan);
   }

   public void testOverCinders2()
   {
      DataSet cindersDataSet = DataSetIOTools.loadDataSet(DataSetName._20191213_134839_Cinders);
      PlanarRegionsList cindersRegions = cindersDataSet.getPlanarRegionsList();

      SideDependentList<Pose3D> stanceSteps = new SideDependentList<>();
      stanceSteps.put(RobotSide.LEFT,  new Pose3D(new Point3D(-0.148,  0.176, -0.000 ), new Quaternion( 0.005, -0.006,  0.613,  0.790 )));
      stanceSteps.put(RobotSide.RIGHT, new Pose3D(new Point3D( 0.210,  0.084, -0.000 ), new Quaternion( 0.005, -0.006,  0.614,  0.790 )));

      FootstepPlan footstepPlan = new FootstepPlan();
      footstepPlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D( 0.300,  0.420, -0.039 ), new Quaternion( 0.000,  0.016,  0.500,  0.866 )));
      footstepPlan.addFootstep(RobotSide.LEFT , new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D( 0.150,  0.880,  0.065 ), new Quaternion( 0.001,  0.016,  0.643,  0.766 )));
      footstepPlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D( 0.511,  1.142,  0.149 ), new Quaternion( 0.001,  0.021,  0.503,  0.864 )));
      footstepPlan.addFootstep(RobotSide.LEFT , new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D( 0.379,  1.621,  0.126 ), new Quaternion( 0.116,  0.089,  0.633,  0.761 )));
      footstepPlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D( 0.767,  1.953,  0.130 ), new Quaternion( 0.000,  0.000,  0.613,  0.790 )));
      footstepPlan.addFootstep(RobotSide.LEFT , new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D( 0.553,  2.007,  0.130 ), new Quaternion( 0.000,  0.000,  0.613,  0.790 )));

      runTest(cindersRegions, stanceSteps, footstepPlan);
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

         CollisionFreeSwingCalculator swingCalculator = new CollisionFreeSwingCalculator(footstepPlannerParameters,
                                                                                         swingParameters,
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
         CollisionFreeSwingCalculator swingCalculator = new CollisionFreeSwingCalculator(footstepPlannerParameters,
                                                                                         swingParameters,
                                                                                         walkingControllerParameters,
                                                                                         footPolygons);
         swingCalculator.setPlanarRegionsList(planarRegionsList);
         swingCalculator.computeSwingTrajectories(initialStance, footstepPlan);
      }
   }

   public static void main(String[] args)
   {
      CollisionFreeSwingCalculatorGraphicalTest graphicalTest = new CollisionFreeSwingCalculatorGraphicalTest();
      graphicalTest.testFlatGround();
//      graphicalTest.testSmallBox();
//      graphicalTest.testBigBox();
//      graphicalTest.testStepUpAndDown();
//      graphicalTest.testUpStairs();
//      graphicalTest.testOverCinders1();
//      graphicalTest.testOverCinders2();
   }
}
