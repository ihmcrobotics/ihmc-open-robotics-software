package us.ihmc.exampleSimulations.exampleContact;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.LinearStickSlipGroundContactModel;
import us.ihmc.simulationconstructionset.util.environments.ContactableSelectableBoxRobot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableConvexPolygonTerrainObject;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.MidiSliderBoard;
import us.ihmc.tools.thread.ThreadTools;

public class ExampleRockContactSimulation
{
   private final YoVariableRegistry registry = new YoVariableRegistry("ExampleRockContactSimulation");
   private final YoFramePoint groundCheckPoint = new YoFramePoint("groundCheckPoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint groundClosestPoint = new YoFramePoint("groundClosestPoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector groundClosestNormal = new YoFrameVector("groundClosestNormal", ReferenceFrame.getWorldFrame(), registry);
   
   double kXY = 1000.0;
   double bXY = 100.0;
   double kZ = 500.0;
   double bZ = 50.0;
   double alphaStick = 0.7;
   double alphaSlip = 0.5;
   
   private final CombinedTerrainObject3D combinedTerrainObject;
   private final Random random = new Random(1989L);
   
   private static final double WALL_START_X = 1.0;
   private static final double WALL_LENGTH = 5.0;
   private static final double WALL_Y = -0.7;
   private static final double WALL_THICKNESS = 0.05;
   private static final double PILLAR_WIDTH = 0.3;
   private static final int NUM_PILLARS = 6;
   
   private static final int NUM_ROCKS = 75;
   private static final double MAX_ROCK_CENTROID_HEIGHT = 0.35;
   private static final double MIN_ROCK_CENTROID_HEIGHT = 0.05;
   private static final int POINTS_PER_ROCK = 21;
   private static final double MAX_ABS_XY_NORMAL_VALUE = 0.2;
   private static final double ROCK_FIELD_WIDTH = 1.0;
   private static final double ROCK_BOUNDING_BOX_WIDTH = 0.3;
   
   private static final boolean FULLY_RANDOM = true; // Will do a neat grid if set to false;
   private static final int ROCKS_PER_ROW = 5;
   
   public ExampleRockContactSimulation()
   {
      
      combinedTerrainObject = new CombinedTerrainObject3D("Rocks with a wall");
      addWall();
      addPillars();
      addRocks();
      addGround();

      ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
      ArrayList<Robot> robots = new ArrayList<Robot>();
      
      ContactableSelectableBoxRobot contactableBoxRobot = new ContactableSelectableBoxRobot("BoxRobot", 0.1, 0.1, 0.1, 1.0);
      contactableBoxRobot.setPosition(2.90, 0.0, 1.5);
//      contactableBoxRobot.setVelocity(0.0, -1.8, 0.0);
//      contactableBoxRobot.setGravity(0.0);
      contactPoints.addAll(contactableBoxRobot.getAllGroundContactPoints());
      robots.add(contactableBoxRobot);
      
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      double forceVectorScale = 0.001;
      AppearanceDefinition appearance = YoAppearance.Green();
      contactableBoxRobot.addYoGraphicForceVectorsToGroundContactPoints(forceVectorScale, appearance , yoGraphicsListRegistry);
      
      YoGraphicPosition checkGroundPosition = new YoGraphicPosition("checkGround", groundCheckPoint, 0.01, YoAppearance.Orange());
      yoGraphicsListRegistry.registerYoGraphic("CheckGroundPosition", checkGroundPosition);
      
      YoGraphicVector checkGroundVector = new YoGraphicVector("checkGroundVector", groundClosestPoint, groundClosestNormal, YoAppearance.Pink());
      yoGraphicsListRegistry.registerYoGraphic("CheckGroundPosition", checkGroundVector);

//      GroundContactModel groundContactModel = new ExperimentalLinearStickSlipGroundContactModel(contactableBoxRobot, 
//            kXY, bXY, kZ, bZ, alphaSlip, alphaStick, contactableBoxRobot.getRobotsYoVariableRegistry());  
      
      GroundContactModel groundContactModel = new LinearStickSlipGroundContactModel(contactableBoxRobot, 
            kXY, bXY, kZ, bZ, alphaSlip, alphaStick, contactableBoxRobot.getRobotsYoVariableRegistry()); 
      
      groundContactModel.setGroundProfile3D(combinedTerrainObject);
      contactableBoxRobot.setGroundContactModel(groundContactModel);
      
      Robot[] robotsArray = new Robot[robots.size()];
      robots.toArray(robotsArray);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(16000);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robotsArray, new JMEGraphics3DAdapter(), parameters);
      scs.setSimulateDuration(3.0);
//      double tics = 100;
//      scs.setDT(1e-4 / tics, (int)(5*tics));
      scs.addStaticLinkGraphics(combinedTerrainObject.getLinkGraphics());
      
      scs.addYoVariableRegistry(registry);
      
      scs.setGroundVisible(false);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      
      Thread myThread = new Thread(scs);
      myThread.start();
      
      Vector3D  normal = new Vector3D();
      Point3D  intersection = new Point3D();
      
      MidiSliderBoard sliderBoard = new MidiSliderBoard(scs);
      int i = 1;

      // TODO: get these from CommonAvatarUserInterface once it exists:
      sliderBoard.setSlider(i++, "groundCheckPointx", scs, 2.0, 3.0);
      sliderBoard.setSlider(i++, "groundCheckPointy", scs, -0.5, 0.5);
      sliderBoard.setSlider(i++, "groundCheckPointz", scs, 0.0, 0.5);
      
      while(true)
      {
         ThreadTools.sleep(100);
         
         combinedTerrainObject.checkIfInside(groundCheckPoint.getX(), groundCheckPoint.getY(), groundCheckPoint.getZ(), intersection, normal);
         groundClosestPoint.set(intersection);
         
         normal.scale(0.03);
         groundClosestNormal.set(normal);
      }
   }
   
   private void addGround()
   {
      combinedTerrainObject.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0);
   }

   private void addRocks()
   {
      for(int i = 0; i < NUM_ROCKS; i++)
      {
         double centroidHeight = random.nextDouble() * (MAX_ROCK_CENTROID_HEIGHT - MIN_ROCK_CENTROID_HEIGHT) + MIN_ROCK_CENTROID_HEIGHT;
         Vector3D normal = generateRandomUpFacingNormal();

         double[] approximateCentroid = generateRandomApproximateCentroid(i);

         double[][] vertices = generateRandomRockVertices(approximateCentroid[0], approximateCentroid[1]);

         addRock(normal, centroidHeight, vertices);         
      }
   }

   private double[] generateRandomApproximateCentroid(int position)
   {
      double[] approximateCentroid = new double[2];
      
      if(FULLY_RANDOM)
      {
         approximateCentroid[0] = random.nextDouble() * WALL_LENGTH + WALL_START_X;
         approximateCentroid[1] = random.nextDouble() * ROCK_FIELD_WIDTH - ROCK_FIELD_WIDTH/2.0;
      }
      else
      {
         int row = position / ROCKS_PER_ROW;
         int rows = NUM_ROCKS / ROCKS_PER_ROW;
         double distancePerRow = WALL_LENGTH / ((double) rows - 1);
         approximateCentroid[0] = WALL_START_X + distancePerRow * row;

         int positionOnRow = position - row * ROCKS_PER_ROW;
         approximateCentroid[1] =  ROCK_FIELD_WIDTH * ((double) positionOnRow)/((double) ROCKS_PER_ROW)- ROCK_FIELD_WIDTH / 2.0;
      }
      return approximateCentroid;
   }

   private Vector3D generateRandomUpFacingNormal()
   {
      double normalX = random.nextDouble() * (2.0 * MAX_ABS_XY_NORMAL_VALUE) - MAX_ABS_XY_NORMAL_VALUE;
      double normalY = random.nextDouble() * (2.0 * MAX_ABS_XY_NORMAL_VALUE) - MAX_ABS_XY_NORMAL_VALUE;
      Vector3D normal = new Vector3D(normalX, normalY, 1.0);
      normal.normalize();
      return normal;
   }

   private double[][] generateRandomRockVertices(double approximateCentroidX, double approximateCentroidY)
   {
      double[][] vertices = new double[POINTS_PER_ROCK][2];

      for(int j = 0; j < POINTS_PER_ROCK; j++)
      {
         vertices[j][0] = random.nextDouble() * ROCK_BOUNDING_BOX_WIDTH + approximateCentroidX - ROCK_BOUNDING_BOX_WIDTH / 2.0;
         vertices[j][1] = random.nextDouble() * ROCK_BOUNDING_BOX_WIDTH + approximateCentroidY - ROCK_BOUNDING_BOX_WIDTH / 2.0;
      }
      return vertices;
   }

   private void addRock(Vector3D normal, double centroidHeight, double[][] vertices)
   {
      ArrayList<Point2D> vertexPoints = new ArrayList<Point2D>();
      
      for (double[] point : vertices)
      {
         Point2D point2d = new Point2D(point);
         vertexPoints.add(point2d);
      }
      
      ConvexPolygon2d convexPolygon = new ConvexPolygon2d(vertexPoints);
      AppearanceDefinition appearance = YoAppearance.Red();
      YoAppearance.makeTransparent(appearance, 0.7f);
      RotatableConvexPolygonTerrainObject rock = new RotatableConvexPolygonTerrainObject(normal, convexPolygon, centroidHeight, appearance);
      this.combinedTerrainObject.addTerrainObject(rock);
   }

   private void addWall()
   {
      Vector3D normal = new Vector3D(0.0, 0.0, 1.0);
      double centroidHeight = 2.0;
      ArrayList<Point2D> pointList = new ArrayList<Point2D>();

      Point2D wallPoint0 = new Point2D(WALL_START_X, WALL_Y);
      Point2D wallPoint1 = new Point2D(WALL_START_X + WALL_LENGTH, WALL_Y);
      Point2D wallPoint2 = new Point2D(WALL_START_X + WALL_LENGTH, WALL_Y + Math.signum(WALL_Y) * WALL_THICKNESS);
      Point2D wallPoint3 = new Point2D(WALL_START_X, WALL_Y + Math.signum(WALL_Y) * WALL_THICKNESS);
      pointList.add(wallPoint0);
      pointList.add(wallPoint1);
      pointList.add(wallPoint2);
      pointList.add(wallPoint3);
      
      ConvexPolygon2d convexPolygon = new ConvexPolygon2d(pointList);
      RotatableConvexPolygonTerrainObject rightWall = new RotatableConvexPolygonTerrainObject(normal, convexPolygon, centroidHeight, YoAppearance.Brown());
      combinedTerrainObject.addTerrainObject(rightWall);
   }
   
   private void addPillars()
   {
      Vector3D normal = new Vector3D(0.0, 0.0, 1.0);
      double centroidHeight = 2.0;
      
      Point2D bottomLeft = new Point2D(-PILLAR_WIDTH/2.0, PILLAR_WIDTH/2.0);
      Point2D bottomRight = new Point2D(-PILLAR_WIDTH/2.0, -PILLAR_WIDTH/2.0);
      Point2D topLeft = new Point2D(PILLAR_WIDTH/2.0, PILLAR_WIDTH/2.0);
      Point2D topRight = new Point2D(PILLAR_WIDTH/2.0, -PILLAR_WIDTH/2.0);
      
      double pillarDistance = ((double) WALL_LENGTH)/((double) NUM_PILLARS - 1.0);
      Vector2D offset = new Vector2D(0.0, -WALL_Y + PILLAR_WIDTH/2.0);
      
      for(int i = 0; i < NUM_PILLARS; i++)
      {
         ArrayList<Point2D> points = new ArrayList<Point2D>();
         offset.setX(WALL_START_X + pillarDistance * i);

         Point2D localBottomLeft = new Point2D();
         localBottomLeft.add(bottomLeft, offset);
         Point2D localBottomRight = new Point2D();
         localBottomRight.add(bottomRight, offset);
         Point2D localTopLeft = new Point2D();
         localTopLeft.add(topLeft, offset);
         Point2D localTopRight = new Point2D();
         localTopRight.add(topRight, offset);

         points.add(localBottomLeft);
         points.add(localBottomRight);
         points.add(localTopLeft);
         points.add(localTopRight);
         
         ConvexPolygon2d convexPolygon = new ConvexPolygon2d(points);
         AppearanceDefinition appearance = YoAppearance.Brown();
         YoAppearance.makeTransparent(appearance, 0.7f);
         RotatableConvexPolygonTerrainObject pillar = new RotatableConvexPolygonTerrainObject(normal, convexPolygon, centroidHeight, appearance);
         combinedTerrainObject.addTerrainObject(pillar);
      }
   }
   
   public static void main(String[] args)
   {
      new ExampleRockContactSimulation();
   }

}
