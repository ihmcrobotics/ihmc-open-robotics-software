package us.ihmc.simulationconstructionset;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.UI})
public class LinkGraphicsTest
{
   private SimulationConstructionSet sim;

   private static final double
      CUBE_L = 0.2, CUBE_W = 0.1, CUBE_H = 0.3;
   private static final double SPHERE_R = 0.15;
   private static final double
      ELLIPSOID_RX = 0.1, ELLIPSOID_RY = 0.2, ELLIPSOID_RZ = 0.3;
   private static final double
      HEMI_ELLIPSOID_RX = 0.2, HEMI_ELLIPSOID_RY = 0.1, HEMI_ELLIPSOID_RZ = 0.4;
   private static final double
      CYLINDER_H = 0.4, CYLINDER_R = 0.05;
   private static final double
      CONE_H = 0.4, CONE_R = 0.1;
   private static final double
      GEN_TRUNCATED_CONE_HEIGHT = 0.2, GEN_TRUNCATED_CONE_BX = 0.15, GEN_TRUNCATED_CONE_BY = 0.15, GEN_TRUNCATED_CONE_TX = 0.1, GEN_TRUNCATED_CONE_TY = 0.05;
   private static final double
      ARC_TORUS_START_ANG = 0.0, ARC_TORUS_END_ANG = 1.5 * Math.PI, ARC_TORUS_MAJ_RAD = 0.2, ARC_TORUS_MIN_RAD = 0.05;
   private static final double
      PYRAMID_CUBE_LX = 0.15, PYRAMID_CUBE_LY = 0.15, PYRAMID_CUBE_LZ = 0.08, PYRAMID_CUBE_LH = 0.15;


   private static final double
      OFFSET = 1.2, COORD_LENGTH = 0.5;

   private static final double
      WEDGE_X = 0.4, WEDGE_Y = 0.3, WEDGE_Z = 0.2;

	@ContinuousIntegrationTest(estimatedDuration = 3.3)
	@Test(timeout = 30000)
   public void testLinkGraphicsWithALargeNumberOfExampleShapes()
   {
      Link link = aLargerNumberOfExampleShapes();
      startSimAndDisplayLink(link);
   }

	@ContinuousIntegrationTest(estimatedDuration = 3.4)
	@Test(timeout = 30000)
   public void testLinkGraphicsWithASmallNumberOfExampleShapes()
   {
      Link link = aSmallNumberOfExampleShapes();
      startSimAndDisplayLink(link);
   }

	@ContinuousIntegrationTest(estimatedDuration = 3.4)
	@Test(timeout = 30000)
   public void testLinkGraphicsWithArcTorus()
   {
      Link link = exampleArcTorusShape();
      startSimAndDisplayLink(link);
   }

	@ContinuousIntegrationTest(estimatedDuration = 3.3)
	@Test(timeout = 30000)
   public void testLinkGrapicsWithMeshData()
   {
      Link link = exampleMeshDataShape();
      startSimAndDisplayLink(link);
   }

	@ContinuousIntegrationTest(estimatedDuration = 3.3)
	@Test(timeout = 30000)
   public void testLinkGrapicsWithCone()
   {
      Link link = exampleConeShape();
      startSimAndDisplayLink(link);
   }

	@ContinuousIntegrationTest(estimatedDuration = 3.3)
	@Test(timeout = 30000)
   public void testLinkGrapicsWithExtrudedPolygon()
   {
      Link link = exampleExtrudedPolygonShape();
      startSimAndDisplayLink(link);
   }

   private void startSimAndDisplayLink(Link linkToDisplay)
   {
//    Robot nullRobot = new Robot("Null");
      sim = new SimulationConstructionSet();

      sim.addStaticLink(linkToDisplay);

      // position the camera to view links
      sim.setCameraPosition(6.0, 6.0, 3.0);
      sim.setCameraFix(0.5, 0.5, 0.0);

      sim.setGroundVisible(false);
      sim.startOnAThread();

      ThreadTools.sleep(3000);
      sim.closeAndDispose();
   }


   private Link aSmallNumberOfExampleShapes()
   {
      Link ret = new Link("example shapes");

      Graphics3DObject linkGraphics = new Graphics3DObject();

      // Cube
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addCube(CUBE_L, CUBE_W, CUBE_H, YoAppearance.Teal());

      // Sphere
      linkGraphics.translate(OFFSET, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addSphere(SPHERE_R, YoAppearance.Aqua());

      // Ellipsoid
      linkGraphics.translate(OFFSET, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addEllipsoid(ELLIPSOID_RX, ELLIPSOID_RY, ELLIPSOID_RZ, YoAppearance.Navy());

//    // Hemiellipsoid
//    linkGraphics.translate(-2.0 * OFFSET, OFFSET, 0.0);
//    linkGraphics.addCoordinateSystem(COORD_LENGTH);
//    linkGraphics.addHemiEllipsoid(HEMI_ELLIPSOID_RX, HEMI_ELLIPSOID_RY, HEMI_ELLIPSOID_RZ, YoAppearance.DarkBlue());
//
//    // Cylinder
//    linkGraphics.translate(OFFSET, 0.0, 0.0);
//    linkGraphics.addCoordinateSystem(COORD_LENGTH);
//    linkGraphics.addCylinder(CYLINDER_H, CYLINDER_R, YoAppearance.Green());

//    // Cone
//    linkGraphics.translate(OFFSET, 0.0, 0.0);
//    linkGraphics.addCoordinateSystem(COORD_LENGTH);
//    linkGraphics.addCone(CONE_H, CONE_R, YoAppearance.DarkGreen());

      // GenTruncCone
//    linkGraphics.translate(-2.0 * OFFSET, OFFSET, 0.0);
//    linkGraphics.addCoordinateSystem(COORD_LENGTH);
//    linkGraphics.addGenTruncatedCone(GEN_TRUNCATED_CONE_HEIGHT, GEN_TRUNCATED_CONE_BX, GEN_TRUNCATED_CONE_BY, GEN_TRUNCATED_CONE_TX, GEN_TRUNCATED_CONE_TY,
//                                     YoAppearance.Red());

      // This is breaking for some reason!
//    // ArcTorus
//    linkGraphics.translate(OFFSET, 0.0, 0.0);
//    linkGraphics.addCoordinateSystem(COORD_LENGTH);
//    linkGraphics.addArcTorus(ARC_TORUS_START_ANG, ARC_TORUS_END_ANG, ARC_TORUS_MAJ_RAD, ARC_TORUS_MIN_RAD, YoAppearance.DarkRed());
      //
      // PyramidCube
//    linkGraphics.translate(OFFSET, 0.0, 0.0);
//    linkGraphics.addCoordinateSystem(COORD_LENGTH);
//    linkGraphics.addPyramidCube(PYRAMID_CUBE_LX, PYRAMID_CUBE_LY, PYRAMID_CUBE_LZ, PYRAMID_CUBE_LH, YoAppearance.BlackMetalMaterial());


//    // Extruded Polygon
//    linkGraphics.translate(-2.0 * OFFSET, OFFSET, 0.0);
//    linkGraphics.addCoordinateSystem(COORD_LENGTH);
//    ArrayList<Point2d> polygonPoints = new ArrayList<Point2d>();
//    polygonPoints.add(new Point2d(0.0, 0.0));
//    polygonPoints.add(new Point2d(0.4, 0.0));
//    polygonPoints.add(new Point2d(0.3, 0.3));
//    double height = 0.25;
//    linkGraphics.addExtrudedPolygon(polygonPoints, height, YoAppearance.Purple());

      // This is breaking for some reason!
//    // Mesh Data
//    linkGraphics.translate(OFFSET, 0.0, 0.0);
//    linkGraphics.addCoordinateSystem(COORD_LENGTH);
//    MeshDataHolder meshData = MeshDataGenerator.Wedge(WEDGE_X, WEDGE_Y, WEDGE_Z);
//    AppearanceDefinition meshAppearance = YoAppearance.Gold();
//    linkGraphics.addMeshData(meshData, meshAppearance );

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link aLargerNumberOfExampleShapes()
   {
      Link ret = new Link("example shapes");

      Graphics3DObject linkGraphics = new Graphics3DObject();

      // Cube
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addCube(CUBE_L, CUBE_W, CUBE_H, YoAppearance.Teal());

      // Sphere
      linkGraphics.translate(OFFSET, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addSphere(SPHERE_R, YoAppearance.Aqua());

      // Ellipsoid
      linkGraphics.translate(OFFSET, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addEllipsoid(ELLIPSOID_RX, ELLIPSOID_RY, ELLIPSOID_RZ, YoAppearance.Navy());

      // Hemiellipsoid
      linkGraphics.translate(-2.0 * OFFSET, OFFSET, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addHemiEllipsoid(HEMI_ELLIPSOID_RX, HEMI_ELLIPSOID_RY, HEMI_ELLIPSOID_RZ, YoAppearance.DarkBlue());

      // Cylinder
      linkGraphics.translate(OFFSET, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addCylinder(CYLINDER_H, CYLINDER_R, YoAppearance.Green());

      // Cone
      linkGraphics.translate(OFFSET, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addCone(CONE_H, CONE_R, YoAppearance.DarkGreen());

      // GenTruncCone
      linkGraphics.translate(-2.0 * OFFSET, OFFSET, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addGenTruncatedCone(GEN_TRUNCATED_CONE_HEIGHT, GEN_TRUNCATED_CONE_BX, GEN_TRUNCATED_CONE_BY, GEN_TRUNCATED_CONE_TX, GEN_TRUNCATED_CONE_TY,
                                       YoAppearance.Red());

      // ArcTorus
      linkGraphics.translate(OFFSET, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addArcTorus(ARC_TORUS_START_ANG, ARC_TORUS_END_ANG, ARC_TORUS_MAJ_RAD, ARC_TORUS_MIN_RAD, YoAppearance.DarkRed());

      // PyramidCube
      linkGraphics.translate(OFFSET, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addPyramidCube(PYRAMID_CUBE_LX, PYRAMID_CUBE_LY, PYRAMID_CUBE_LZ, PYRAMID_CUBE_LH, YoAppearance.BlackMetalMaterial());


      // Extruded Polygon
      linkGraphics.translate(-2.0 * OFFSET, OFFSET, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      ArrayList<Point2D> polygonPoints = new ArrayList<Point2D>();
      polygonPoints.add(new Point2D(0.0, 0.0));
      polygonPoints.add(new Point2D(0.4, 0.0));
      polygonPoints.add(new Point2D(0.3, 0.3));
      double height = 0.25;
      linkGraphics.addExtrudedPolygon(polygonPoints, height, YoAppearance.Purple());

//    // Mesh Data
      linkGraphics.translate(OFFSET, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      MeshDataHolder meshData = MeshDataGenerator.Wedge(WEDGE_X, WEDGE_Y, WEDGE_Z);
      AppearanceDefinition meshAppearance = YoAppearance.Gold();
      linkGraphics.addMeshData(meshData, meshAppearance);

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link exampleConeShape()
   {
      Link ret = new Link("exampleArcTorusShape");

      Graphics3DObject linkGraphics = new Graphics3DObject();

      // Cone
      linkGraphics.translate(OFFSET, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addCone(CONE_H, CONE_R, YoAppearance.DarkGreen());

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link exampleExtrudedPolygonShape()
   {
      Link ret = new Link("exampleExtrudedPolygonShape");
      Graphics3DObject linkGraphics = new Graphics3DObject();

      // Extruded Polygon
      linkGraphics.translate(-2.0 * OFFSET, OFFSET, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      ArrayList<Point2D> polygonPoints = new ArrayList<Point2D>();
      polygonPoints.add(new Point2D(0.0, 0.0));
      polygonPoints.add(new Point2D(0.4, 0.0));
      polygonPoints.add(new Point2D(0.3, 0.3));
      double height = 0.25;
      linkGraphics.addExtrudedPolygon(polygonPoints, height, YoAppearance.Purple());

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link exampleArcTorusShape()
   {
      Link ret = new Link("exampleArcTorusShape");

      Graphics3DObject linkGraphics = new Graphics3DObject();

      // ArcTorus
      linkGraphics.translate(OFFSET, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addArcTorus(ARC_TORUS_START_ANG, ARC_TORUS_END_ANG, ARC_TORUS_MAJ_RAD, ARC_TORUS_MIN_RAD, YoAppearance.DarkRed());

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link exampleMeshDataShape()
   {
      Link ret = new Link("exampleArcTorusShape");
      Graphics3DObject linkGraphics = new Graphics3DObject();

      // Mesh Data
      linkGraphics.translate(OFFSET, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      MeshDataHolder meshData = MeshDataGenerator.Wedge(WEDGE_X, WEDGE_Y, WEDGE_Z);
      AppearanceDefinition meshAppearance = YoAppearance.Gold();
      linkGraphics.addMeshData(meshData, meshAppearance);

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

}
