package us.ihmc.footstepPlanning.environments;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class PlanarRegionsListDefinedEnvironmentExample
{

   public static void startEnvironmentVisualizationForPlanarRegionsList(PlanarRegionsList planarRegionsList, boolean generateGroundPlane, boolean visualizeGroundProfile,
         boolean visualizeTerrainLinkGraphics, boolean visualizeNormals, boolean visualizeBoundingBoxes)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("PlanarRegionsListEnvironmentVisualizationRobot"));
      scs.setGroundVisible(false);
      YoVariableRegistry robotsYoVariableRegistry = scs.getRobots()[0].getRobotsYoVariableRegistry();
      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment("ExamplePlanarRegionsListEnvironment", planarRegionsList, 1e-5,
                                                                                                generateGroundPlane);
      TerrainObject3D terrainObject3D = environment.getTerrainObject3D();

      if (visualizeNormals)
      {
         YoGraphicsList vectors = new YoGraphicsList("NormalVectors");
         for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
         {
            PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
            YoFramePoint3D planarRegionPointInWorld = new YoFramePoint3D("PlanarRegionPoint" + i, ReferenceFrame.getWorldFrame(), robotsYoVariableRegistry);
            YoFrameVector3D surfaceNormal = new YoFrameVector3D("NormalVector" + i, ReferenceFrame.getWorldFrame(), robotsYoVariableRegistry);

            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            Point3D translation = new Point3D();
            planarRegion.getTransformToWorld(transformToWorld);
            transformToWorld.getTranslation(translation);
            planarRegionPointInWorld.set(translation);

            Vector3D normal = new Vector3D();
            terrainObject3D.getHeightMapIfAvailable().heightAndNormalAt(translation.getX(), translation.getY(), translation.getZ(), normal);
            surfaceNormal.set(normal);

            YoGraphicVector surfaceNormalGraphic = new YoGraphicVector("PlanarRegionSurfaceNormalGraphic" + i, planarRegionPointInWorld, surfaceNormal,
                  YoAppearance.Aqua());
            vectors.add(surfaceNormalGraphic);
         }

         scs.addYoGraphicsList(vectors, false);
      }

      if (visualizeBoundingBoxes)
      {
         for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
         {
            PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            Point3D translation = new Point3D();
            planarRegion.getTransformToWorld(transformToWorld);

            Graphics3DObject boundingBoxVisualization = new Graphics3DObject();
            BoundingBox3D boundingBox3dInWorld = planarRegion.getBoundingBox3dInWorld();

            double lx = boundingBox3dInWorld.getMaxX() - boundingBox3dInWorld.getMinX();
            double ly = boundingBox3dInWorld.getMaxY() - boundingBox3dInWorld.getMinY();
            double lz = boundingBox3dInWorld.getMaxZ() - boundingBox3dInWorld.getMinZ();

            transformToWorld.getTranslation(translation);
            translation.setZ(translation.getZ() - (lz / 2.0));
            boundingBoxVisualization.translate(translation);
            boundingBoxVisualization.addCube(lx, ly, lz, YoAppearance.RGBColor(1.0, 0, 0, 0.5));

            scs.addStaticLinkGraphics(boundingBoxVisualization, Graphics3DNodeType.VISUALIZATION);
         }
      }

      if (visualizeGroundProfile)
      {
         HeightMapWithNormals heightMap = terrainObject3D.getHeightMapIfAvailable();
         Graphics3DObject heightMapGraphics = new Graphics3DObject();
         heightMapGraphics.addHeightMap(heightMap, 300, 300, YoAppearance.DarkGreen());
         scs.addStaticLinkGraphics(heightMapGraphics);
      }

      if (visualizeTerrainLinkGraphics)
      {
         scs.addStaticLinkGraphics(terrainObject3D.getLinkGraphics());
      }

      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      double startX = 0.0;
      double startY = 0.0;
      double cinderBlockSize = 0.4;
      double cinderBlockHeight = 0.15;
      int courseWidthXInNumberOfBlocks = 21;
      int courseLengthYInNumberOfBlocks = 6;
      double heightVariation = 0.1;
      //      Random random = new Random(1776L);

//      PlanarRegionsList planarRegionsList = PlanarRegionsListExamples
//            .generateCinderBlockField(startX, startY, cinderBlockSize, cinderBlockHeight, courseWidthXInNumberOfBlocks, courseLengthYInNumberOfBlocks, heightVariation);

      PlanarRegionsList planarRegionsList = PlanarRegionsListExamples
            .generateSteppingStonesEnvironment(3.5);

      startEnvironmentVisualizationForPlanarRegionsList(planarRegionsList, false, true, true, true, true);
   }
}
