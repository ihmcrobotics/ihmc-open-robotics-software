package us.ihmc.footstepPlanning.environments;

import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.graphics3DAdapter.HeightMapWithNormals;
import us.ihmc.graphics3DDescription.Graphics3DObject;
import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsList;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class PlanarRegionsListDefinedEnvironmentExample
{

   public static void startEnvironmentVisualizationForPlanarRegionsList(PlanarRegionsList planarRegionsList, boolean visualizeGroundProfile,
         boolean visualizeTerrainLinkGraphics, boolean visualizeNormals, boolean visualizeBoundingBoxes)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("PlanarRegionsListEnvironmentVisualizationRobot"));
      YoVariableRegistry robotsYoVariableRegistry = scs.getRobots()[0].getRobotsYoVariableRegistry();
      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment("ExamplePlanarRegionsListEnvironment", planarRegionsList);
      TerrainObject3D terrainObject3D = environment.getTerrainObject3D();

      if(visualizeNormals)
      {
         YoGraphicsList vectors = new YoGraphicsList("NormalVectors");
         for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
         {
            PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
            YoFramePoint planarRegionPointInWorld = new YoFramePoint("PlanarRegionPoint" + i, ReferenceFrame.getWorldFrame(), robotsYoVariableRegistry);
            YoFrameVector surfaceNormal = new YoFrameVector("NormalVector" + i, ReferenceFrame.getWorldFrame(),
                  robotsYoVariableRegistry);

            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            Point3d translation = new Point3d();
            planarRegion.getTransformToWorld(transformToWorld);
            transformToWorld.getTranslation(translation);
            planarRegionPointInWorld.set(translation);

            Vector3d normal = new Vector3d();
            terrainObject3D.getHeightMapIfAvailable().heightAndNormalAt(translation.x, translation.y, translation.z, normal);
            surfaceNormal.setVector(normal);

            YoGraphicVector surfaceNormalGraphic = new YoGraphicVector("PlanarRegionSurfaceNormalGraphic" + i, planarRegionPointInWorld, surfaceNormal, YoAppearance.Aqua());
            vectors.add(surfaceNormalGraphic);
         }

         scs.addYoGraphicsList(vectors, false);
      }

      if(visualizeBoundingBoxes)
      {
         for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
         {
            PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            Point3d translation = new Point3d();
            planarRegion.getTransformToWorld(transformToWorld);


            Graphics3DObject boundingBoxVisualization = new Graphics3DObject();
            BoundingBox3d boundingBox3dInWorld = planarRegion.getBoundingBox3dInWorld();

            double lx = boundingBox3dInWorld.getXMax() - boundingBox3dInWorld.getXMin();
            double ly = boundingBox3dInWorld.getYMax() - boundingBox3dInWorld.getYMin();
            double lz = boundingBox3dInWorld.getZMax() - boundingBox3dInWorld.getZMin();

            transformToWorld.getTranslation(translation);
            translation.setZ(translation.z - (lz / 2.0));
            boundingBoxVisualization.translate(translation);
            boundingBoxVisualization.addCube(lx, ly, lz, YoAppearance.RGBColor(1.0, 0, 0, 0.5));

            scs.addStaticLinkGraphics(boundingBoxVisualization);
         }
      }

      if(visualizeGroundProfile)
      {
         HeightMapWithNormals heightMap = terrainObject3D.getHeightMapIfAvailable();
         Graphics3DObject heightMapGraphics = new Graphics3DObject();
         heightMapGraphics.addHeightMap(heightMap, 300, 300, YoAppearance.DarkGreen());
         scs.addStaticLinkGraphics(heightMapGraphics);
      }

      if(visualizeTerrainLinkGraphics)
      {
         scs.addStaticLinkGraphics(terrainObject3D.getLinkGraphics());
      }

      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
//      Random random = new Random(1776L);

      PlanarRegionsList planarRegionsList = PlanarRegionsListExamples.generateCinderBlockField();

      startEnvironmentVisualizationForPlanarRegionsList(planarRegionsList, true, true, true, true);
   }
}
