package us.ihmc.footstepPlanning;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.footstepPlanning.bodyPath.HeightMapRANSACNormalCalculator;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.pathPlanning.HeightMapDataSetName;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapPlanarRegionCalculator;
import us.ihmc.perception.gpuHeightMap.HeightMapTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.File;
import java.util.function.IntFunction;

public class HeightMapDataVisualizer
{
   private static final boolean SHOW_NORMALS = false;
   private static final boolean COLOR_REGIONS = true;

   private static final boolean MARK_A_CELL = false;
   private static final int indexXToMark = 80;
   private static final int indexYToMark = 80;

   private static final AppearanceDefinition defaultColor = YoAppearance.Olive();

   public HeightMapDataVisualizer()
   {
      HeightMapData heightMapData = HeightMapDataSetName.Stairs_1.getHeightMapData();
//      HeightMapData heightMapData = HeightMapDataSetName.Stepping_Stones_3.getHeightMapData();;
//      HeightMapData heightMapData = HeightMapDataSetName.Obstacles_2.getHeightMapData();

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(0.11, 0.043);
      polygon.addVertex(0.11, -0.043);
      polygon.addVertex(-0.11, 0.055);
      polygon.addVertex(-0.11, -0.055);
      polygon.update();

      long t0 = System.nanoTime();

//      HeightMapLeastSquaresNormalCalculator lsSurfaceNormalCalculator = new HeightMapLeastSquaresNormalCalculator();
//      lsSurfaceNormalCalculator.computeSurfaceNormals(heightMapData, 0.4);

      HeightMapRANSACNormalCalculator surfaceNormalCalculator = new HeightMapRANSACNormalCalculator();
      surfaceNormalCalculator.initialize(heightMapData);

      for (int xi = 0; xi < heightMapData.getCellsPerAxis(); xi++)
      {
         for (int yi = 0; yi < heightMapData.getCellsPerAxis(); yi++)
         {
            surfaceNormalCalculator.getSurfaceNormal(xi, yi);
         }
      }

      long t1 = System.nanoTime();
      System.out.println("Surface normal calculation: " + Conversions.nanosecondsToSeconds(t1 - t0) + " sec");

      HeightMapPlanarRegionCalculator planarRegionCalculator = new HeightMapPlanarRegionCalculator();
      planarRegionCalculator.computeRegions(heightMapData, surfaceNormalCalculator::getSurfaceNormal);

      long t2 = System.nanoTime();
      System.out.println("Planar region calculation:  " + Conversions.nanosecondsToSeconds(t2 - t1) + " sec");

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(buildHeightMapGraphics(heightMapData, surfaceNormalCalculator::getSurfaceNormal, planarRegionCalculator));

      // good values
//      scs.addStaticLinkGraphics(buildSnapGraphics(heightMapData,
//                                                  polygon,
//                                                  new Pose2D(1.35, 0.25, -0.9),
//                                                  new Pose2D(1.7, 0.7, 0.4),
//                                                  new Pose2D(2.0, 0.3, -0.8),
//                                                  new Pose2D(2.5, 0.7, 1.5),
//                                                  new Pose2D(2.65, 0.15, 0.0),
//                                                  new Pose2D(1.28, 0.25, 0.8),
//                                                  new Pose2D(1.68, 0.7, -1.2),
//                                                  new Pose2D(2.05, 0.36, 0.8),
//                                                  new Pose2D(2.38, 0.7, 1.5),
//                                                  new Pose2D(2.7, 0.26, 0.0)
//                                                              ));

      // bad values
//      scs.addStaticLinkGraphics(buildSnapGraphics(heightMapData,
//                                                  polygon,
//                                                  new Pose2D(1.35, 0.0, 1.6),
//                                                  new Pose2D(1.84, 0.7, 0.4),
//                                                  new Pose2D(1.91, 0.3, -0.8),
//                                                  new Pose2D(2.5, 0.57, 1.5),
//                                                  new Pose2D(2.52, 0.28, 0.0),
//
//                                                  new Pose2D(1.2, 0.38, 0.8),
//                                                  new Pose2D(1.5, 0.7, -1.2),
//                                                  new Pose2D(1.8, 0.5, 0.8),
//                                                  new Pose2D(2.22, 0.55, 1.5)
//                                                                    ));

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   private File loadThroughChooser()
   {
      JFileChooser fileChooser = new JFileChooser();
      fileChooser.setDialogTitle("Import Height Map");
      fileChooser.setFileFilter(new FileNameExtensionFilter("JSON log", "json"));
      fileChooser.setCurrentDirectory(new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs"));
      int chooserState = fileChooser.showOpenDialog(null);
      if (chooserState != JFileChooser.APPROVE_OPTION)
         return null;
      return fileChooser.getSelectedFile();
   }

   private static Graphics3DObject buildHeightMapGraphics(HeightMapData heightMapData,
                                                          IntFunction<UnitVector3DReadOnly> surfaceNormalCalculator,
                                                          HeightMapPlanarRegionCalculator planarRegionCalculator)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();

      graphics3DObject.translate(heightMapData.getGridCenter().getX(), heightMapData.getGridCenter().getY(), heightMapData.getEstimatedGroundHeight());
      graphics3DObject.addCube(heightMapData.getGridSizeXY(), heightMapData.getGridSizeXY(), 0.01, YoAppearance.Blue());
      graphics3DObject.addCoordinateSystem(0.3);

      double groundPlaneHeight = heightMapData.getEstimatedGroundHeight();

      AppearanceDefinition[] colors = new AppearanceDefinition[] {YoAppearance.Blue(), YoAppearance.Green(), YoAppearance.Red(), YoAppearance.Orange(), YoAppearance.Black(), YoAppearance.White(), YoAppearance.Indigo(), YoAppearance.Brown(), YoAppearance.DarkSalmon(), YoAppearance.HotPink(), YoAppearance.LightGreen(), YoAppearance.PaleGreen(), YoAppearance.Orchid(), YoAppearance.Turquoise(), YoAppearance.Tomato(), YoAppearance.Violet(), YoAppearance.Maroon(), YoAppearance.Chartreuse()};

      int gridWidth = 2 * heightMapData.getCenterIndex() + 1;
      for (int key = 0; key < gridWidth * gridWidth; key++)
      {
         Point2D cellPosition = new Point2D(HeightMapTools.keyToXCoordinate(key,
                                                                            heightMapData.getGridCenter().getX(),
                                                                            heightMapData.getGridResolutionXY(),
                                                                            heightMapData.getCenterIndex()),
                                            HeightMapTools.keyToYCoordinate(key,
                                                                            heightMapData.getGridCenter().getY(),
                                                                            heightMapData.getGridResolutionXY(),
                                                                            heightMapData.getCenterIndex()));
         double height = heightMapData.getHeightAt(HeightMapTools.keyToXIndex(key, heightMapData.getCenterIndex()), HeightMapTools.keyToYIndex(key, heightMapData.getCenterIndex()));

         double renderedHeight = height - groundPlaneHeight;
         graphics3DObject.identity();
         graphics3DObject.translate(cellPosition.getX(), cellPosition.getY(), groundPlaneHeight + 0.5 * renderedHeight);

         if (height > groundPlaneHeight + 1e-5)
         {
            AppearanceDefinition color;
            if (MARK_A_CELL && key == HeightMapTools.indicesToKey(indexXToMark, indexYToMark, heightMapData.getCenterIndex()))
            {
               color = YoAppearance.Red();
            }
            else if (COLOR_REGIONS)
            {
               int regionId = planarRegionCalculator.getRegionId(key);
               if (regionId == -1)
                  color = defaultColor;
               else
                  color = colors[regionId % colors.length];
            }
            else
            {
               color = defaultColor;
            }

            graphics3DObject.addCube(heightMapData.getGridResolutionXY(), heightMapData.getGridResolutionXY(), renderedHeight, true, color);

            if (SHOW_NORMALS)
            {
               graphics3DObject.translate(0.0, 0.0, 0.5 * renderedHeight + 0.01);
               UnitVector3DReadOnly surfaceNormal = surfaceNormalCalculator.apply(key);

               if (!surfaceNormal.epsilonEquals(Axis3D.Z, 1e-4))
               {
                  Vector3D rotationAxis = new Vector3D();
                  rotationAxis.cross(surfaceNormal, Axis3D.Z);
                  rotationAxis.normalize();

                  double angle = -Math.acos(surfaceNormal.getZ());
                  graphics3DObject.rotate(angle, rotationAxis);
               }

               graphics3DObject.addCylinder(0.08, 0.006, YoAppearance.Orange());
            }
         }
      }

      return graphics3DObject;
   }

   private static Graphics3DObject buildSnapGraphics(HeightMapData heightMapData, ConvexPolygon2D polygon, Pose2D... poses)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      environmentHandler.setHeightMap(heightMapData);

      for (int i = 0; i < poses.length; i++)
      {
         Pose2D pose = poses[i];

         ConvexPolygon2D polygonToSnap = new ConvexPolygon2D(polygon);
         RigidBodyTransform stepTransform = new RigidBodyTransform();
         stepTransform.getTranslation().set(pose.getX(), pose.getY(), 0.0);
         stepTransform.getRotation().setToYawOrientation(pose.getYaw());
         polygonToSnap.applyTransform(stepTransform);

         graphics3DObject.identity();
         graphics3DObject.transform(stepTransform);
         graphics3DObject.translate(0.0, 0.0, 0.1);
         graphics3DObject.addExtrudedPolygon(polygon, 0.02, YoAppearance.Glass());

         RigidBodyTransform snapTransform = snapper.snapPolygonToHeightMap(polygonToSnap,
                                                                           environmentHandler,
                                                                           parameters.getHeightMapSnapThreshold(),
                                                                           parameters.getMinimumSurfaceInclineRadians());
         snapTransform.transform(stepTransform);

         graphics3DObject.identity();
         graphics3DObject.transform(stepTransform);
         graphics3DObject.addCoordinateSystem(0.3);
         graphics3DObject.addExtrudedPolygon(polygon, 0.02, YoAppearance.Red());
      }

      return graphics3DObject;
   }

   public static void main(String[] args)
   {
      new HeightMapDataVisualizer();
   }
}
