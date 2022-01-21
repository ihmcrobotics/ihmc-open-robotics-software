package us.ihmc.footstepPlanning.ui;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import controller_msgs.msg.dds.HeightMapMessage;
import controller_msgs.msg.dds.HeightMapMessagePubSubType;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.footstepPlanning.bodyPath.HeightMapRANSACNormalCalculator;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapPlanarRegionCalculator;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.function.IntFunction;

public class HeightMapSnapperVisualizer
{
   public HeightMapSnapperVisualizer()
   {
//      File file = loadThroughChooser();
//      File file = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + "stepping_stones.json");
      File file = new File(System.getProperty("user.home") + File.separator + "heightMapDatasets" + File.separator + "stairs_1.json");
//      File file = new File(System.getProperty("user.home") + File.separator + "heightMapDatasets" + File.separator + "cinders.json");

      if (file == null)
         return;

      ObjectMapper objectMapper = new ObjectMapper();
      HeightMapMessage heightMapMessage;
      try
      {
         JSONSerializer<HeightMapMessage> serializer = new JSONSerializer<>(new HeightMapMessagePubSubType());
         InputStream inputStream = new FileInputStream(file);
         JsonNode jsonNode = objectMapper.readTree(inputStream);
         heightMapMessage = serializer.deserialize(jsonNode.toString());
         inputStream.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return;
      }

      HeightMapData heightMapData = new HeightMapData(heightMapMessage);

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(0.11, 0.043);
      polygon.addVertex(0.11, -0.043);
      polygon.addVertex(-0.11, 0.055);
      polygon.addVertex(-0.11, -0.055);
      polygon.update();

      long start = System.nanoTime();

//      HeightMapLeastSquaresNormalCalculator surfaceNormalCalculator = new HeightMapLeastSquaresNormalCalculator();
//      surfaceNormalCalculator.computeSurfaceNormals(heightMapData, 0.3);

      HeightMapRANSACNormalCalculator surfaceNormalCalculator = new HeightMapRANSACNormalCalculator();
      surfaceNormalCalculator.computeSurfaceNormals(heightMapData);

      HeightMapPlanarRegionCalculator planarRegionCalculator = new HeightMapPlanarRegionCalculator();
      planarRegionCalculator.computeRegions(heightMapData, surfaceNormalCalculator::getSurfaceNormal);

      long stop = System.nanoTime();
      System.out.println((stop - start));

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
                                                          IntFunction<UnitVector3DBasics> surfaceNormalCalculator,
                                                          HeightMapPlanarRegionCalculator planarRegionCalculator)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();

      graphics3DObject.translate(heightMapData.getGridCenter().getX(), heightMapData.getGridCenter().getY(), heightMapData.getEstimatedGroundHeight());
      graphics3DObject.addCube(heightMapData.getGridSizeXY(), heightMapData.getGridSizeXY(), 0.01, YoAppearance.Blue());
      graphics3DObject.addCoordinateSystem(0.3);

      double minHeight = heightMapData.getMinHeight();
      double groundPlaneHeight = heightMapData.getEstimatedGroundHeight();
      AppearanceDefinition low = YoAppearance.Black();
      AppearanceDefinition high = YoAppearance.Olive();

      AppearanceDefinition[] colors = new AppearanceDefinition[] {YoAppearance.Blue(), YoAppearance.Green(), YoAppearance.Red(), YoAppearance.Orange(), YoAppearance.Black(), YoAppearance.White(), YoAppearance.Indigo(), YoAppearance.Brown(), YoAppearance.DarkSalmon(), YoAppearance.HotPink(), YoAppearance.LightGreen(), YoAppearance.PaleGreen(), YoAppearance.Orchid(), YoAppearance.Turquoise(), YoAppearance.Tomato(), YoAppearance.Violet(), YoAppearance.Maroon()};

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
            double alpha = MathTools.clamp((height - minHeight) / 0.2, 0.0, 1.0);
            AppearanceDefinition interpolatedColor = YoAppearance.RGBColor(EuclidCoreTools.interpolate(low.getColor().getX(), high.getColor().getX(), alpha),
                                                                           EuclidCoreTools.interpolate(low.getColor().getY(), high.getColor().getY(), alpha),
                                                                           EuclidCoreTools.interpolate(low.getColor().getZ(), high.getColor().getZ(), alpha));

            if (key == HeightMapTools.indicesToKey(84, 80, heightMapData.getCenterIndex()))
            {
               interpolatedColor = YoAppearance.Red();
            }

//            graphics3DObject.addCube(heightMapData.getGridResolutionXY(), heightMapData.getGridResolutionXY(), renderedHeight, true, interpolatedColor);

            int regionId = planarRegionCalculator.getRegionId(key);
            graphics3DObject.addCube(heightMapData.getGridResolutionXY(), heightMapData.getGridResolutionXY(), renderedHeight, true, colors[regionId % colors.length]);
         }

//         graphics3DObject.translate(0.0, 0.0, 0.5 * renderedHeight + 0.01);
//         UnitVector3DBasics surfaceNormal = surfaceNormalCalculator.apply(key);
//
//         if (!surfaceNormal.epsilonEquals(Axis3D.Z, 1e-4))
//         {
//            Vector3D rotationAxis = new Vector3D();
//            rotationAxis.cross(surfaceNormal, Axis3D.Z);
//            rotationAxis.normalize();
//
//            double angle = -Math.acos(surfaceNormal.getZ());
//            graphics3DObject.rotate(angle, rotationAxis);
//         }

//         graphics3DObject.addCylinder(0.08, 0.006, YoAppearance.Orange());
      }

      return graphics3DObject;
   }

   private static Graphics3DObject buildSnapGraphics(HeightMapData heightMapData, ConvexPolygon2D polygon, Pose2D... poses)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();

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

         RigidBodyTransform snapTransform = snapper.snapPolygonToHeightMap(polygonToSnap, heightMapData, parameters.getHeightMapSnapThreshold());
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
      new HeightMapSnapperVisualizer();
   }
}
