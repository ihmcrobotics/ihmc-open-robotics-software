package us.ihmc.footstepPlanning.ui;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import controller_msgs.msg.dds.HeightMapMessage;
import controller_msgs.msg.dds.HeightMapMessagePubSubType;
import javafx.stage.FileChooser;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;

public class HeightMapSnapperVisualizer
{
   public HeightMapSnapperVisualizer()
   {
      JFileChooser fileChooser = new JFileChooser();
      fileChooser.setDialogTitle("Import Height Map");
      fileChooser.setFileFilter(new FileNameExtensionFilter("JSON log", "json"));
      fileChooser.setCurrentDirectory(new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs"));
      int chooserState = fileChooser.showOpenDialog(null);
      if (chooserState != JFileChooser.APPROVE_OPTION)
         return;

      ObjectMapper objectMapper = new ObjectMapper();
      HeightMapMessage heightMapMessage;
      try
      {
         JSONSerializer<HeightMapMessage> serializer = new JSONSerializer<>(new HeightMapMessagePubSubType());
         InputStream inputStream = new FileInputStream(fileChooser.getSelectedFile());
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
      polygon.addVertex(0.5, 0.5);
      polygon.addVertex(0.5, -0.5);
      polygon.addVertex(-0.5, 0.5);
      polygon.addVertex(-0.5, -0.5);
      polygon.update();

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(buildHeightMapGraphics(heightMapData));
      scs.addStaticLinkGraphics(buildSnapGraphics(heightMapData, polygon, new Pose2D(1.2, 0.0, 0.0)));

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   private static Graphics3DObject buildHeightMapGraphics(HeightMapData heightMapData)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.addCoordinateSystem(0.3);

      double minHeight = heightMapData.minHeight();
      double zeroHeight = minHeight - 0.2;
      AppearanceDefinition olive = YoAppearance.Olive();
      olive.setTransparency(0.7);

      for (int i = 0; i < heightMapData.getNumberOfCells(); i++)
      {
         Point2D cellPosition = heightMapData.getCellPosition(i);
         double height = heightMapData.getHeight(i);
         double renderedHeight = (height - zeroHeight);

         graphics3DObject.identity();
         graphics3DObject.translate(cellPosition.getX(), cellPosition.getY(), zeroHeight + 0.5 * renderedHeight);
         graphics3DObject.addCube(heightMapData.getGridResolutionXY(), heightMapData.getGridResolutionXY(), renderedHeight, true, olive);
      }

      return graphics3DObject;
   }

   private static Graphics3DObject buildSnapGraphics(HeightMapData heightMapData, ConvexPolygon2D polygon, Pose2D... poses)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();

      for (int i = 0; i < poses.length; i++)
      {
         Pose2D pose = poses[i];

         ConvexPolygon2D polygonToSnap = new ConvexPolygon2D(polygon);
         RigidBodyTransform stepTransform = new RigidBodyTransform();
         stepTransform.getTranslation().set(pose.getX(), pose.getY(), 0.0);
         stepTransform.getRotation().setToYawOrientation(pose.getYaw());
         polygonToSnap.applyTransform(stepTransform);

         RigidBodyTransform snapTransform = snapper.snapPolygonToHeightMap(polygonToSnap, heightMapData);
         snapTransform.transform(stepTransform);

         graphics3DObject.identity();
         graphics3DObject.transform(stepTransform);
         System.out.println(stepTransform);
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
