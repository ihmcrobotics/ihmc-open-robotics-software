package us.ihmc.sensorProcessing.pointClouds.testbed;

import boofcv.gui.image.ShowImages;
import bubo.clouds.FactoryFitting;
import bubo.clouds.FactoryPointCloudShape;
import bubo.clouds.detect.CloudShapeTypes;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.clouds.detect.wrapper.ConfigMultiShapeRansac;
import bubo.clouds.detect.wrapper.ConfigSurfaceNormals;
import bubo.clouds.fit.MatchCloudToCloud;
import bubo.gui.FactoryVisualization3D;
import bubo.gui.UtilDisplayBubo;
import bubo.gui.d3.PointCloudPanel;
import bubo.struct.StoppingCondition;
import com.thoughtworks.xstream.XStream;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

import javax.swing.*;
import java.awt.*;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.List;

import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadScanLines;
import static us.ihmc.sensorProcessing.pointClouds.testbed.CreateCloudFromFilteredScanApp.filter;

/**
 * @author Peter Abeles
 */
public class AutomaticAlignTestbedToPlanes {


   public static void main(String[] args) {

      String directory = "../SensorProcessing/data/testbed/2014-08-01/";

      Se3_F64 estimatedToModel;
      try {
         estimatedToModel = (Se3_F64)new XStream().fromXML(new FileInputStream(directory+"estimatedToModel.xml"));
      } catch (FileNotFoundException e) {
         throw new RuntimeException(e);
      }

      // point cloud with two bricks 09

      System.out.println("Loading and filtering point clouds");
      List<List<Point3D_F64>> scans0 = loadScanLines(directory+"cloud09_scans.txt");
      List<Point3D_F64> cloud0 = filter(scans0, 3);

      System.out.println("Detecting planes");

      ConfigMultiShapeRansac configRansac = ConfigMultiShapeRansac.createDefault(500,1.2,0.025, CloudShapeTypes.PLANE);
      configRansac.minimumPoints = 5000;
//      ConfigSchnabel2007 configSchnabel = ConfigSchnabel2007.createDefault(20000,0.6,0.15,CloudShapeTypes.PLANE);

      PointCloudShapeFinder finder = FactoryPointCloudShape.ransacSingleAll(
              new ConfigSurfaceNormals(100, 0.15), configRansac);

      finder.process(cloud0,null);

      FactoryVisualization3D factory = UtilDisplayBubo.createVisualize3D();


      System.out.println("Detecting the testbed");

      Se3_F64 estimatedToWorld =  DetectTestbedSaveTrasform.findTestbed(finder.getFound());

      Se3_F64 modelToWorld = estimatedToModel.invert(null).concat(estimatedToWorld,null);

      System.out.println("Rendering results");


      ManualAlignTestbedToCloud display = new ManualAlignTestbedToCloud();
      display.addPoints(cloud0,0xFF0000,1);
      display.addTestBedModel();
      display.setTestbedToWorld(modelToWorld);

      JPanel gui = new JPanel();
      gui.add( display.getCanvas() );
      gui.setPreferredSize( new Dimension(800,800));

      ShowImages.showWindow(gui, "Automatic Alignment");
      gui.requestFocus();
   }
}
