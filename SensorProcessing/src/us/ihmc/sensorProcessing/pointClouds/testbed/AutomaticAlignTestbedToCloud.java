package us.ihmc.sensorProcessing.pointClouds.testbed;

import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

import java.awt.Dimension;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.List;

import javax.swing.JPanel;

import boofcv.gui.image.ShowImages;
import bubo.clouds.FactoryFitting;
import bubo.clouds.fit.MatchCloudToCloud;
import bubo.struct.StoppingCondition;

import com.thoughtworks.xstream.XStream;

/**
 * @author Peter Abeles
 */
public class AutomaticAlignTestbedToCloud {


   public static void main(String[] args) {

      String directory = "../SensorProcessing/data/testbed/2014-07-10/";

      System.out.println("Loading and filtering point clouds");
      List<Point3D_F64> cloudReference = CreateCloudFromFilteredScanApp.loadFilteredScans(directory+"cloud01_scans.txt",4);
      List<Point3D_F64> cloudTarget = CreateCloudFromFilteredScanApp.loadFilteredScans(directory+"cloud02_scans.txt",4);

      Se3_F64 testbedToReference;
      try {
         testbedToReference = (Se3_F64)new XStream().fromXML(new FileInputStream(directory+"testbedToWorld01.xml"));
      } catch (FileNotFoundException e) {
         throw new RuntimeException(e);
      }

      System.out.println("Running ICP");
      long beforeICP = System.currentTimeMillis();
      MatchCloudToCloud<Se3_F64,Point3D_F64> fit = FactoryFitting.cloudIcp3D(1, new StoppingCondition(20, 1e-4, 1e-6));

      fit.setSource(cloudReference);
      fit.setDestination(cloudTarget);

      if( !fit.compute() )
         throw new RuntimeException("ICP failed!");

      System.out.println("Total time ICP "+(System.currentTimeMillis()-beforeICP)/1000.0);

      Se3_F64 refToTarget = fit.getSourceToDestination();

      System.out.println("refToTarget");
      System.out.println(refToTarget);


      Se3_F64 testbedToTarget = testbedToReference.concat(refToTarget,null);

      System.out.println("Displaying results");
      ManualAlignTestbedToCloud display = new ManualAlignTestbedToCloud();
      display.addPoints(cloudTarget,0xFF0000,1);
      display.addTestBedModel();
      display.setTestbedToWorld(testbedToTarget);

      JPanel gui = new JPanel();
      gui.add( display.getCanvas() );
      gui.setPreferredSize( new Dimension(800,800));

      ShowImages.showWindow(gui, "Automatic Alignment");
      gui.requestFocus();
   }
}
