package us.ihmc.imageProcessing.segmentation;

import java.io.FileInputStream;
import java.io.IOException;

import boofcv.io.UtilIO;

/**
 * @author Peter Abeles
 */
public class CreateGaussianModelApp {
   public static void main( String args[] ) throws IOException {
      Gaussian3D_F64 model;
      String roadName = "road.txt";
      String lineName = "line.txt";
//      String coneBlack = "cone_black.txt";
//      String coneOrange = "cone_orange.txt";
//
//      System.out.println("Loading model");
//      model = GaussianColorClassifier.train(new FileInputStream(roadName));
//      model.print();
//
//      BoofMiscOps.saveXML(model,"gaussian_road.xml");
//
//      model = GaussianColorClassifier.train(new FileInputStream(lineName));
//      model.print();
//
//      BoofMiscOps.saveXML(model,"gaussian_line.xml");
//
//      model = GaussianColorClassifier.train(new FileInputStream("line_far.txt"));
//      model.print();
//
//      BoofMiscOps.saveXML(model,"gaussian_line_far.xml");

//      model = GaussianColorClassifier.train(new FileInputStream("car_color.txt"));
//      model.print();
//
//      BoofMiscOps.saveXML(model,"gaussian_car_color.xml");

//      model = GaussianColorClassifier.train(new FileInputStream("car_color2.txt"));
//      model.print();
//
//      BoofMiscOps.saveXML(model,"gaussian_car_color2.xml");
//
//      model = GaussianColorClassifier.train(new FileInputStream(coneBlack));
//      model.print();
//
//      BoofMiscOps.saveXML(model,"gaussian_cone_black.xml");
//
//      model = GaussianColorClassifier.train(new FileInputStream(coneOrange));
//      model.print();
//
//      BoofMiscOps.saveXML(model,"gaussian_cone_orange.xml");
//
//      model = GaussianColorClassifier.train(new FileInputStream("white_start.txt"));
//      model.print();
//
//      BoofMiscOps.saveXML(model,"gaussian_white_start.xml");
//
      model = GaussianColorClassifier.train(new FileInputStream("hood.txt"));
      model.print();

      UtilIO.saveXML(model, "gaussian_hood.xml");
   }
}
