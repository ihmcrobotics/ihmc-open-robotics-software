package us.ihmc.sensorProcessing.pointClouds;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.thoughtworks.xstream.XStream;

import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;

/**
 * @author Peter Abeles
 */
public class GeometryOps {

//   public static List<Point3D_F64> loadCloud( String fileName )  {
//      InputStream in = null;
//      try {
//         in = new FileInputStream(fileName);
//      } catch (FileNotFoundException e) {
//         throw new RuntimeException(e);
//      }
//      ReadCsvObject<Point3D_F64> reader = new ReadCsvObject<Point3D_F64>(in, Point3D_F64.class,"x","y","z");
//
//      List<Point3D_F64> cloud = new ArrayList<Point3D_F64>();
//
//      while( true ) {
//         try {
//            Point3D_F64 p = reader.nextObject(null);
//            if (p == null)
//               break;
//            else {
//               cloud.add(p);
//            }
//         } catch (IOException e) {
//            e.printStackTrace();
//         }
//      }
//
//      return cloud;
//   }

//   public static List<List<Point3D_F64>> loadScanLines( String fileName )  {
//      InputStream in = null;
//      try {
//         in = new FileInputStream(fileName);
//      } catch (FileNotFoundException e) {
//         throw new RuntimeException(e);
//      }
//
//      ReadCsv reader = new ReadCsv(in);
//      reader.setComment('#');
//
//      List<List<Point3D_F64>> ret = new ArrayList<List<Point3D_F64>>();
//      while( true ) {
//         try {
//            List<String> words = reader.extractWords();
//            if( words == null )
//               break;
//            int total = Integer.parseInt(words.get(0));
//            List<Point3D_F64> list = new ArrayList<Point3D_F64>();
//            if( total*3+1 != words.size() ) {
//               throw new RuntimeException("Unexpected number of words");
//            } else {
//               for (int i = 1; i < words.size(); i += 3 ) {
//                  double x = Double.parseDouble(words.get(i));
//                  double y = Double.parseDouble(words.get(i+1));
//                  double z = Double.parseDouble(words.get(i+2));
//                  list.add( new Point3D_F64(x,y,z));
//               }
//            }
//            ret.add(list);
//         } catch (IOException e) {
//            throw new RuntimeException(e);
//         }
//      }
//
//      return ret;
//   }

   public static void saveCsv( Se3_F64 a , String fileName ) {
      try {
         PrintStream out = new PrintStream(fileName);

         out.printf("%.12f %.12f %.12f %.12f\n",a.R.get(0,0),a.R.get(0,1),a.R.get(0,2),a.T.x);
         out.printf("%.12f %.12f %.12f %.12f\n",a.R.get(1,0),a.R.get(1,1),a.R.get(1,2),a.T.y);
         out.printf("%.12f %.12f %.12f %.12f\n",a.R.get(2,0),a.R.get(2,1),a.R.get(2,2),a.T.z);
         out.println("0 0 0 1");
         out.close();

      } catch (FileNotFoundException e) {
         throw new RuntimeException(e);
      }
   }
//
//   public static Se3_F64 loadCsvSe3( String fileName ) {
//      try {
//         ReadCsv reader = new ReadCsv(new FileInputStream(fileName));
//
//         Se3_F64 ret = new Se3_F64();
//
//         List<String> words = reader.extractWords();
//         ret.R.set(0,0, Double.parseDouble(words.get(0)));
//         ret.R.set(0,1, Double.parseDouble(words.get(1)));
//         ret.R.set(0,2, Double.parseDouble(words.get(2)));
//         ret.T.x =  Double.parseDouble(words.get(3));
//
//         words = reader.extractWords();
//         ret.R.set(1,0, Double.parseDouble(words.get(0)));
//         ret.R.set(1,1, Double.parseDouble(words.get(1)));
//         ret.R.set(1,2, Double.parseDouble(words.get(2)));
//         ret.T.y =  Double.parseDouble(words.get(3));
//
//         words = reader.extractWords();
//         ret.R.set(2,0, Double.parseDouble(words.get(0)));
//         ret.R.set(2,1, Double.parseDouble(words.get(1)));
//         ret.R.set(2,2, Double.parseDouble(words.get(2)));
//         ret.T.z = Double.parseDouble(words.get(3));
//
//         reader.getReader().close();
//         return ret;
//      } catch (IOException e) {
//         throw new RuntimeException(e);
//      }
//   }

   public static Transform convert( Se3_F64 input , Transform output ) {
      if( output == null )
         output = new Transform();

      Quaternion_F64 quatGR = new Quaternion_F64();

      ConvertRotation3D_F64.matrixToQuaternion(input.getR(), quatGR);

      Quaternion quatJME = new Quaternion();
      quatJME.set((float)quatGR.x,(float)quatGR.y,(float)quatGR.z,(float)quatGR.w);

      output.setRotation(quatJME);
      output.setTranslation((float)input.T.x,(float)input.T.y,(float)input.T.z);

      return output;
   }

   public static Se3_F64 convert( Transform input , Se3_F64 output ) {

      if( output == null )
         output = new Se3_F64();

      Quaternion quatJME = input.getRotation();
      Vector3f tranJME = input.getTranslation();

      Quaternion_F64 quatGR = new Quaternion_F64();
      quatGR.x = quatJME.getX();
      quatGR.y = quatJME.getY();
      quatGR.z = quatJME.getZ();
      quatGR.w = quatJME.getW();

      ConvertRotation3D_F64.quaternionToMatrix(quatGR, output.getR());
      output.getT().set(tranJME.x,tranJME.y,tranJME.z);

      try {
         new XStream().toXML(output,new FileOutputStream("testbedToWorld.xml"));
      } catch (FileNotFoundException e) {
         throw new RuntimeException(e);
      }

      return output;
   }

   public static Point3D_F64 loadLocation( String fileName ) {
      try {
         BufferedReader input = new BufferedReader(new FileReader(fileName));

         input.readLine();
         String a[] = input.readLine().split("\\s");

         String words[] = new String[3];
         int where = 0;
         for( String b : a ) {
            if( b.length() != 0 ) {
               words[where++] = b;
            }
         }

         Point3D_F64 p = new Point3D_F64();
         p.x = Double.parseDouble(words[0]);
         p.y = Double.parseDouble(words[1]);
         p.z = Double.parseDouble(words[2]);

         return p;

      } catch (FileNotFoundException e) {
         return new Point3D_F64();
      } catch (IOException e) {
         return new Point3D_F64();
      }
   }
}
