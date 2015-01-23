package us.ihmc.imageProcessing.segmentation;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

/**
 * Writes and reads labeled color values from the image
 *
 * @author Peter Abeles
 */
public class LabeledPixelCodec {

   /**
    * Saves the labeled colors into a human readable format
    */
   public static void write( OutputStream stream, String label , String colorModel , List<double[]> values )
           throws IOException
   {
      PrintStream out = new PrintStream(stream);

      out.println("# First Line: <label> <color model>.  Others: <band 0> <...> <band N-1>");
      out.println(label+" "+colorModel);
      for( int i = 0; i < values.size(); i++ ) {
         for( double d : values.get(i))
            out.print(d+" ");
         out.println();
      }
      stream.close();
   }

   /**
    * Reads the human readable format
    */
   public static Set read( InputStream stream )
           throws IOException

   {
      BufferedReader reader = new BufferedReader(new InputStreamReader(stream));

      Set set = new Set();

      while( true ) {
         String s = reader.readLine();
         if( s.charAt(0) != '#') {
            String[] w = s.split(" ");
            set.label = w[0];
            set.colorModel = w[1];
            break;
         }
      }

      while( true ) {
         String s = reader.readLine();
         if( s == null )
            break;
         String[] w = s.split(" ");
         double d[] = new double[w.length];
         for( int i = 0; i < w.length; i++ ) {
            d[i] = Double.parseDouble(w[i]);
         }
         set.values.add(d);
      }

      return set;
   }

   public static class Set {
      String label;
      String colorModel;
      List<double[]> values = new ArrayList<double[]>();
   }
}
