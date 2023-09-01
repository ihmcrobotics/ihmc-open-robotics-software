package us.ihmc.robotics.geometry.concaveHull;

import static us.ihmc.euclid.tools.EuclidCoreIOTools.getTuple2DString;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;

public class GeometryPolygonTestTools
{
   private static final String DEFAULT_FORMAT = EuclidCoreTestTools.DEFAULT_FORMAT;

   public static void assertConcavePolygon2DEquals(ConcavePolygon2DReadOnly expected, ConcavePolygon2DReadOnly actual, double epsilon)
   {
      assertConcavePolygon2DEquals(null, expected, actual, epsilon);
   }

   public static void assertConcavePolygon2DEquals(String messagePrefix, ConcavePolygon2DReadOnly expected, ConcavePolygon2DReadOnly actual, double epsilon)
   {
      assertConcavePolygon2DEquals(messagePrefix, expected, actual, epsilon, DEFAULT_FORMAT);
   }

   public static void assertConcavePolygon2DEquals(String messagePrefix, ConcavePolygon2DReadOnly expected, ConcavePolygon2DReadOnly actual, double epsilon,
                                                  String format)
   {
      if (expected == null && actual == null)
         return;

      if (!(expected != null && actual != null))
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);

      if (!geometricallyEquals(expected, actual, epsilon))
      {
         throwNotEqualAssertionError(messagePrefix, expected, actual, format);
      }
   }

   private static boolean geometricallyEquals(ConcavePolygon2DReadOnly expected, ConcavePolygon2DReadOnly actual, double epsilon)
   {
      if (actual == expected)
         return true;
      if (actual == null || expected == null)
         return false;


      Vertex2DSupplier other = (Vertex2DSupplier) actual;

      if (expected.getNumberOfVertices() != other.getNumberOfVertices())
         return false;

      for (int i = 0; i < expected.getNumberOfVertices(); i++)
      {
         boolean oneEquals = false;
         for (int j = 0; j < actual.getNumberOfVertices(); j++)
         {
            if (!expected.getVertex(i).geometricallyEquals(other.getVertex(j), epsilon))
            {
               oneEquals = true;
               break;
            }
         }
         if (!oneEquals)
            return false;
      }

      return true;
   }

   private static void throwNotEqualAssertionError(String messagePrefix, ConcavePolygon2DReadOnly expected, ConcavePolygon2DReadOnly actual, String format)
   {
      String expectedAsString = getConcavePolygon2DString(format, expected);
      String actualAsString = getConcavePolygon2DString(format, actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   public static String getConcavePolygon2DString(String format, ConcavePolygon2DReadOnly convexPolygon2D)
   {
      if (convexPolygon2D == null)
         return "null";
      else
         return getConcavePolygon2DString(format, convexPolygon2D.getPolygonVerticesView(), convexPolygon2D.getNumberOfVertices());
   }

   public static String getConcavePolygon2DString(String format, List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return "Concave Polygon 2D: vertices = []";

      String string = "Concave Polygon 2D: vertices = [\n";
      for (int i = 0; i < numberOfVertices - 1; i++)
         string += getTuple2DString(format, vertices.get(i)) + ",\n";
      string += getTuple2DString(format, vertices.get(numberOfVertices - 1)) + " ]";
      return string;
   }
}
