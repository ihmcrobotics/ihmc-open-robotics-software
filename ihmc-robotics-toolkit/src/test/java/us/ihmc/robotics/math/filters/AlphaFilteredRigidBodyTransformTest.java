package us.ihmc.robotics.math.filters;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.EuclidCoreTestMissingTools;

import java.util.Random;

public class AlphaFilteredRigidBodyTransformTest
{
   private static final double EPSILON = 1.0e-15;

   @Test
   public void testRegression()
   {
      Random random = new Random(3453456);

      double alpha = random.nextDouble();

      AlphaFilteredRigidBodyTransform filteredRigidBodyTransform = new AlphaFilteredRigidBodyTransform();
      filteredRigidBodyTransform.setAlpha(0.9);

      RigidBodyTransform unfilteredRigidBodyTransform = new RigidBodyTransform();

      unfilteredRigidBodyTransform.getTranslation().add(new Point3D(0.1, 0.1, 0.1));
      unfilteredRigidBodyTransform.getRotation().append(new YawPitchRoll(0.1, 0.1, 0.1));

      filteredRigidBodyTransform.update(unfilteredRigidBodyTransform);

      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(filteredRigidBodyTransform));

      EuclidCoreTestTools.assertEquals(parseStringToDoubles(""" 
                                                            0.9900332889206207 -0.0894177463594257 0.1088051169064211 | 0.1000000000000000
                                                            0.0993346653975306 0.9910282997404070 -0.0894177463594257 | 0.1000000000000000
                                                            -0.0998334166468282 0.0993346653975306 0.9900332889206207 | 0.1000000000000000
                                                            0.0000000000000000 0.0000000000000000 0.0000000000000000 | 1.0000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);
      unfilteredRigidBodyTransform.getTranslation().add(new Point3D(0.1, 0.1, 0.1));
      unfilteredRigidBodyTransform.getRotation().append(new YawPitchRoll(0.1, 0.1, 0.1));

      filteredRigidBodyTransform.update(unfilteredRigidBodyTransform);

      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(filteredRigidBodyTransform));

      EuclidCoreTestTools.assertEquals(parseStringToDoubles(""" 
                                                            0.9879463963435010 -0.0977116814683914 0.1200597570233632 | 0.1100000000000000
                                                            0.1097050673132180 0.9891497487794362 -0.0977116814683914 | 0.1100000000000000
                                                            -0.1092095058027995 0.1097050673132180 0.9879463963435011 | 0.1100000000000000
                                                            0.0000000000000000 0.0000000000000000 0.0000000000000000 | 1.0000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);
      unfilteredRigidBodyTransform.getTranslation().add(new Point3D(0.1, 0.1, 0.1));
      unfilteredRigidBodyTransform.getRotation().append(new YawPitchRoll(0.1, 0.1, 0.1));

      filteredRigidBodyTransform.update(unfilteredRigidBodyTransform);

      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(filteredRigidBodyTransform));

      EuclidCoreTestTools.assertEquals(parseStringToDoubles(""" 
                                                            0.9834409873207434 -0.1131163390455875 0.1415927904185955 | 0.1290000000000000
                                                            0.1295926256343390 0.9850941301328119 -0.1131163390455875 | 0.1290000000000000
                                                            -0.1266869205514076 0.1295926256343390 0.9834409873207435 | 0.1290000000000000
                                                            0.0000000000000000 0.0000000000000000 0.0000000000000000 | 1.0000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);
      unfilteredRigidBodyTransform.getTranslation().add(new Point3D(0.1, 0.1, 0.1));
      unfilteredRigidBodyTransform.getRotation().append(new YawPitchRoll(0.1, 0.1, 0.1));

      filteredRigidBodyTransform.update(unfilteredRigidBodyTransform);

      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(filteredRigidBodyTransform));

      EuclidCoreTestTools.assertEquals(parseStringToDoubles(""" 
                                                            0.9757980801204758 -0.1342587634073139 0.1726055945835227 | 0.1561000000000000
                                                            0.1583397744951649 0.9782142404714618 -0.1342587634073140 | 0.1561000000000000
                                                            -0.1508198350549846 0.1583397744951649 0.9757980801204760 | 0.1561000000000000
                                                            0.0000000000000000 0.0000000000000000 0.0000000000000000 | 1.0000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);
      unfilteredRigidBodyTransform.getTranslation().add(new Point3D(0.1, 0.1, 0.1));
      unfilteredRigidBodyTransform.getRotation().append(new YawPitchRoll(0.1, 0.1, 0.1));

      filteredRigidBodyTransform.update(unfilteredRigidBodyTransform);

      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(filteredRigidBodyTransform));

      EuclidCoreTestTools.assertEquals(parseStringToDoubles(""" 
                                                            0.9640634283712346 -0.1596208716326060 0.2123743944460991 | 0.1904900000000000
                                                            0.1953779100890390 0.9676510990995081 -0.1596208716326060 | 0.1904900000000000
                                                            -0.1800254935456074 0.1953779100890391 0.9640634283712346 | 0.1904900000000000
                                                            0.0000000000000000 0.0000000000000000 0.0000000000000000 | 1.0000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);
   }

   public static RigidBodyTransform parseStringToDoubles(String input)
   {
      String[] lines = input.split("\\R"); // Split the input by newlines [3]
      String[] tokens;
      double[] doubles = new double[lines.length * 4]; // Initialize an array to store the doubles
      int index = 0;

      for (String line : lines)
      {
         line = line.replace("|", "").trim(); // Remove the '|' character and trim whitespace
         tokens = line.split("\\s+"); // Split the line by whitespace [2]

         for (String token : tokens)
         {
            doubles[index++] = Double.parseDouble(token); // Convert the token to a double and store it in the array [1]
         }
      }

      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      return new RigidBodyTransform(doubles);
   }
}
