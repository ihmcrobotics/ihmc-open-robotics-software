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

      EuclidCoreTestTools.assertEquals(EuclidCoreTestMissingTools.newRigidBodyTransformFromString(""" 
                                                            0.9900332889206207 -0.0894177463594257 0.1088051169064211 | 0.1000000000000000
                                                            0.0993346653975306 0.9910282997404070 -0.0894177463594257 | 0.1000000000000000
                                                            -0.0998334166468282 0.0993346653975306 0.9900332889206207 | 0.1000000000000000
                                                            0.0000000000000000 0.0000000000000000 0.0000000000000000 | 1.0000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);
      unfilteredRigidBodyTransform.getTranslation().add(new Point3D(0.1, 0.1, 0.1));
      unfilteredRigidBodyTransform.getRotation().append(new YawPitchRoll(0.1, 0.1, 0.1));

      filteredRigidBodyTransform.update(unfilteredRigidBodyTransform);

      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(filteredRigidBodyTransform));

      EuclidCoreTestTools.assertEquals(EuclidCoreTestMissingTools.newRigidBodyTransformFromString(""" 
                                                            0.9879463963435010 -0.0977116814683914 0.1200597570233632 | 0.1100000000000000
                                                            0.1097050673132180 0.9891497487794362 -0.0977116814683914 | 0.1100000000000000
                                                            -0.1092095058027995 0.1097050673132180 0.9879463963435011 | 0.1100000000000000
                                                            0.0000000000000000 0.0000000000000000 0.0000000000000000 | 1.0000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);
      unfilteredRigidBodyTransform.getTranslation().add(new Point3D(0.1, 0.1, 0.1));
      unfilteredRigidBodyTransform.getRotation().append(new YawPitchRoll(0.1, 0.1, 0.1));

      filteredRigidBodyTransform.update(unfilteredRigidBodyTransform);

      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(filteredRigidBodyTransform));

      EuclidCoreTestTools.assertEquals(EuclidCoreTestMissingTools.newRigidBodyTransformFromString(""" 
                                                            0.9834409873207434 -0.1131163390455875 0.1415927904185955 | 0.1290000000000000
                                                            0.1295926256343390 0.9850941301328119 -0.1131163390455875 | 0.1290000000000000
                                                            -0.1266869205514076 0.1295926256343390 0.9834409873207435 | 0.1290000000000000
                                                            0.0000000000000000 0.0000000000000000 0.0000000000000000 | 1.0000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);
      unfilteredRigidBodyTransform.getTranslation().add(new Point3D(0.1, 0.1, 0.1));
      unfilteredRigidBodyTransform.getRotation().append(new YawPitchRoll(0.1, 0.1, 0.1));

      filteredRigidBodyTransform.update(unfilteredRigidBodyTransform);

      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(filteredRigidBodyTransform));

      EuclidCoreTestTools.assertEquals(EuclidCoreTestMissingTools.newRigidBodyTransformFromString(""" 
                                                            0.9757980801204758 -0.1342587634073139 0.1726055945835227 | 0.1561000000000000
                                                            0.1583397744951649 0.9782142404714618 -0.1342587634073140 | 0.1561000000000000
                                                            -0.1508198350549846 0.1583397744951649 0.9757980801204760 | 0.1561000000000000
                                                            0.0000000000000000 0.0000000000000000 0.0000000000000000 | 1.0000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);
      unfilteredRigidBodyTransform.getTranslation().add(new Point3D(0.1, 0.1, 0.1));
      unfilteredRigidBodyTransform.getRotation().append(new YawPitchRoll(0.1, 0.1, 0.1));

      filteredRigidBodyTransform.update(unfilteredRigidBodyTransform);

      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(filteredRigidBodyTransform));

      EuclidCoreTestTools.assertEquals(EuclidCoreTestMissingTools.newRigidBodyTransformFromString(""" 
                                                            0.9640634283712346 -0.1596208716326060 0.2123743944460991 | 0.1904900000000000
                                                            0.1953779100890390 0.9676510990995081 -0.1596208716326060 | 0.1904900000000000
                                                            -0.1800254935456074 0.1953779100890391 0.9640634283712346 | 0.1904900000000000
                                                            0.0000000000000000 0.0000000000000000 0.0000000000000000 | 1.0000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);
   }

   @Test
   public void testReset()
   {
      Random random = new Random(3453456);

      double alpha = random.nextDouble();

      AlphaFilteredRigidBodyTransform filteredRigidBodyTransform = new AlphaFilteredRigidBodyTransform();
      filteredRigidBodyTransform.setAlpha(0.9);

      RigidBodyTransform unfilteredRigidBodyTransform = new RigidBodyTransform();

      unfilteredRigidBodyTransform.getTranslation().add(new Point3D(-0.1, 0.1, -0.1));
      unfilteredRigidBodyTransform.getRotation().append(new YawPitchRoll(0.1, -0.1, 0.1));
      filteredRigidBodyTransform.update(unfilteredRigidBodyTransform);
      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(filteredRigidBodyTransform));
      EuclidCoreTestTools.assertEquals(EuclidCoreTestMissingTools.newRigidBodyTransformFromString(""" 
                                                            0.990033288920620700 -0.109251584435635540 -0.088871694747662740 | -0.100000000000000000
                                                            0.099334665397530610 0.989038278100834400 -0.109251584435635540 | 0.100000000000000000
                                                            0.099833416646828150 0.099334665397530610 0.990033288920620700 | -0.100000000000000000
                                                            0.000000000000000000 0.000000000000000000 0.000000000000000000 | 1.000000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);

      unfilteredRigidBodyTransform.getTranslation().add(new Point3D(-0.1, 0.1, -0.1));
      unfilteredRigidBodyTransform.getRotation().append(new YawPitchRoll(0.1, -0.1, 0.1));
      filteredRigidBodyTransform.update(unfilteredRigidBodyTransform);
      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(filteredRigidBodyTransform));
      EuclidCoreTestTools.assertEquals(EuclidCoreTestMissingTools.newRigidBodyTransformFromString(""" 
                                                            0.987946818678216400 -0.120594329665051940 -0.097046850109766770 | -0.110000000000000010
                                                            0.108601364045026250 0.986743508405399000 -0.120594329665051990 | 0.110000000000000010
                                                            0.110303341704367470 0.108601364045026240 0.987946818678216500 | -0.110000000000000010
                                                            0.000000000000000000 0.000000000000000000 0.000000000000000000 | 1.000000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);

      filteredRigidBodyTransform.reset();

      unfilteredRigidBodyTransform.getTranslation().add(new Point3D(-0.1, 0.1, -0.1));
      unfilteredRigidBodyTransform.getRotation().append(new YawPitchRoll(0.1, -0.1, 0.1));
      filteredRigidBodyTransform.update(unfilteredRigidBodyTransform);
      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(filteredRigidBodyTransform));
      EuclidCoreTestTools.assertEquals(EuclidCoreTestMissingTools.newRigidBodyTransformFromString(""" 
                                                            0.912137624808415300 -0.343802033741961170 -0.223170596189895150 | -0.300000000000000040
                                                            0.256378604455114070 0.903366023698333700 -0.343802033741961170 | 0.300000000000000040
                                                            0.319804572491560860 0.256378604455114070 0.912137624808415400 | -0.300000000000000040
                                                            0.000000000000000000 0.000000000000000000 0.000000000000000000 | 1.000000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);

      unfilteredRigidBodyTransform.getTranslation().add(new Point3D(-0.1, 0.1, -0.1));
      unfilteredRigidBodyTransform.getRotation().append(new YawPitchRoll(0.1, -0.1, 0.1));
      filteredRigidBodyTransform.update(unfilteredRigidBodyTransform);
      System.out.println(EuclidCoreTestMissingTools.toStringFullPrecision(filteredRigidBodyTransform));
      EuclidCoreTestTools.assertEquals(EuclidCoreTestMissingTools.newRigidBodyTransformFromString(""" 
                                                            0.906330827001449200 -0.355699718189048300 -0.228127469865204740 | -0.310000000000000050
                                                            0.262498500897342700 0.896979513426520900 -0.355699718189048330 | 0.310000000000000050
                                                            0.331147956438683330 0.262498500897342700 0.906330827001449300 | -0.310000000000000050
                                                            0.000000000000000000 0.000000000000000000 0.000000000000000000 | 1.000000000000000000
                                                                  """), filteredRigidBodyTransform, EPSILON);
   }
}
