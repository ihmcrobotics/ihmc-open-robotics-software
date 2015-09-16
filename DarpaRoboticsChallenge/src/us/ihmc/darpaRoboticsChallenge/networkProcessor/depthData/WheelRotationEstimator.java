package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;

import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import us.ihmc.darpaRoboticsChallenge.odometry.IcpCloud3D;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.GeoregressionConversionTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class WheelRotationEstimator extends IcpCloud3D
{
   private static final boolean PRINT_BEST_SECTOR = false; //GrayTODO: commit as false;
   private final double blurAngle;
   private static final boolean IGNORE_ICP_SOLVER = true;
   private List<Point3D_F64> tempPoints = new ArrayList<Point3D_F64>();
   private Point3d tempPoint = new Point3d();
   private double bestFitFraction = 0.0;
   private RigidBodyTransform tempTransformFromCurrentToRotary = new RigidBodyTransform();
   private RigidBodyTransform tempInverseTransformFromCurrentToRotary = new RigidBodyTransform();
   private RigidBodyTransform tempTransformFromRotaryToTemplate = new RigidBodyTransform();
   private RigidBodyTransform bestTransformFromCurrentToTemplate = new RigidBodyTransform();
   private Se3_F64 georegressionBestTransformFromCurrentToTemplate = new Se3_F64();
   private double outerRadiusLimit = CarLocalizerTransformKeeper.BOUNDING_OUTER_WHEEL_RADIUS - 0.015;
   private int[] polarTemplateHistogram;
   private int[] currentPolarHistogram;
   private double[] polarTemplatePDF;
   private double[] polarTemplateScoringPattern;
   private double[] blurPattern;
   private double[] currentPolarPDF;
   private final int sectors;

   public WheelRotationEstimator(double maxDistance, int maxIterations, double convergenceTol, int sectors, double blurAngle)
   {
      super(maxDistance, maxIterations, convergenceTol);
      this.polarTemplateHistogram = new int[sectors];
      this.currentPolarHistogram = new int[sectors];
      this.polarTemplatePDF = new double[sectors];
      this.currentPolarPDF = new double[sectors];
      this.polarTemplateScoringPattern = new double[sectors];
      this.blurPattern = new double[sectors];
      this.sectors = sectors;
      this.blurAngle = blurAngle;

      setupBlurPattern();
   }

   private void setupBlurPattern()
   {
      int splitPoint = sectors / 2;
      for (int i = 0; i < sectors; i++)
      {
         int mappedI = i;
         if (i > splitPoint)
            mappedI = i - sectors;
         double halfPointSectors = blurAngle * sectors / (Math.PI * 2);
         double x = mappedI / halfPointSectors;
         blurPattern[i] = 1.0 / (x * x + 1.0);

      }
   }

   public void setReference(List<Point3D_F64> reference)
   {
      super.setReference(reference);
      computePolarHistogram(polarTemplateHistogram, reference);
      updatePDFFromHistogram(polarTemplateHistogram, polarTemplatePDF);

      for (int i = 0; i < sectors; i++)
      {
         polarTemplateScoringPattern[i] = 0;
      }

      for (int i = 0; i < sectors; i++)
      {
         for (int j = 0; j < sectors; j++)
         {
            polarTemplateScoringPattern[j] += blurPattern[i] * polarTemplatePDF[(j + i) % sectors];
         }
      }

      double sum = 0;
      for (int i = 0; i < sectors; i++)
      {
         sum += polarTemplateScoringPattern[i];
      }

      double average = sum / (double) sectors;
      for (int i = 0; i < sectors; i++)
      {
         polarTemplateScoringPattern[i] -= average;
      }

   }

   private void clearHistogram(int[] histogram)
   {
      for (int i = 0; i < histogram.length; i++)
      {
         histogram[i] = 0;
      }
   }

   private void computePolarHistogram(int[] histogram, List<Point3D_F64> data)
   {
      clearHistogram(histogram);

      double limitSquared = outerRadiusLimit * outerRadiusLimit;
      for (int i = 0; i < data.size(); i++)
      {
         Point3D_F64 datum = data.get(i);
         double y, z;
         y = datum.y;
         z = datum.z;
         if (y * y + z * z < 1e-6)
            continue;
         if (y * y + z * z > limitSquared)
            continue;

         double angle = toAngle(y, z);
         int index = angleToIndex(angle);
         histogram[index]++;
      }
   }

   public static double toAngle(double y, double z)
   {
      double angle = Math.atan2(y, -z);
      angle = AngleTools.trimAngleMinusPiToPi(angle);

      return angle;
   }

   public int angleToIndex(double angle)
   {
      double trimmedAngle = AngleTools.trimAngleMinusPiToPi(angle);
      double doubleIndex = (trimmedAngle) * sectors / (2 * Math.PI);

      return (int) (Math.floor(doubleIndex) + sectors) % sectors;
   }

   public double indexToAngle(int index)
   {
      double doubleIndex = (double) index;
      double doubleSectors = (double) sectors;
      double sectorsFraction = doubleIndex / doubleSectors;

      return AngleTools.trimAngleMinusPiToPi(sectorsFraction * Math.PI * 2.0);
   }

   public boolean setCurrent(List<Point3D_F64> current)
   {
      int bestSector = computeBestSector(current);
      ensureSufficientPoints(current);
      setupRotationTransforms(bestSector);
      rotateTempPoints(current);

      bestTransformFromCurrentToTemplate.set(tempTransformFromCurrentToRotary);

      if (!IGNORE_ICP_SOLVER)
      {
         calculateBestTransformFromCurrentToTemplateUsingICPSolver();
      }

      GeoregressionConversionTools.setGeoregressionTransformFromVecmath(bestTransformFromCurrentToTemplate, georegressionBestTransformFromCurrentToTemplate);

      return true;
   }

   private void calculateBestTransformFromCurrentToTemplateUsingICPSolver()
   {
      if (super.setCurrent(tempPoints))
      {
         Se3_F64 transformFromRotaryToTemplate = super.getReferenceToCurrent();
         GeoregressionConversionTools.setVecmathTransformFromGeoregressionTransform(tempTransformFromRotaryToTemplate, transformFromRotaryToTemplate);
         bestTransformFromCurrentToTemplate.multiply(tempTransformFromCurrentToRotary, tempTransformFromRotaryToTemplate);
      }
   }

   private int computeBestSector(List<Point3D_F64> current)
   {
      computePolarHistogram(currentPolarHistogram, current);
      updatePDFFromHistogram(currentPolarHistogram, currentPolarPDF);
      double bestSectorScore = Double.MIN_VALUE;
      int bestSector = -1;
      for (int i = 0; i < sectors; i++)
      {
         double score = computeScore(i);

         if (PRINT_BEST_SECTOR)
            System.out.println("sector " + i + " has score " + score);

         if (score > bestSectorScore)
         {
            bestSectorScore = score;
            bestSector = i;
         }
      }

      if (PRINT_BEST_SECTOR)
      {
         System.out.println("best sector is " + bestSector);
         System.out.println("best angle is " + indexToAngle(bestSector));
      }

      return bestSector;
   }

   private double computeScore(int i)
   {
      double score = 0;
      for (int j = 0; j < sectors; j++)
      {
         double binBonus = currentPolarPDF[(j + i) % sectors] * (polarTemplateScoringPattern[j] * 1000);
         score += binBonus;
      }

      return score;
   }

   private void updatePDFFromHistogram(int[] hist, double[] pdf)
   {
      int count = 0;
      for (int i = 0; i < sectors; i++)
      {
         count += hist[i];
      }

      double normalizer = 1.0 / ((double) count);
      for (int i = 0; i < sectors; i++)
      {
         pdf[i] = hist[i] * normalizer;
      }
   }

   private void rotateTempPoints(List<Point3D_F64> current)
   {
      for (int j = 0; j < current.size(); j++)
      {
         Point3D_F64 point3d_F64 = current.get(j);
         tempPoint.set(point3d_F64.x, point3d_F64.y, point3d_F64.z);
         tempTransformFromCurrentToRotary.transform(tempPoint);
         tempPoints.get(j).set(tempPoint.x, tempPoint.y, tempPoint.z);
      }
   }

   private void setupRotationTransforms(int i)
   {
      tempTransformFromCurrentToRotary.setIdentity();
      tempTransformFromCurrentToRotary.rotX(indexToAngle(i));
      tempInverseTransformFromCurrentToRotary.invert(tempTransformFromCurrentToRotary);
   }

   private void ensureSufficientPoints(List<Point3D_F64> current)
   {
      if (current.size() > tempPoints.size())
      {
         for (int j = tempPoints.size() - 1; j < current.size(); j++)
         {
            tempPoints.add(new Point3D_F64());
         }
      }
   }

   public double getFitFraction()
   {
      return this.bestFitFraction;
   }

   public Se3_F64 getReferenceToCurrent()
   {
      return this.georegressionBestTransformFromCurrentToTemplate;
   }

}
