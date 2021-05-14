package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.kinematics.fourbar.FourBar;
import us.ihmc.robotics.kinematics.fourbar.FourBarAngle;
import us.ihmc.robotics.kinematics.fourbar.FourBarEdge;
import us.ihmc.robotics.kinematics.fourbar.FourBarTools;
import us.ihmc.robotics.kinematics.fourbar.FourBarVertex;
import us.ihmc.robotics.screwTheory.FourBarKinematicLoopFunctionTools.FourBarToJointConverter;

/**
 * Numerical solver that performs a binary search using the four bar range of motion as initial
 * boundaries.
 */
public class InvertedFourBarJointIKBinarySolver implements InvertedFourBarJointIKSolver
{
   private static final boolean DEBUG = false;

   private final double tolerance;
   private final int maxIterations;
   private int iterations;
   private final FourBar fourBar = new FourBar();
   private boolean useNaiveMethod = false;

   private boolean limitsInverted = false;
   private FourBarToJointConverter[] converters = null;

   public InvertedFourBarJointIKBinarySolver(double tolerance)
   {
      this(tolerance, 100);
   }

   public InvertedFourBarJointIKBinarySolver(double tolerance, int maxIterations)
   {
      this.tolerance = tolerance;
      this.maxIterations = maxIterations;
   }

   public void setUseNaiveMethod(boolean useNaiveMethod)
   {
      this.useNaiveMethod = useNaiveMethod;
   }

   @Override
   public void setConverters(FourBarToJointConverter[] converters)
   {
      this.converters = converters;
   }

   private double lastTheta = Double.NaN;
   private double thetaMin = Double.NaN;
   private double thetaMax = Double.NaN;
   private FourBarVertex lastVertexToSolveFor = null;
   private double lastSolution = Double.NaN;

   @Override
   public double solve(double theta, FourBarVertex vertexToSolveFor)
   {
      if (!Double.isNaN(lastSolution))
      { // Check if the user is requesting for the same configuration as the last that was solved.
         if (vertexToSolveFor == lastVertexToSolveFor && EuclidCoreTools.epsilonEquals(lastTheta, theta, tolerance))
            return lastSolution;
      }

      if (!FourBarTools.isFourBarInverted(vertexToSolveFor))
         throw new IllegalArgumentException("The given vertex does not belong to an inverted four bar.");

      lastTheta = theta;
      lastVertexToSolveFor = vertexToSolveFor;

      double angle = isThetaAtLimit(theta, vertexToSolveFor);

      if (!Double.isNaN(angle))
      {
         lastSolution = angle;
      }
      else
      {
         if (useNaiveMethod)
            lastSolution = solveNaive(theta, vertexToSolveFor);
         else
            lastSolution = solveInternal(theta, vertexToSolveFor);
      }

      return lastSolution;
   }

   private double isThetaAtLimit(double theta, FourBarVertex vertexToSolveFor)
   {
      FourBarVertex A = vertexToSolveFor;
      boolean isNextCrossing = A.getNextEdge().isCrossing();
      FourBarVertex B = isNextCrossing ? A.getNextVertex() : A.getPreviousVertex();

      double minAngleA = convert(A, A.getMinAngle());
      double maxAngleA = convert(A, A.getMaxAngle());

      thetaMin = minAngleA + convert(B, B.getMinAngle());
      thetaMax = maxAngleA + convert(B, B.getMaxAngle());

      limitsInverted = thetaMin > thetaMax;

      if (limitsInverted)
      {
         thetaMin = -thetaMin;
         thetaMax = -thetaMax;
      }

      if (A.isConvex() == isNextCrossing)
      {
         if (theta <= thetaMin + tolerance)
            return minAngleA;
         else if (theta >= thetaMax - tolerance)
            return maxAngleA;
      }
      else
      {
         if (-theta <= thetaMin + tolerance)
            return minAngleA;
         else if (-theta >= thetaMax - tolerance)
            return maxAngleA;
      }

      return Double.NaN;
   }

   private double solveNaive(double theta, FourBarVertex vertexToSolveFor)
   {
      long start = System.nanoTime();
      setupFourBar(vertexToSolveFor);

      FourBarToJointConverter converterA = null;
      FourBarToJointConverter converterB = null;
      FourBarToJointConverter converterD = null;

      if (converters != null)
      {
         converterA = getConverter(vertexToSolveFor);
         converterB = getConverter(vertexToSolveFor.getNextVertex());
         converterD = getConverter(vertexToSolveFor.getPreviousVertex());
      }

      FourBarVertex A = fourBar.getVertexA();
      double minDAB = A.getMinAngle();
      double maxDAB = A.getMaxAngle();

      double currentDAB = Double.NaN;

      // The search uses the property that theta is the sum of the angles of the two vertices that are the end-points of a crossing edge.
      // Note that there are 2 sets of such vertices, while the sum of a first set is equal to theta, the other is equal to -theta.
      if (A.isConvex() != A.getNextEdge().isCrossing())
      {
         if (DEBUG)
            LogTools.debug("Flip {}", vertexToSolveFor.getName());
         theta = -theta;
      }

      for (iterations = 0; iterations < maxIterations; iterations++)
      {
         currentDAB = 0.5 * (minDAB + maxDAB);
         fourBar.update(FourBarAngle.DAB, currentDAB);
         double actualTheta;
         if (converterA == null)
            actualTheta = currentDAB;
         else
            actualTheta = converterA.toJointAngle(currentDAB);

         if (fourBar.getEdgeDA().isCrossing())
         { // theta = DAB + CDA
            actualTheta += convert(converterD, fourBar.getAngleCDA());
         }
         else
         { // theta = DAB + ABC
            actualTheta += convert(converterB, fourBar.getAngleABC());
         }

         if (limitsInverted)
            actualTheta = -actualTheta;

         if (EuclidCoreTools.epsilonEquals(actualTheta, theta, tolerance))
            break;

         if (actualTheta > theta)
         {
            maxDAB = currentDAB;
            thetaMax = actualTheta;
         }
         else
         {
            minDAB = currentDAB;
            thetaMin = actualTheta;
         }

         if (Math.abs(thetaMax - thetaMin) <= tolerance)
            break;
      }

      if (DEBUG)
         LogTools.debug("Iterations: {}, time elapsed: {}millisec", iterations, (System.nanoTime() - start) / 1.0e6);
      return convert(converterA, currentDAB);
   }

   private void setupFourBar(FourBarVertex vertexToSolverFor)
   {
      FourBarVertex A = vertexToSolverFor;
      FourBarVertex B = A.getNextVertex();
      FourBarVertex C = B.getNextVertex();
      FourBarVertex D = C.getNextVertex();
      fourBar.setup(A.getNextEdge().getLength(),
                    B.getNextEdge().getLength(),
                    C.getNextEdge().getLength(),
                    D.getNextEdge().getLength(),
                    A.isConvex(),
                    B.isConvex(),
                    C.isConvex(),
                    D.isConvex());
   }

   private double solveInternal(double theta, FourBarVertex vertexToSolveFor)
   {
      long start = System.nanoTime();
      FourBarVertex A = vertexToSolveFor;
      FourBarEdge ABEdge = A.getNextEdge();
      FourBarEdge BCEdge = ABEdge.getNext();
      FourBarEdge CDEdge = BCEdge.getNext();
      FourBarEdge DAEdge = CDEdge.getNext();
      double AB = ABEdge.getLength();
      double BC = BCEdge.getLength();
      double CD = CDEdge.getLength();
      double DA = DAEdge.getLength();

      double minDAB = A.getMinAngle();
      double maxDAB = A.getMaxAngle();

      FourBarToJointConverter converterA = null;
      FourBarToJointConverter converterB = null;
      FourBarToJointConverter converterD = null;

      if (converters != null)
      {
         converterA = getConverter(vertexToSolveFor);
         converterB = getConverter(vertexToSolveFor.getNextVertex());
         converterD = getConverter(vertexToSolveFor.getPreviousVertex());
      }

      // The search uses the property that theta is the sum of the angles of the two vertices that are the end-points of a crossing edge.
      // Note that there are 2 sets of such vertices, while the sum of a first set is equal to theta, the other is equal to -theta.
      if (vertexToSolveFor.isConvex() != ABEdge.isCrossing())
      {
         if (DEBUG)
            LogTools.debug("Flip " + vertexToSolveFor.getName());
         theta = -theta;
      }

      double currentDAB = Double.NaN;

      /*
       * TODO: Ideally we want to avoid relying onto Math.acos(...) that is really expensive. When only
       * using the cosine values to do the search, the algorithm works almost all the time, but at
       * occasion it fails pretty bad. There's obviously some edge-case that is not properly considered
       * and I couldn't figure it out yet, so for now it is safer to compare angles directly.
       */

      for (iterations = 0; iterations < maxIterations; iterations++)
      {
         currentDAB = 0.5 * (minDAB + maxDAB);

         double otherAngle;

         double BD = EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(AB, DA, currentDAB);

         if (DAEdge.isCrossing())
         { // theta = DAB + CDA
            double cosADB = FourBarTools.cosineAngleWithCosineLaw(DA, BD, AB);
            double cosCDB = FourBarTools.cosineAngleWithCosineLaw(CD, BD, BC);
            // Using property: acos(x) - acos(y) = acos(xy + sqrt((1 - xx)(1 - yy))
            // angleCDA
            otherAngle = Math.abs(Math.acos(cosADB * cosCDB + Math.sqrt((1.0 - cosADB * cosADB) * (1.0 - cosCDB * cosCDB))));
            if (!DAEdge.getStart().isConvex())
               otherAngle = -otherAngle;
            otherAngle = convert(converterD, otherAngle);
         }
         else
         { // theta = DAB + ABC
            double cosABD = FourBarTools.cosineAngleWithCosineLaw(AB, BD, DA);
            double cosCBD = FourBarTools.cosineAngleWithCosineLaw(BC, BD, CD);
            // Using property: acos(x) - acos(y) = acos(xy + sqrt((1 - xx)(1 - yy))
            // angleABC
            otherAngle = Math.abs(Math.acos(cosABD * cosCBD + Math.sqrt((1.0 - cosABD * cosABD) * (1.0 - cosCBD * cosCBD))));
            if (!BCEdge.getStart().isConvex())
               otherAngle = -otherAngle;
            otherAngle = convert(converterB, otherAngle);
         }

         double actualTheta = convert(converterA, currentDAB) + otherAngle;

         if (limitsInverted)
            actualTheta = -actualTheta;

         if (EuclidCoreTools.epsilonEquals(actualTheta, theta, tolerance))
            break;

         if (actualTheta > theta)
         {
            maxDAB = currentDAB;
            thetaMax = actualTheta;
         }
         else
         {
            minDAB = currentDAB;
            thetaMin = actualTheta;
         }

         if (Math.abs(thetaMax - thetaMin) <= tolerance)
            break;
      }

      if (DEBUG)
         LogTools.debug("Iterations: {}, time elapsed: {}millisec", iterations, (System.nanoTime() - start) / 1.0e6);
      return convert(converterA, currentDAB);
   }

   private double convert(FourBarVertex vertex, double vertexAngle)
   {
      return convert(getConverter(vertex), vertexAngle);
   }

   private double convert(FourBarToJointConverter converter, double vertexAngle)
   {
      return converter == null ? vertexAngle : converter.toJointAngle(vertexAngle);
   }

   private FourBarToJointConverter getConverter(FourBarVertex vertex)
   {
      return converters == null ? null : converters[vertex.getFourBarAngle().ordinal()];
   }
}
