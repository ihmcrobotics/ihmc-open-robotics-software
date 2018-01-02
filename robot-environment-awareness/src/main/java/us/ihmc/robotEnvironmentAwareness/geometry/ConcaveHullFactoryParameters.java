package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.Scanner;

import us.ihmc.jOctoMap.tools.ScannerTools;

public class ConcaveHullFactoryParameters
{
   private double edgeLengthThreshold;
   private boolean removeAllTrianglesWithTwoBorderEdges;
   private boolean allowSplittingConcaveHull;
   private int maxNumberOfIterations;

   public ConcaveHullFactoryParameters()
   {
      setDefaultParameters();
   }

   public ConcaveHullFactoryParameters(ConcaveHullFactoryParameters other)
   {
      set(other);
   }

   public void set(ConcaveHullFactoryParameters other)
   {
      edgeLengthThreshold = other.edgeLengthThreshold;
      removeAllTrianglesWithTwoBorderEdges = other.removeAllTrianglesWithTwoBorderEdges;
      allowSplittingConcaveHull = other.allowSplittingConcaveHull;
      maxNumberOfIterations = other.maxNumberOfIterations;
   }

   public void setDefaultParameters()
   {
      edgeLengthThreshold = 0.10;
      removeAllTrianglesWithTwoBorderEdges = true;
      allowSplittingConcaveHull = true;
      maxNumberOfIterations = 5000;
   }

   public double getEdgeLengthThreshold()
   {
      return edgeLengthThreshold;
   }

   public boolean doRemoveAllTrianglesWithTwoBorderEdges()
   {
      return removeAllTrianglesWithTwoBorderEdges;
   }

   public boolean isSplittingConcaveHullAllowed()
   {
      return allowSplittingConcaveHull;
   }

   public int getMaxNumberOfIterations()
   {
      return maxNumberOfIterations;
   }

   public void setEdgeLengthThreshold(double edgeLengthThreshold)
   {
      this.edgeLengthThreshold = edgeLengthThreshold;
   }

   public void setRemoveAllTrianglesWithTwoBorderEdges(boolean removeAllTrianglesWithTwoBorderEdges)
   {
      this.removeAllTrianglesWithTwoBorderEdges = removeAllTrianglesWithTwoBorderEdges;
   }

   public void setAllowSplittingConcaveHull(boolean allowSplittingConcaveHull)
   {
      this.allowSplittingConcaveHull = allowSplittingConcaveHull;
   }

   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      this.maxNumberOfIterations = maxNumberOfIterations;
   }

   @Override
   public String toString()
   {
      return "edge length threshold: " + edgeLengthThreshold
            + ", remove any triangle with two borderedges: " + removeAllTrianglesWithTwoBorderEdges
            + ", allow splitting concave hull: " + allowSplittingConcaveHull
            + ", maximum number of iterations: " + maxNumberOfIterations;
   }

   public static ConcaveHullFactoryParameters parse(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      parameters.setEdgeLengthThreshold(ScannerTools.readNextDouble(scanner, parameters.getEdgeLengthThreshold()));
      parameters.setRemoveAllTrianglesWithTwoBorderEdges(ScannerTools.readNextBoolean(scanner, parameters.doRemoveAllTrianglesWithTwoBorderEdges()));
      parameters.setAllowSplittingConcaveHull(ScannerTools.readNextBoolean(scanner, parameters.isSplittingConcaveHullAllowed()));
      parameters.setMaxNumberOfIterations(ScannerTools.readNextInt(scanner, parameters.getMaxNumberOfIterations()));
      scanner.close();
      return parameters;
   }
}
