package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.Scanner;

import us.ihmc.jOctoMap.tools.ScannerTools;
import us.ihmc.tools.property.*;

public class ConcaveHullFactoryParameters extends StoredPropertySet implements ConcaveHullFactoryParametersBasics
{
   public static final String PROJECT_NAME = "ihmc-open-robotics-software";
   public static final String TO_RESOURCE_FOLDER = "robot-environment-awareness/src/main/resources";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey edgeLengthThreshold = keys.addDoubleKey("Edge length threshold");
   public static final BooleanStoredPropertyKey removeAllTrianglesWithTwoBorderEdges = keys.addBooleanKey("Remove all triangles with two border edges");
   public static final BooleanStoredPropertyKey allowSplittingConcaveHull = keys.addBooleanKey("Allow splitting concave hull");
   public static final IntegerStoredPropertyKey maxNumberOfIterations = keys.addIntegerKey("Max number of iterations");
   public static final DoubleStoredPropertyKey triangulationTolerance = keys.addDoubleKey("Triangulation tolerance");

   public ConcaveHullFactoryParameters()
   {
      this("");
   }

   public ConcaveHullFactoryParameters(String fileNameSuffix)
   {
      super(keys, ConcaveHullFactoryParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER, fileNameSuffix);
      load();
   }

   public ConcaveHullFactoryParameters(ConcaveHullFactoryParameters other)
   {
      super(keys, ConcaveHullFactoryParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
      set(other);
   }

   public void set(ConcaveHullFactoryParameters other)
   {
      setAll(other.getAll());
   }

   @Override
   public String toString()
   {
      return "edge length threshold: " + edgeLengthThreshold + ", remove any triangle with two borderedges: " + removeAllTrianglesWithTwoBorderEdges
            + ", allow splitting concave hull: " + allowSplittingConcaveHull + ", maximum number of iterations: " + maxNumberOfIterations
            + ", triangulation tolerance: " + triangulationTolerance;
   }

   public void setFromString(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      setEdgeLengthThreshold(ScannerTools.readNextDouble(scanner, getEdgeLengthThreshold()));
      setRemoveAllTrianglesWithTwoBorderEdges(ScannerTools.readNextBoolean(scanner, doRemoveAllTrianglesWithTwoBorderEdges()));
      setAllowSplittingConcaveHull(ScannerTools.readNextBoolean(scanner, isSplittingConcaveHullAllowed()));
      setMaxNumberOfIterations(ScannerTools.readNextInt(scanner, getMaxNumberOfIterations()));
      setTriangulationTolerance(ScannerTools.readNextDouble(scanner, getTriangulationTolerance()));
      scanner.close();
   }

   public static ConcaveHullFactoryParameters parse(String parametersAsString)
   {
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      parameters.setFromString(parametersAsString);
      return parameters;
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys, ConcaveHullFactoryParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
      parameters.loadUnsafe();
      parameters.save();
   }
}
