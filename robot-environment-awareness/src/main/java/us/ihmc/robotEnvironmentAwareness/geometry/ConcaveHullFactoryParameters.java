package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.tools.property.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main to regenerate.
 */
public class ConcaveHullFactoryParameters extends StoredPropertySet implements ConcaveHullFactoryParametersBasics
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "robot-environment-awareness/src/main/resources";
   public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "robot-environment-awareness/src/main/java";

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

   public ConcaveHullFactoryParameters(String versionSpecifier)
   {
      super(keys, ConcaveHullFactoryParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, versionSpecifier);
      load();
   }

   public ConcaveHullFactoryParameters(StoredPropertySetReadOnly other)
   {
      super(keys, ConcaveHullFactoryParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys,
                                                           ConcaveHullFactoryParameters.class,
                                                           DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                           SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
   }
}
