package us.ihmc.robotEnvironmentAwareness.planarRegion;

import us.ihmc.tools.property.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main to regenerate.
 */
public class PolygonizerParameters extends StoredPropertySet implements PolygonizerParametersBasics
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "robot-environment-awareness/src/main/resources";
   public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "robot-environment-awareness/src/main/java";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   /**
    * Threshold used when creating a new concave hull with {@link
    * us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory}. Uses the
    * Duckham and al. (2008) algorithm defined in the paper entitled "Efficient
    * generation of simple polygons for characterizing the shape of a set of points in
    * the plane".
    */
   public static final DoubleStoredPropertyKey concaveHullThreshold = keys.addDoubleKey("Concave hull threshold");
   /**
    * The minimum number of nodes required for a {@link
    * us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationNodeData}
    * to be polygonized.
    */
   public static final IntegerStoredPropertyKey minNumberOfNodes = keys.addIntegerKey("Min number of nodes");
   /**
    * Filter parameter on the concave hull of a region. Used to removed vertices
    * describing shallow angle.
    */
   public static final DoubleStoredPropertyKey shallowAngleThreshold = keys.addDoubleKey("Shallow angle threshold");
   /**
    * Filter parameter on the concave hull of a region. Used to removed vertices that
    * create peaks.
    */
   public static final DoubleStoredPropertyKey peakAngleThreshold = keys.addDoubleKey("Peak angle threshold");
   /**
    * Filter parameter on the concave hull of a region. Used to removed short edges.
    */
   public static final DoubleStoredPropertyKey lengthThreshold = keys.addDoubleKey("Length threshold");
   /**
    * Threshold used for decomposing the concave hull into convex polygons. Describes
    * the maximum depth of a concavity before the concave hull gets split in 2.
    */
   public static final DoubleStoredPropertyKey depthThreshold = keys.addDoubleKey("Depth threshold");
   /**
    * Filter for splitting concave hulls at any narrow passage which width is less
    * than 2 * lengthThreshold.
    */
   public static final BooleanStoredPropertyKey cutNarrowPassage = keys.addBooleanKey("Cut narrow passage");

   public PolygonizerParameters()
   {
      this("");
   }

   public PolygonizerParameters(String versionSpecifier)
   {
      super(keys, PolygonizerParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, versionSpecifier);
      load();
   }

   public PolygonizerParameters(StoredPropertySetReadOnly other)
   {
      super(keys, PolygonizerParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys,
                                                           PolygonizerParameters.class,
                                                           DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                           SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
   }
}
