package us.ihmc.perception.segmentationTools;

import java.util.Scanner;

import us.ihmc.jOctoMap.tools.ScannerTools;
import us.ihmc.perception.geometry.SimpleConcaveHullFactory;
import us.ihmc.tools.property.*;

public class PolygonizerParameters extends StoredPropertySet implements PolygonizerParametersBasics
{
   public static final String PROJECT_NAME = "ihmc-open-robotics-software";
   public static final String TO_RESOURCE_FOLDER = "robot-environment-awareness/src/main/resources";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   /**
    * Threshold used when creating a new concave hull with {@link SimpleConcaveHullFactory}.
    * <p>
    * Uses the Duckham and al. (2008) algorithm defined in the paper untitled "Efficient generation of
    * simple polygons for characterizing the shape of a set of points in the plane".
    */
   public static final DoubleStoredPropertyKey concaveHullThreshold = keys.addDoubleKey("Concave hull threshold");
   /**
    * The minimum number of nodes required for a {@link PlanarRegionSegmentationNodeData} to be
    * polygonized.
    */
   public static final IntegerStoredPropertyKey minNumberOfNodes = keys.addIntegerKey("Min number of nodes");
   /**
    * Filter parameter on the concave hull of a region. Used to removed vertices describing shallow
    * angle.
    */
   public static final DoubleStoredPropertyKey shallowAngleThreshold = keys.addDoubleKey("Shallow angle threshold");
   /** Filter parameter on the concave hull of a region. Used to removed vertices that create peaks. */
   public static final DoubleStoredPropertyKey peakAngleThreshold = keys.addDoubleKey("Peak angle threshold");
   /** Filter parameter on the concave hull of a region. Used to removed short edges. */
   public static final DoubleStoredPropertyKey lengthThreshold = keys.addDoubleKey("Length threshold");
   /**
    * Threshold used for decomposing the concave hull into convex polygons. Describes the maximum depth
    * of a concavity before the concave hull gets split in 2.
    */
   public static final DoubleStoredPropertyKey depthThreshold = keys.addDoubleKey("Depth threshold");
   /**
    * Filter for splitting concave hulls at any narrow passage which width is less than
    * {@code 2 * lengthThreshold}.
    */
   public static final BooleanStoredPropertyKey cutNarrowPassage = keys.addBooleanKey("Cut narrow passage");

   public PolygonizerParameters()
   {
      this("");
   }

   public PolygonizerParameters(String fileNameSuffix)
   {
      super(keys, PolygonizerParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER, fileNameSuffix);
      load();
   }

   public PolygonizerParameters(PolygonizerParameters other)
   {
      super(keys, PolygonizerParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
      set(other);
   }

   public void set(PolygonizerParameters other)
   {
      setAll(other.getAll());
   }

   @Override
   public String toString()
   {
      return "concaveHullThreshold: " + concaveHullThreshold + ", minNumberOfNodes: " + minNumberOfNodes + ", shallowAngleThreshold: " + shallowAngleThreshold
            + ", peakAngleThreshold: " + peakAngleThreshold + ", lengthThreshold: " + lengthThreshold + ", depthThreshold: " + depthThreshold
            + ", cutNarrowPassage: " + cutNarrowPassage;
   }

   public void setFromString(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      setConcaveHullThreshold(ScannerTools.readNextDouble(scanner, getConcaveHullThreshold()));
      setMinNumberOfNodes(ScannerTools.readNextInt(scanner, getMinNumberOfNodes()));
      setShallowAngleThreshold(ScannerTools.readNextDouble(scanner, getShallowAngleThreshold()));
      setPeakAngleThreshold(ScannerTools.readNextDouble(scanner, getPeakAngleThreshold()));
      setLengthThreshold(ScannerTools.readNextDouble(scanner, getLengthThreshold()));
      setDepthThreshold(ScannerTools.readNextDouble(scanner, getDepthThreshold()));
      setCutNarrowPassage(ScannerTools.readNextBoolean(scanner, getCutNarrowPassage()));
      scanner.close();
   }

   public static PolygonizerParameters parse(String parametersAsString)
   {
      PolygonizerParameters parameters = new PolygonizerParameters();
      parameters.setFromString(parametersAsString);
      return parameters;
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys, PolygonizerParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
      parameters.loadUnsafe();
      parameters.save();
   }
}
