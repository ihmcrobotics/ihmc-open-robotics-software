package us.ihmc.sensorProcessing.heightMap;

import us.ihmc.robotics.heightMap.HeightMapParametersReadOnly;
import us.ihmc.tools.property.StoredPropertySet;

public class HeightMapParameters extends StoredPropertySet implements HeightMapParametersReadOnly
{
   public static final String PROJECT_NAME = "ihmc-open-robotics-software";
   public static final String PATH_TO_RESOURCES = "ihmc-sensor-processing/src/main/resources";

   public HeightMapParameters()
   {
      this("");
   }

   public HeightMapParameters(String fileNameSuffix)
   {
      this(PROJECT_NAME, PATH_TO_RESOURCES, fileNameSuffix);
   }

   public HeightMapParameters(String projectName, String pathToResources)
   {
      this(projectName, pathToResources, "");
   }

   public HeightMapParameters(String projectName, String pathToResources, String fileNameSuffix)
   {
      super(HeightMapParameterKeys.keys, HeightMapParameters.class, projectName, pathToResources, fileNameSuffix);
      loadUnsafe();
   }

   /**
    * Resolution of the height map grid
    */
   @Override
   public double getGridResolutionXY()
   {
      return get(HeightMapParameterKeys.gridResolutionXY);
   }

   /**
    * Length of the side of the square height map grid
    */
   @Override
   public double getGridSizeXY()
   {
      return get(HeightMapParameterKeys.gridSizeXY);
   }

   /**
    * Max z relative to robot mid foot z. Points above this threshold are ignored.
    */
   @Override
   public double getMaxZ()
   {
      return get(HeightMapParameterKeys.maxZ);
   }

   /**
    * When calibrated on flat ground, this is the average standard deviation observed for a grid cell.
    */
   @Override
   public double getNominalStandardDeviation()
   {
      return get(HeightMapParameterKeys.nominalStandardDeviation);
   }

   @Override
   public int getMaxPointsPerCell()
   {
      return get(HeightMapParameterKeys.maxPointsPerCell);
   }

   /**
    * If a grid cell is at height h, points below (h - s * m) are ignored, and points above (h + s * m) will cause the cell to throw out old data and reset.
    * where s is getNominalStandardDeviation() and m is this value.
    */
   @Override
   public double getMahalanobisScale()
   {
      return get(HeightMapParameterKeys.mahalonobisScale);
   }

   /**
    * Use this to update and fix the INI file
    */
   public static void main(String[] args)
   {
      StoredPropertySet storedPropertySet = new StoredPropertySet(HeightMapParameterKeys.keys, HeightMapParameters.class, PROJECT_NAME, PATH_TO_RESOURCES);
      storedPropertySet.loadUnsafe();
      storedPropertySet.save();
   }
}
