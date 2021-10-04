package us.ihmc.sensorProcessing.heightMap;

import us.ihmc.tools.property.StoredPropertySet;

public class HeightMapParameters extends StoredPropertySet
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

   public double getGridResolutionXY()
   {
      return get(HeightMapParameterKeys.gridResolutionXY);
   }

   public double getGridSizeXY()
   {
      return get(HeightMapParameterKeys.gridSizeXY);
   }

   public double getMaxZ()
   {
      return get(HeightMapParameterKeys.maxZ);
   }

   public double getNominalVariance()
   {
      return get(HeightMapParameterKeys.nominalVariance);
   }

   public int getMaxPointsPerCell()
   {
      return get(HeightMapParameterKeys.maxPointsPerCell);
   }

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
