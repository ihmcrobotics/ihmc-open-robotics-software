package us.ihmc.simulationConstructionSetTools.util.ground;

import java.io.File;
import java.nio.file.Path;

import com.jme3.system.NativeLibraryLoader;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.PathTools;
import vhacd4.Vhacd4Parameters;

public final class MeshTerrainObjectParameters
{
   static
   {
      Path scsCachePath = PathTools.systemTemporaryDirectory().resolve("SCSCache");

      FileTools.ensureDirectoryExists(scsCachePath, DefaultExceptionHandler.PRINT_STACKTRACE);

      System.setProperty("java.library.path", System.getProperty("java.library.path") + File.pathSeparator + scsCachePath.toString());
      NativeLibraryLoader.setCustomExtractionFolder(scsCachePath.toString());

   }

   private int maximumNumberOfHulls = 32;
   private int maximumNumberOfVerticesPerHull = 64;
   private int maximumVoxelResolution = 400000;
   private double maximumVolumetricPercentError = 0.01;

   private boolean showOriginalMeshGraphics = true;
   private boolean showDecomposedMeshGraphics = false;
   private boolean doConvexDecomposition = true;
   
   private Vhacd4Parameters vhacd4Parameters;

   private String modelDirectory;

   public MeshTerrainObjectParameters()
   {
      this(null);
   }

   public MeshTerrainObjectParameters(String modelDirectory)
   {
      this.modelDirectory = modelDirectory;

      NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
      updateParameters();
   }

   public void updateParameters()
   {
      this.vhacd4Parameters = new Vhacd4Parameters();

      // Setting VHACD4 parameters
      this.vhacd4Parameters.setMaxVerticesPerHull(this.maximumNumberOfVerticesPerHull);
      this.vhacd4Parameters.setVoxelResolution(this.maximumVoxelResolution);
      this.vhacd4Parameters.setVolumePercentError(this.maximumVolumetricPercentError);
      this.vhacd4Parameters.setMaxHulls(this.maximumNumberOfHulls);
      
   }

   public String getModelDirectory()
   {
      return modelDirectory;
   }

   public int getMaxNoOfHulls()
   {
      return maximumNumberOfHulls;
   }

   public void setMaxNoOfHulls(int maxNoOfHulls)
   {
      this.maximumNumberOfHulls = maxNoOfHulls;
   }

   public int getMaxNoOfVertices()
   {
      return maximumNumberOfVerticesPerHull;
   }

   public void setMaxNoOfVertices(int maxNoOfVertices)
   {
      this.maximumNumberOfVerticesPerHull = maxNoOfVertices;
   }

   public double getMaxVolumePercentError()
   {
      return maximumVolumetricPercentError;
   }

   public void setMaxVolumePercentError(double maxVolumePercentError)
   {
      this.maximumVolumetricPercentError = maxVolumePercentError;
   }

   public int getVoxelResolution()
   {
      return maximumVoxelResolution;
   }

   public void setVoxelResolution(int voxelResolution)
   {
      this.maximumVoxelResolution = voxelResolution;
   }

   public boolean isShowUndecomposedMeshGraphics()
   {
      return showOriginalMeshGraphics;
   }

   public void setShowUndecomposedMeshGraphics(boolean showUndecomposedMeshGraphics)
   {
      this.showOriginalMeshGraphics = showUndecomposedMeshGraphics;
   }

   public boolean isShowDecomposedMeshGraphics()
   {
      return showDecomposedMeshGraphics;
   }

   public void setShowDecomposedMeshGraphics(boolean showDecomposedMeshGraphics)
   {
      this.showDecomposedMeshGraphics = showDecomposedMeshGraphics;
   }

   public boolean isDoConvexDecomposition()
   {
      return doConvexDecomposition;
   }

   public void setDoConvexDecomposition(boolean doConvexDecomposition)
   {
      this.doConvexDecomposition = doConvexDecomposition;
   }

   public Vhacd4Parameters getVhacd4Parameters()
   {
      updateParameters();
      return vhacd4Parameters;
   }

   public void setVhacd4Parameters(Vhacd4Parameters vhacd4Parameters)
   {
      this.vhacd4Parameters = vhacd4Parameters;
   }


}
