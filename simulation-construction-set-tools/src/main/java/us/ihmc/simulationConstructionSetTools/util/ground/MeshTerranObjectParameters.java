package us.ihmc.simulationConstructionSetTools.util.ground;

import java.io.File;
import java.nio.file.Path;

import javax.swing.UIManager;

import org.lwjgl.glfw.GLFW;

import com.jme3.system.NativeLibraryLoader;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.PathTools;
import vhacd.VHACDParameters;
import vhacd4.Vhacd4Parameters;

public class MeshTerranObjectParameters
{
   static
   {
      Path scsCachePath = PathTools.systemTemporaryDirectory().resolve("SCSCache");

      FileTools.ensureDirectoryExists(scsCachePath, DefaultExceptionHandler.PRINT_STACKTRACE);

      System.setProperty("java.library.path", System.getProperty("java.library.path") + File.pathSeparator + scsCachePath.toString());
      NativeLibraryLoader.setCustomExtractionFolder(scsCachePath.toString());

   }
   
   private int maximumNumberOfHulls = 20;
   private int maximumNumberOfVerticesPerHull = 64;
   private double maximumVolumePercentError = 0.01;
   private int maximumVoxelResolution = 100000;
   private double maximumConvacity = 0.5;

   private boolean showOriginalMeshGraphics = true;
   private boolean showDecomposedMeshGraphics = false;

   private VHACDParameters vhacdParameters;
   private Vhacd4Parameters vhacd4Parameters;

   public enum ConvexDecomposition
   {
      VHACD, VHACD4, NO_DECOMPOSITION
   };

   private ConvexDecomposition decompositionType = ConvexDecomposition.VHACD4;

   public MeshTerranObjectParameters()
   {
      NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
      updateParameters();
   }

   public void updateParameters()
   {
      this.vhacdParameters = new VHACDParameters();
      this.vhacd4Parameters = new Vhacd4Parameters();
      
      // Setting VHACD4 parameters
      this.vhacd4Parameters.setMaxVerticesPerHull(this.maximumNumberOfVerticesPerHull);
      this.vhacd4Parameters.setVoxelResolution(this.maximumVoxelResolution);
      this.vhacd4Parameters.setVolumePercentError(this.maximumVolumePercentError);
      this.vhacd4Parameters.setMaxHulls(this.maximumNumberOfHulls);

      // Setting VHACD parameters
      this.vhacdParameters.setMaxConcavity(this.maximumConvacity);
      this.vhacdParameters.setMaxVerticesPerHull(this.maximumNumberOfVerticesPerHull);
      this.vhacdParameters.setVoxelResolution(this.maximumVoxelResolution);
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
      return maximumVolumePercentError;
   }

   public void setMaxVolumePercentError(double maxVolumePercentError)
   {
      this.maximumVolumePercentError = maxVolumePercentError;
   }

   public int getVoxelResolution()
   {
      return maximumVoxelResolution;
   }

   public void setVoxelResolution(int voxelResolution)
   {
      this.maximumVoxelResolution = voxelResolution;
   }

   public double getMaxConvacity()
   {
      return maximumConvacity;
   }

   public void setMaxConvacity(double maxConvacity)
   {
      this.maximumConvacity = maxConvacity;
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

   public VHACDParameters getVhacdParameters()
   {
      updateParameters();
      return vhacdParameters;
   }

   public void setVhacdParameters(VHACDParameters vhacdParameters)
   {
      this.vhacdParameters = vhacdParameters;
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

   public ConvexDecomposition getDecompositionType()
   {
      return decompositionType;
   }

   public void setDecompositionType(ConvexDecomposition decompositionType)
   {
      this.decompositionType = decompositionType;
   }

}
