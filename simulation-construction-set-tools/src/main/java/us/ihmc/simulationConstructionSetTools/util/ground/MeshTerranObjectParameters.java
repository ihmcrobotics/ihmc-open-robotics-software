package us.ihmc.simulationConstructionSetTools.util.ground;

import com.jme3.system.NativeLibraryLoader;

import vhacd.VHACDParameters;
import vhacd4.Vhacd4Parameters;

public class MeshTerranObjectParameters
{
   private int maxNoOfHulls = 20;
   private int maxNoOfVerticesPerHull = 64;
   private double maxVolumePercentError = 0.01;
   private int voxelResolution = 100000;
   private double maxConvacity = 0.5;

   private boolean showUndecomposedMeshGraphics = true;
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
      updateParameters();
   }

   public void updateParameters()
   {
      NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
      this.vhacdParameters = new VHACDParameters();
      this.vhacd4Parameters = new Vhacd4Parameters();
      
      // Setting VHACD4 parameters
      this.vhacd4Parameters.setMaxVerticesPerHull(this.maxNoOfVerticesPerHull);
      this.vhacd4Parameters.setVoxelResolution(this.voxelResolution);
      this.vhacd4Parameters.setVolumePercentError(this.maxVolumePercentError);
      this.vhacd4Parameters.setMaxHulls(this.maxNoOfHulls);

      // Setting VHACD parameters
      this.vhacdParameters.setMaxConcavity(this.maxConvacity);
      this.vhacdParameters.setMaxVerticesPerHull(this.maxNoOfVerticesPerHull);
      this.vhacdParameters.setVoxelResolution(this.voxelResolution);
   }

   public int getMaxNoOfHulls()
   {
      return maxNoOfHulls;
   }

   public void setMaxNoOfHulls(int maxNoOfHulls)
   {
      this.maxNoOfHulls = maxNoOfHulls;
   }

   public int getMaxNoOfVertices()
   {
      return maxNoOfVerticesPerHull;
   }

   public void setMaxNoOfVertices(int maxNoOfVertices)
   {
      this.maxNoOfVerticesPerHull = maxNoOfVertices;
   }

   public double getMaxVolumePercentError()
   {
      return maxVolumePercentError;
   }

   public void setMaxVolumePercentError(double maxVolumePercentError)
   {
      this.maxVolumePercentError = maxVolumePercentError;
   }

   public int getVoxelResolution()
   {
      return voxelResolution;
   }

   public void setVoxelResolution(int voxelResolution)
   {
      this.voxelResolution = voxelResolution;
   }

   public double getMaxConvacity()
   {
      return maxConvacity;
   }

   public void setMaxConvacity(double maxConvacity)
   {
      this.maxConvacity = maxConvacity;
   }

   public boolean isShowUndecomposedMeshGraphics()
   {
      return showUndecomposedMeshGraphics;
   }

   public void setShowUndecomposedMeshGraphics(boolean showUndecomposedMeshGraphics)
   {
      this.showUndecomposedMeshGraphics = showUndecomposedMeshGraphics;
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
      return vhacdParameters;
   }

   public void setVhacdParameters(VHACDParameters vhacdParameters)
   {
      this.vhacdParameters = vhacdParameters;
   }

   public Vhacd4Parameters getVhacd4Parameters()
   {
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
