package us.ihmc.gdx.tools.assimp;

import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import org.apache.logging.log4j.Level;
import org.lwjgl.assimp.*;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.log.LogTools;

public class AssimpResourceImporter
{
   public static boolean ASSIMP_INITIAL_SETUP = false;

   public AIScene importScene(String resourcePath)
   {
      ensureAssimpInitialSetup();

      AIPropertyStore assimpPropertyStore = Assimp.aiCreatePropertyStore();

      int postProcessingSteps = 0; // none

      /** libGDX reads UVs flipped from assimp default */
      postProcessingSteps += Assimp.aiProcess_FlipUVs;

      /** libGDX needs triangles */
      postProcessingSteps += Assimp.aiProcess_Triangulate;

      /** libGDX has limits in MeshBuilder.
       *  Not sure if there is a triangle limit.
       */
      Assimp.aiSetImportPropertyInteger(assimpPropertyStore, Assimp.AI_CONFIG_PP_SLM_VERTEX_LIMIT, MeshBuilder.MAX_VERTICES);
//      Assimp.aiSetImportPropertyInteger(assimpPropertyStore, Assimp.AI_CONFIG_PP_SLM_TRIANGLE_LIMIT, MeshBuilder.MAX_VERTICES);
      postProcessingSteps += Assimp.aiProcess_SplitLargeMeshes;

//      postProcessingSteps += Assimp.aiProcess_OptimizeGraph;
//      postProcessingSteps += Assimp.aiProcess_OptimizeMeshes;
//      postProcessingSteps += Assimp.aiProcess_JoinIdenticalVertices;

      AIFileOpenProcI assimpFileOpenFunction = new AIFileOpenProcI()
      {
         /**
          * There might be several opens for one load.
          * There is a CanRead check before everything else.
          * Then it will open just to check the file size.
          * Then it will load to actually read it in.
          */
         @Override
         public long invoke(long assimpFileIOAddress, long fileNameAddress, long openModeAddress)
         {
            AssimpOpenedFile assimpOpenedFile = new AssimpOpenedFile(assimpFileIOAddress, fileNameAddress, openModeAddress);
            return assimpOpenedFile.getAssimpFileStructAddress();
         }
      };
      AIFileCloseProcI assimpFileCloseFunction = new AIFileCloseProcI()
      {
         @Override
         public void invoke(long assimpFileIOAddress, long assimpFileAddress)
         {
            LogTools.debug("Closing");
         }
      };
      AIFileIO assimpFileIO = AIFileIO.create();
      assimpFileIO.OpenProc(assimpFileOpenFunction);
      assimpFileIO.CloseProc(assimpFileCloseFunction);
      assimpFileIO.UserData(MemoryUtil.memAddress(MemoryUtil.memAlloc(1)));

      AIScene assimpScene = Assimp.aiImportFileExWithProperties(resourcePath, postProcessingSteps, assimpFileIO, assimpPropertyStore);

      if (assimpScene == null)
      {
         LogTools.error(Assimp.aiGetErrorString());
      }

      return assimpScene;
   }

   private static void ensureAssimpInitialSetup()
   {
      if (!ASSIMP_INITIAL_SETUP)
      {
         ASSIMP_INITIAL_SETUP = true;

         LogTools.debug("Using assimp {}.{}", Assimp.aiGetVersionMajor(), Assimp.aiGetVersionMinor());

         Assimp.aiEnableVerboseLogging(LogTools.getLevel().isLessSpecificThan(Level.DEBUG));

         AILogStreamCallbackI assimpLogStreamCallback = new AILogStreamCallbackI()
         {
            @Override
            public void invoke(long messageAddress, long userDataAddress)
            {
               String messageString = MemoryUtil.memUTF8(messageAddress);
               LogTools.debug("[Assimp] {}", messageString.trim());
            }
         };

         AILogStream assimpLogStream = AILogStream.create();
         assimpLogStream.callback(assimpLogStreamCallback);
         assimpLogStream.user(MemoryUtil.memAddress(MemoryUtil.memAlloc(1)));
         Assimp.aiAttachLogStream(assimpLogStream);
      }
   }
}
