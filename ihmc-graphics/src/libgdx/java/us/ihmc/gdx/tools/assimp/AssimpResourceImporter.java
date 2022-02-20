package us.ihmc.gdx.tools.assimp;

import org.apache.logging.log4j.Level;
import org.lwjgl.assimp.*;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.log.LogTools;

public class AssimpResourceImporter
{
   public AIScene importScene(String resourcePath)
   {
      LogTools.debug("Using assimp {}.{}", Assimp.aiGetVersionMajor(), Assimp.aiGetVersionMinor());

      int postProcessingSteps = 0; // none
      postProcessingSteps += Assimp.aiProcess_FlipUVs;

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

      AIScene assimpScene = Assimp.aiImportFileEx(resourcePath, postProcessingSteps, assimpFileIO);

      if (assimpScene == null)
      {
         LogTools.error(Assimp.aiGetErrorString());
      }

      return assimpScene;
   }
}
