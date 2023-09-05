package us.ihmc.rdx.tools.assimp;

import org.apache.commons.io.IOUtils;
import org.apache.commons.lang3.mutable.MutableLong;
import org.lwjgl.assimp.*;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.resources.ResourceTools;
import us.ihmc.tools.string.StringTools;

import java.net.URL;
import java.nio.ByteBuffer;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Supplier;

public class AssimpOpenedFile
{
   private final long assimpFileStructAddress;
   private String fileName;

   public AssimpOpenedFile(long assimpFileIOAddress, long fileNameAddress, long openModeAddress)
   {
      fileName = MemoryUtil.memUTF8(fileNameAddress);
      String openMode = MemoryUtil.memUTF8(openModeAddress);
      LogTools.debug("{}: Opening in mode: {}", fileName, openMode);

      Path filePath = Paths.get(fileName);
      URL url = ResourceTools.getResourceSystem(filePath);
      if (url == null)
      {
         Supplier<String> message = StringTools.format("Could not get resource: {}", filePath);
         LogTools.error(message);
         throw new RuntimeException(message.get());
      }

      byte[] byteArray = ExceptionTools.handle(() -> IOUtils.toByteArray(url), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

      final MutableLong position = new MutableLong(0);

      AIFile assimpFile = AIFile.create();

      AIFileReadProcI assimpFileReadFunction = new AIFileReadProcI()
      {
         /** This gets invoked once to find if the file exists, then again later for reading. */
         @Override
         public long invoke(long assimpFileAddress, long characterBufferToReadToAddress, long size, long count)
         {
            long desiredNumberOfBytesToRead = size * count;
            long bytesLeftInFile = byteArray.length - position.getValue();
            long bytesToBeRead = Math.min(desiredNumberOfBytesToRead, bytesLeftInFile);

            LogTools.debug(StringTools.format("{}: Read size: {} count: {} desired bytes: {} bytes left: {} going to be read: {}", fileName, size, count, desiredNumberOfBytesToRead, bytesLeftInFile, bytesToBeRead));

            ByteBuffer characterBufferToReadTo = MemoryUtil.memByteBuffer(characterBufferToReadToAddress, byteArray.length);
            characterBufferToReadTo.put(byteArray, (int) position.longValue(), (int) bytesToBeRead);

            return bytesToBeRead;
         }
      };
      AIFileWriteProcI assimpFileWriteFunction = new AIFileWriteProcI()
      {
         @Override
         public long invoke(long assimpFileAddress, long characterBufferToWriteAddress, long memB, long count)
         {
            LogTools.debug("{}: Write", fileName);
            return 0;
         }
      };
      AIFileTellProcI assimpFileTellFunction = new AIFileTellProcI()
      {
         @Override
         public long invoke(long assimpFileAddress)
         {
            LogTools.debug("{}: Tell", fileName);
            return 0;
         }
      };
      AIFileTellProcI assimpFileSizeFunction = new AIFileTellProcI()
      {
         @Override
         public long invoke(long assimpFileAddress)
         {
            long size = byteArray.length;
            LogTools.debug("{}: Size: {}", fileName, size);
            return size;
         }
      };
      AIFileSeekI assimpFileSeekFunction = new AIFileSeekI()
      {
         @Override
         public int invoke(long assimpFileAddress, long offset, int origin)
         {
            LogTools.debug("{}: Seek offset: {} origin: {}", fileName, offset, origin);
            if (origin == Assimp.aiOrigin_SET)
            {
               position.setValue(offset);
            }
            else if (origin == Assimp.aiOrigin_CUR)
            {
               position.add(offset);
            }
            else if (origin == Assimp.aiOrigin_END)
            {
               position.setValue(byteArray.length - offset);
            }
            return Assimp.aiReturn_FAILURE;
         }
      };
      AIFileFlushProcI assimpFileFlushFunction = new AIFileFlushProcI()
      {
         @Override
         public void invoke(long assimpFileAddress)
         {
            LogTools.debug("{}: Flush", fileName);
         }
      };

      assimpFile.ReadProc(assimpFileReadFunction);
      assimpFile.WriteProc(assimpFileWriteFunction);
      assimpFile.TellProc(assimpFileTellFunction);
      assimpFile.FileSizeProc(assimpFileSizeFunction);
      assimpFile.SeekProc(assimpFileSeekFunction);
      assimpFile.FlushProc(assimpFileFlushFunction);
      assimpFile.UserData(MemoryUtil.memAddress(MemoryUtil.memAlloc(1)));

      assimpFileStructAddress = assimpFile.address();
   }

   public long getAssimpFileStructAddress()
   {
      return assimpFileStructAddress;
   }
}
