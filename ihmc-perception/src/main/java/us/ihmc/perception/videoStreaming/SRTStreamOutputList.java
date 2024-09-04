package us.ihmc.perception.videoStreaming;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import us.ihmc.perception.ffmpeg.FFMPEGTools;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static org.bytedeco.ffmpeg.global.avformat.avformat_alloc_output_context2;
import static org.bytedeco.ffmpeg.global.avformat.avformat_free_context;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_copy;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_free;

public class SRTStreamOutputList
{
   private final String outputFormat;
   private final AVDictionary ioOptions;
   private final AVDictionary formatOptions;

   /** AVFormatContext used as an example for initializing codecs. Cannot be used to perform libav functions. */
   private final AVFormatContext exampleFormatContext;

   private final List<SRTStreamOutput> outputList = new ArrayList<>();

   int error;

   public SRTStreamOutputList(String outputFormat, AVDictionary ioOptions, AVDictionary formatOptions)
   {
      this.outputFormat = outputFormat;

      // Copy the IO options
      this.ioOptions = new AVDictionary();
      error = av_dict_copy(this.ioOptions, ioOptions, 0);
      FFMPEGTools.checkError(error, this.ioOptions, "Copying IO options");

      // Copy the format options
      this.formatOptions = new AVDictionary();
      error = av_dict_copy(this.formatOptions, formatOptions, 0);
      FFMPEGTools.checkError(error, this.ioOptions, "Copying format options");

      exampleFormatContext = new AVFormatContext();
      error = avformat_alloc_output_context2(exampleFormatContext, null, outputFormat, null);
      FFMPEGTools.checkError(error, exampleFormatContext, "Allocating output format context");
   }

   public void addOutput(String outputAddress, int outputPort)
   {
      outputList.add(new SRTStreamOutput(outputAddress, outputPort, outputFormat, ioOptions, formatOptions));
   }

   public List<SRTStreamOutput> getOutputList()
   {
      return outputList;
   }

   public int size()
   {
      return outputList.size();
   }

   public AVFormatContext getExampleFormatContext()
   {
      return exampleFormatContext;
   }

   public void writeToAll(AVPacket packetToWrite)
   {
      boolean writeSucceeded;

      Iterator<SRTStreamOutput> streamOutputIterator = outputList.iterator();
      while (streamOutputIterator.hasNext())
      {
         SRTStreamOutput streamOutput = streamOutputIterator.next();
         writeSucceeded = streamOutput.write(packetToWrite);
         if (!writeSucceeded)
         {
            streamOutput.close();
            streamOutputIterator.remove();
         }
      }
   }

   public void close()
   {
      av_dict_free(ioOptions);
      ioOptions.close();

      av_dict_free(formatOptions);
      formatOptions.close();

      avformat_free_context(exampleFormatContext);
      exampleFormatContext.close();

      for (SRTStreamOutput streamOutput : outputList)
      {
         streamOutput.close();
      }
   }
}
