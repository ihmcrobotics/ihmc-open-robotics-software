package us.ihmc.gdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVCodecContext;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.global.avcodec;
import org.bytedeco.ffmpeg.global.avformat;
import org.bytedeco.ffmpeg.global.avutil;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;

public class FFMPEGFileReader
{
   private AVFormatContext avFormatContext;

   private int streamIndex;
   private AVCodecContext decoderContext;
   public FFMPEGFileReader(String file) {
      LogTools.info("Initializing ffmpeg contexts for playback from {}", file);
      avFormatContext = new AVFormatContext();
      FFMPEGTools.checkNonZeroError(avformat.avformat_open_input(avFormatContext, file, null, null), "Initializing format context");

      FFMPEGTools.checkNonZeroError(avformat.avformat_find_stream_info(avFormatContext, (AVDictionary) null), "Finding stream information");

      openCodecContext();

      AVStream stream = avFormatContext.streams(streamIndex);

      LogTools.debug("FILE PROPERTIES: Width: {}\tHeight: {}\tFormat:{}",
                     decoderContext.width(), decoderContext.height(), avutil.av_get_pix_fmt_name(decoderContext.pix_fmt()).getString());

      avformat.av_dump_format(avFormatContext, 0, file, 0);
   }

   //Adapted from demuxing_decoding.c. Currently assumes video stream, but could be adapted for audio use, too
   private void openCodecContext()
   {
      int ret;

      if ((ret = avformat.av_find_best_stream(avFormatContext, avutil.AVMEDIA_TYPE_VIDEO,
                                             -1, -1, (AVCodec) null, 0)) < 0)
      {
         FFMPEGTools.checkNonZeroError(ret, "Finding video stream");
      }

      streamIndex = ret;
      AVStream stream = avFormatContext.streams(streamIndex);
      AVCodec decoder = avcodec.avcodec_find_decoder(stream.codecpar().codec_id());
      FFMPEGTools.checkPointer(decoder, "Finding codec");

      decoderContext = avcodec.avcodec_alloc_context3(decoder);
      FFMPEGTools.checkPointer(decoderContext, "Allocating decoder context");

      FFMPEGTools.checkNonZeroError(avcodec.avcodec_parameters_to_context(decoderContext, stream.codecpar()), "Copying codec parameters to decoder context");
      FFMPEGTools.checkNonZeroError(avcodec.avcodec_open2(decoderContext, decoder, (AVDictionary) null), "Opening codec");
   }

   public boolean hasNextFrame() {
      return false;
   }

   public void getNextFrame(BytedecoImage image) {

   }
}
