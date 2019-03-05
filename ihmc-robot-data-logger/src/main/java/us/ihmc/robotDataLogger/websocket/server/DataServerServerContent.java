package us.ihmc.robotDataLogger.websocket.server;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.UUID;
import java.util.stream.Collectors;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import io.netty.util.CharsetUtil;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.AnnouncementPubSubType;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakePubSubType;

/**
 *
 * This class holds all the static content that is available on the HTTP server.
 * 
 * This includes the index page, announcement, handshake, model and resource zip.
 * 
 * It is cached and ready to server with minimal object allocations.
 * 
 * @author Jesper Smith
 *
 */
class DataServerServerContent
{
   private final String name;
   private final String hostName;

   private final ByteBuf announcementBuffer;
   private final ByteBuf handshakeBuffer;
   private final ByteBuf index;

   private final ByteBuf model;
   private final ByteBuf resourceZip;

   public DataServerServerContent(Announcement announcement, Handshake handshake, LogModelProvider logModelProvider)
   {
      try
      {
         this.name = announcement.getNameAsString();
         this.hostName = announcement.getHostNameAsString();

         announcement.setIdentifier(UUID.randomUUID().toString());
         if (logModelProvider != null)
         {
            byte[] model = logModelProvider.getModel();
            announcement.getModelFileDescription().setHasModel(true);
            announcement.getModelFileDescription().setName(logModelProvider.getModelName());
            announcement.getModelFileDescription().setModelLoaderClass(logModelProvider.getLoader().getCanonicalName());
            announcement.getModelFileDescription().setModelFileSize(model.length);
            for (String resourceDirectory : logModelProvider.getResourceDirectories())
            {
               announcement.getModelFileDescription().getResourceDirectories().add(resourceDirectory);
            }

            this.model = Unpooled.wrappedBuffer(model);

            byte[] resourceZip = logModelProvider.getResourceZip();
            if (resourceZip != null && resourceZip.length > 0)
            {

               this.resourceZip = Unpooled.wrappedBuffer(resourceZip);
               announcement.getModelFileDescription().setHasResourceZip(true);
               announcement.getModelFileDescription().setResourceZipSize(resourceZip.length);
            }
            else
            {
               announcement.getModelFileDescription().setHasResourceZip(false);
               this.resourceZip = null;
            }
         }
         else
         {
            announcement.getModelFileDescription().setHasModel(false);
            this.model = null;
            this.resourceZip = null;
         }

         AnnouncementPubSubType announcementPubSubType = new AnnouncementPubSubType();
         JSONSerializer<Announcement> announcementSerializer = new JSONSerializer<Announcement>(announcementPubSubType);
         byte[] announcementData = announcementSerializer.serializeToBytes(announcement);
         announcementBuffer = Unpooled.directBuffer(announcementData.length);
         announcementBuffer.writeBytes(announcementData);

         HandshakePubSubType handshakeType = new HandshakePubSubType();
         JSONSerializer<Handshake> handshakeSerializer = new JSONSerializer<Handshake>(handshakeType);
         byte[] handshakeData = handshakeSerializer.serializeToBytes(handshake);
         handshakeBuffer = Unpooled.directBuffer(handshakeData.length);
         handshakeBuffer.writeBytes(handshakeData);

         index = createIndex();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public ByteBuf getAnnouncement()
   {
      return announcementBuffer.retainedDuplicate();
   }

   public String getAnnouncementContentType()
   {
      return "application/json; charset=UTF-8";
   }

   public ByteBuf getHandshake()
   {
      return handshakeBuffer.retainedDuplicate();
   }

   public String getHandshakeContentType()
   {
      return "application/json; charset=UTF-8";
   }

   public ByteBuf getIndex()
   {
      return index.retainedDuplicate();
   }

   public String getIndexContentType()
   {
      return "text/html; charset=UTF-8";
   }

   public boolean hasModel()
   {
      return model != null;
   }

   public ByteBuf getModel()
   {
      return model.retainedDuplicate();
   }

   public String getModelContentType()
   {
      return "text/xml; charset=UTF-8";
   }

   public boolean hasResourceZip()
   {
      return resourceZip != null;
   }

   public ByteBuf getResourceZip()
   {
      return resourceZip.retainedDuplicate();
   }

   public String getResourceZipContentType()
   {
      return "application/zip";
   }

   private ByteBuf createIndex()
   {
      InputStream is = getClass().getResourceAsStream("index.html");
      if (is != null)
      {
         BufferedReader reader = new BufferedReader(new InputStreamReader(is));
         String index = reader.lines().collect(Collectors.joining(System.lineSeparator()));

         index = index.replace("{NAME}", name);
         index = index.replace("{HOSTNAME}", hostName);

         return Unpooled.copiedBuffer(index, CharsetUtil.UTF_8);

      }
      else
      {
         throw new RuntimeException("Cannot load index.html");
      }

   }

}
