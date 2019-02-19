package us.ihmc.robotDataLogger.logger;

/**
 * General settings for the data server
 * 
 * @author Jesper Smith
 *
 */
public class DataServerSettings
{
   public static final int DEFAULT_PORT = 8008;
   public static final boolean DEFAULT_AUTODISCOVERABLE = true;

   private boolean logSession;
   private String videoStream;
   private int port;
   private boolean autoDiscoverable;

   /**
    * Settings for the data server
    * 
    * Intialized to the default port with autoDiscoverable set to true. 
    * 
    * @param logSession A running logger on the network will log this session to disk
    * 
    */
   public DataServerSettings(boolean logSession)
   {
      this(logSession, null);
   }

   /**
    * Settings for the data server
    * 
    * Initialized to the default port with autoDiscoverable set to true.
    * 
    * @param logSession A running logger on the network will log this session to disk
    * @param videoStreamIdentifier Identifier for the GUI video stream
    */
   public DataServerSettings(boolean logSession, String videoStreamIdentifier)
   {
      this(logSession, DEFAULT_AUTODISCOVERABLE, DEFAULT_PORT, videoStreamIdentifier);
   }

   /**
    * Settings for the data server
    * 
    * Initialized to the default port and without video stream identifier 
    * 
    * @param logSession A running logger on the network will log this session to disk
    * @param autoDiscoverable The existence of the session is broadcast over the network so clients can find it without knowing the IP beforehand 
    * 
    */
   public DataServerSettings(boolean logSession, boolean autoDiscoverable)
   {
      this(logSession, autoDiscoverable, DEFAULT_PORT, null);
   }

   /**
    * Settings for the data server
    * 
    * @param logSession A running logger on the network will log this session to disk
    * @param autoDiscoverable The existence of the session is broadcast over the network so clients can find it without knowing the IP beforehand 
    * @param port Port to listen on (default: 8080)
    * @param videoStreamIdentifier Optional identifier for the GUI video stream
    */
   public DataServerSettings(boolean logSession, boolean autoDiscoverable, int port, String videoStreamIdentifier)
   {
      this.logSession = logSession;
      this.videoStream = videoStreamIdentifier;
      this.port = port;
      this.autoDiscoverable = autoDiscoverable;
   }

   public boolean isLogSession()
   {
      return logSession;
   }

   public String getVideoStream()
   {
      return videoStream;
   }

   public int getPort()
   {
      return port;
   }

   public void setPort(int port)
   {
      this.port = port;
   }

   public boolean isAutoDiscoverable()
   {
      return autoDiscoverable;
   }

   public void setAutoDiscoverable(boolean autoDiscoverable)
   {
      this.autoDiscoverable = autoDiscoverable;
   }

   public void setLogSession(boolean log)
   {
      this.logSession = log;
   }

   public void setVideoStream(String videoStream)
   {
      this.videoStream = videoStream;
   }

}
