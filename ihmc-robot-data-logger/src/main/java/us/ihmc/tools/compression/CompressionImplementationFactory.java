package us.ihmc.tools.compression;

/**
 * Singleton factory for the choosen compression implementation.
 * 
 * @author Jesper Smith
 *
 */
public class CompressionImplementationFactory
{
   private static CompressionImplementation instance = null;

   public static synchronized CompressionImplementation instance()
   {
      if (instance == null)
      {
         instance = new LZ4CompressionImplementation();
//         instance = new CopyCompressionImplementation();
      }
      return instance;
   }
   
   /**
    * Disallow construction
    */
   private CompressionImplementationFactory()
   {
      
   }

}
