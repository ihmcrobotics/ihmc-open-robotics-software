package us.ihmc.simulationconstructionset;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.PushbackInputStream;

/**
 * This class is a drop-in replacement for DataInputStream. It exposes all of the same public interface,
 * and only possesses one "additional" method: <pre>readASCIILine()</pre>. This method is an exact duplicate
 * of the <pre>readLine()</pre> method from DataInputStream.  It only has a new name because the original
 * method is final and so can't be overridden.  This class was created to get around the deprecation issues
 * with the original method. Should the old method be removed from DataInputStream in future releases, we will
 * be able to retain the functionality.
 * 
 * @author Doug Stephen
 * @version 1.0, Added July 5 2012
 */
public class YoDataInputStream extends DataInputStream
{
   
   private char lineBuffer[];

   public YoDataInputStream(InputStream in)
   {
      super(in);
   }

   /**
    * This is a direct copy of the deprecated method <pre>readLine()</pre>
    * from DataInputStream.  It has been renamed to <pre>readASCIILine()</pre>
    * because of the <pre>final</pre> status of the method, as it can't be overridden.
    * 
    * This method is only in place to preserve the functionality of the readLine method
    * in spite of its deprecation.  Cursory analysis indicates that the only reason the 
    * is deprecated is that it doesn't gracefully handle UTF encoded data; because all of
    * our text data is encoded in ASCII and because our randomly generated unit tests
    * routinely pass, we will hold to the idea that this method satisfies our needs and
    * that we shouldn't worry about the deprecation issue.
    * 
    * We have created this method in order to continue using its functionality in the event
    * that the deprecation leads to the code being removed from future JDK/JRE releases.
    * 
    * @return the next ASCII formatted line from the stream
    * @throws IOException
    */
   public final String readASCIILine() throws IOException
   {
      char buf[] = lineBuffer;

      if (buf == null)
      {
         buf = lineBuffer = new char[128];
      }

      int room = buf.length;
      int offset = 0;
      int c;

      loop: while (true)
      {
         switch (c = in.read())
         {
         case -1:
         case '\n':
            break loop;

         case '\r':
            int c2 = in.read();
            if ((c2 != '\n') && (c2 != -1))
            {
               if (!(in instanceof PushbackInputStream))
               {
                  this.in = new PushbackInputStream(in);
               }
               ((PushbackInputStream) in).unread(c2);
            }
            break loop;

         default:
            if (--room < 0)
            {
               buf = new char[offset + 128];
               room = buf.length - offset - 1;
               System.arraycopy(lineBuffer, 0, buf, 0, offset);
               lineBuffer = buf;
            }
            buf[offset++] = (char) c;
            break;
         }
      }
      if ((c == -1) && (offset == 0))
      {
         return null;
      }
      return String.copyValueOf(buf, 0, offset);
   }

}
