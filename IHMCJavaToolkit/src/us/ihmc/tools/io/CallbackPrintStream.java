package us.ihmc.tools.io;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.util.regex.Pattern;

/**
 * 
 * Simple helper class to redirect PrintStreams to a function. 
 * 
 * This class will call outputLine for every line printed to this PrintStream. 
 * All newlines characters are stripped.
 *
 *
 * @author Jesper Smith
 */
public abstract class CallbackPrintStream extends PrintStream
{   
   private final static Pattern newlinePattern = Pattern.compile("\\r?\\n");

   
   public CallbackPrintStream()
   {
      super(new FlushingOutputStream(), true);
      
      ((FlushingOutputStream) out).setCallback(this);
   }
   
   private static class FlushingOutputStream extends ByteArrayOutputStream
   {  
      private CallbackPrintStream callbackPrintStream;
      
      @Override
      public synchronized void flush()
      {
         if(count > 0 && buf[count -1] == '\n')
         {
            String[] messages = newlinePattern.split(toString());
            for(String message : messages)
            {
               callbackPrintStream.outputLine(message);
            }
            reset();
         }
      }

      public void setCallback(CallbackPrintStream callbackPrintStream)
      {
         this.callbackPrintStream = callbackPrintStream;
      }
      
   }

   /**
    * Callback for messages written to this printstream. Gets called on every new line delimited by a newline character.
    * 
    * @param message is a single line of output written to this PrintStream. Newline characters are stripped.
    */
   public abstract void outputLine(String message);
}
