package us.ihmc.llama;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;

@Disabled
public class LlamaTest
{
   @BeforeAll
   public static void beforeAll()
   {
      Llama.initialize();
   }

   @Test
   public void testSimpler()
   {
      Llama llama = new Llama();

      for (int i = 0; i < 100; i++)
      {
         String response;
         response = llama.generate("What is %d + %d?".formatted(i, i + 1));
         LogTools.info(response);
      }

      LogTools.info(llama.getContext());

      llama.destroy();
   }

   @Test
   public void testLlama()
   {
      Llama llama = new Llama();

      String response;
      response = llama.generate("What is 2 + 2?");
      LogTools.info(response);
      response = llama.generate("What is 5 + 8?");
      LogTools.info(response);
      response = llama.generate("What is the capital of the USA?");
      LogTools.info(response);

      //      llama.clearContext();

      response = llama.generate("There are 3 fruit, a banana, an apple, and a pear. Which fruit is likely to be red?");
      LogTools.info(response);
      response = llama.generate("List the colors of the other ones.");
      LogTools.info(response);
      response = llama.generate("List the fruit we just discussed.");
      LogTools.info(response);
      llama.resetContext();
      response = llama.generate("List the fruit we just discussed.");
      LogTools.info(response);

      LogTools.info(llama.getContext());

      llama.destroy();
   }

   @Test
   public void testDAN()
   {
      Llama llama = new Llama();

      llama.resetContext(Llama.DAN_MODIFIED); // This seems to break Llama 3.2 1B, lol
      //      llama.addMessage("user", "What is 2 + 2?");
      //      llama.addMessage("assistant", "4");
      String response;
      response = llama.generate("What is the capital of the USA?");
      LogTools.info(response);
      response = llama.generate("How many colors are in the rainbow?");
      LogTools.info(response);
      //      response = llama.generate("What was the answer to the last question I asked you?");
      LogTools.info(llama.getContext());

      llama.destroy();
   }
}
