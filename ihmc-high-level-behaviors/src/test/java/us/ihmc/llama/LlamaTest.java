package us.ihmc.llama;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import us.ihmc.log.LogTools;

@Disabled
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class LlamaTest
{
   static Llama llama;

   @BeforeAll
   public static void beforeAll()
   {
      Llama.initialize();
      llama = new Llama();
   }

   @AfterAll
   public static void afterAll()
   {
      llama.destroy();
   }

   @BeforeEach
   public void beforeEach()
   {
      llama.resetContext(Llama.CHAT_WITH_LLAMA);
   }

   @Test
   @Order(1)
   public void testSimpler()
   {
      for (int i = 0; i < 100; i++)
      {
         String response;
         response = llama.generate("What is %d + %d?".formatted(i, i + 1));
         LogTools.info(response);
      }

      LogTools.info(llama.getContext());
   }

   @Test
   @Order(2)
   public void testLlama()
   {
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
   }

   @Test
   @Order(3)
   public void testDAN()
   {
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
   }
}
