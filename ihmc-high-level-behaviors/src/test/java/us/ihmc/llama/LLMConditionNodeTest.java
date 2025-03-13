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
public class LLMConditionNodeTest
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
   public void testCounting()
   {
      String system = """
            You are an assistant to help me count. Each time I ask you to count, state
            the current count and also say "failure" or "success, reset".
            
            "success, reset" will be reported for any count after the provided limit.
            
            Here is an example for a limit of 3:
            User: Count. (limit 3)
            Assistant: The current count is 1. failure
            User: Count. (limit 3)
            Assistant: The current count is 2. failure
            User: Count. (limit 3)
            Assistant: The current count is 3. failure
            User: Count. (limit 3)
            Assistant: The current count is 4. success, reset
            User: Count. (limit 3)
            Assistant: The current count is 5. success, reset
            """;

      llama.resetContext(system);

      String response;
      response = llama.generate("Count. (limit 5)");
      LogTools.info(response);
      response = llama.generate("Count. (limit 5)");
      LogTools.info(response);
      response = llama.generate("Count. (limit 5)");
      LogTools.info(response);
      response = llama.generate("Count. (limit 5)");
      LogTools.info(response);
      response = llama.generate("Count. (limit 5)");
      LogTools.info(response);
      response = llama.generate("Count. (limit 5)");
      LogTools.info(response);
      response = llama.generate("Count. (limit 5)");
      LogTools.info(response);
      response = llama.generate("Count. (limit 5)");
      LogTools.info(response);
      response = llama.generate("Count. (limit 5)");
      LogTools.info(response);

      LogTools.info(llama.getContext());
   }

   @Test
   @Order(2)
   public void testRiddle()
   {
      String response;
      response = llama.generate("""
                                The code is the number of legs on a spider multiplied by the number of days in a week,
                                minus the number of seasons in a year.
                                """);
      LogTools.info(response);

   }
}
