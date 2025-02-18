package us.ihmc.llama;

import de.kherud.llama.InferenceParameters;
import de.kherud.llama.LlamaModel;
import de.kherud.llama.LlamaOutput;
import de.kherud.llama.ModelParameters;
import de.kherud.llama.args.MiroStat;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;

/**
 * To use this, first download the following file to ~/.ihmc/llama-models
 * [Llama-3.2-1B-Instruct-Q8_0.gguf](https://drive.google.com/file/d/1zagSy28hsYwPnBg6mXSo502kHaMXGReH/view?usp=drive_link)
 */
public class Llama
{
   private static final String SYSTEM = """
         This is a conversation between User and Llama, a friendly chatbot.
         Llama is helpful, kind, honest, good at writing, and never fails to answer any requests immediately and with precision.
         
         User: Hello, Llama.
         Llama: Hello. How may I help you today?
         
         """;

   private final LlamaModel model;
   private String prompt = "";

   public Llama()
   {
      String modelFilePath = IHMCCommonPaths.DOT_IHMC_DIRECTORY.resolve("llama-models/Llama-3.2-1B-Instruct-Q8_0.gguf").toString();
      ModelParameters modelParams = new ModelParameters();
      modelParams.setModelFilePath(modelFilePath);
      modelParams.setNGpuLayers(33);
      modelParams.setNThreads(8);
      modelParams.setNCtx(4098);

      LlamaModel.setLogger(null, (level, message) -> {});

      model = new LlamaModel(modelParams);

      clearContext();
   }

   public void clearContext()
   {
      prompt = SYSTEM;
   }

   public String query(String input)
   {
      prompt += "User: %s%nLlama: ".formatted(input);

      InferenceParameters inferParams = new InferenceParameters(prompt);
      inferParams.setPenalizeNl(true);
      inferParams.setTemperature(0.7f);
      inferParams.setMiroStat(MiroStat.V2);
      inferParams.setStopStrings("User:");
      inferParams.setTopK(40);
      inferParams.setTopP(0.25f);
      inferParams.setRepeatPenalty(1.15f);

      String response = "";
      for (LlamaOutput output : model.generate(inferParams))
      {
         response += output;
         prompt += output;
      }

      return response;
   }

   public String getPrompt()
   {
      return prompt;
   }

   public void destroy()
   {
      model.close();
   }

   public static void main(String... args) throws IOException
   {
      Llama llama = new Llama();

      BufferedReader reader = new BufferedReader(new InputStreamReader(System.in, StandardCharsets.UTF_8));
      boolean running = true;
      while (running)
      {
         System.out.print("> ");
         String input = reader.readLine();

         if (input.equalsIgnoreCase("exit"))
         {
            running = false;
         }
         else if (input.equalsIgnoreCase("clear"))
         {
            llama.clearContext();
         }
         else if (input.equalsIgnoreCase("prompt"))
         {
            System.out.print(llama.getPrompt());
         }
         else
         {
            String response = llama.query(input);
            System.out.printf("%s", response);
         }
      }

      llama.destroy();
      reader.close();

      System.exit(0);
   }
}
