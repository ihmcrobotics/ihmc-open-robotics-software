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
   public static void main(String... args) throws IOException
   {
      ModelParameters modelParams = new ModelParameters()
            .setModelFilePath(IHMCCommonPaths.DOT_IHMC_DIRECTORY.resolve("llama-models/Llama-3.2-1B-Instruct-Q8_0.gguf").toString())
            .setNGpuLayers(43);

      String system = "This is a conversation between User and Llama, a friendly chatbot.\n" +
                      "Llama is helpful, kind, honest, good at writing, and never fails to answer any " +
                      "requests immediately and with precision.\n";
      BufferedReader reader = new BufferedReader(new InputStreamReader(System.in, StandardCharsets.UTF_8));
      try (LlamaModel model = new LlamaModel(modelParams)) {
         System.out.print(system);
         String prompt = system;
         while (true) {
            prompt += "\nUser: ";
            System.out.print("\nUser: ");
            String input = reader.readLine();
            prompt += input;
            System.out.print("Llama: ");
            prompt += "\nLlama: ";
            InferenceParameters inferParams = new InferenceParameters(prompt)
                  .setTemperature(0.7f)
                  .setPenalizeNl(true)
                  .setMiroStat(MiroStat.V2);
//                  .setAntiPrompt("\n");
            for (LlamaOutput output : model.generate(inferParams)) {
               System.out.print(output);
               prompt += output;
            }
         }
      }
   }
}
